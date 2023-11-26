from threading import Timer
from orchestrator.shared import *
from orchestrator.utils import get_broadcast_ip, prompt


class Server:
    """
    Server class that handles the communication between the master node and the
    other nodes.
    """

    def __init__(self, socket):
        self.__broadcast_ip = get_broadcast_ip()
        self.__master_node = None
        self.__is_running = False
        self.__nodes = dict()
        self.__socket = socket
        self.__ordered_nodes = set()

    def stop_node_timers(self):
        """
        Stops all node timers
        """
        for node in self.__nodes.keys():
            self.__nodes[node].cancel()

    def print_nodes(self):
        """
        Prints all connected nodes
        """
        if len(self.__nodes) == 0:
            print("No connected nodes")
            return

        for idx, node in enumerate(self.__nodes):
            if node == self.__master_node:
                print(
                    f"{idx}: {node} (master)")
                continue

            print(f"{idx}: {node}")

    def remove_node(self, ip):
        if ip not in self.__nodes.keys():
            return

        del self.__nodes[ip]
        print(f"** Node {ip} removed from list of nodes due to timeout. **")
        prompt()

    def handle_message(self, msg, addr):
        """
        Handles incoming messages from the nodes. The messages 'msg' are in the
        format of [command, data]
        """
        cmd = msg[0]
        data = "" if len(msg) == 1 else msg[1]
        ip = addr[0]
        if cmd == MSG_CMD_HEARTBEAT:
            if ip in self.__nodes.keys():
                self.__nodes[ip].cancel()
                self.__nodes[ip] = Timer(
                    HEARTBEAT_TIMEOUT, self.remove_node, args=[ip])
                self.__nodes[ip].start()
                return

            # Set a timer for that specific node address to be removed if it times out after HEARTBEAT_TIMEOUT seconds
            self.__nodes[ip] = Timer(
                HEARTBEAT_TIMEOUT, self.remove_node, args=[ip])
            self.__nodes[ip].start()

            print(f"** Registered node {ip} to list of nodes **")
        elif cmd == MSG_CMD_START_CONFIRM:
            print(f"** Node {ip} started **")
            if self.__master_node == ip:
                print("Starting slaves")
                self.__socket.sendto(
                    str.encode(MSG_CMD_START),
                    (self.__broadcast_ip, SOCKET_PORT)
                )
        elif cmd == MSG_CMD_STOP_CONFIRM:
            print(f"** Node {ip} stopped **")
        elif cmd == MSG_CMD_UPDATE_START_CONFIRM:
            print(f"** Node {ip} started update **")
        elif cmd == MSG_CMD_UPDATE_CONFIRM:
            print(f"** Node {ip} updated **")
            self.__nodes[ip] = Timer(
                HEARTBEAT_TIMEOUT, self.remove_node, args=[ip])
            self.__nodes[ip].start()
        elif cmd == MSG_CMD_MASTER_CONFIRM:
            print(f"** Node {ip} set to master **")
            self.__master_node = ip

            # Assign order to vehicles
            current_id = 1
            for node in self.__nodes.keys():
                if node == self.__master_node:
                    vehicle_id = 0
                else:
                    vehicle_id = current_id
                    current_id += 1

                self.__socket.sendto(
                    str.encode(f"{MSG_CMD_ORDER}|{vehicle_id}"),
                    (node, SOCKET_PORT)
                )
        elif cmd == MSG_CMD_NOT_MASTER_CONFIRM:
            print(f"** Node {ip} set to slave **")
        elif cmd == MSG_CMD_ORDER_CONFIRM:
            print(f"** Node {ip} assigned id {data} **")
            self.__ordered_nodes.add(ip)
        elif cmd == MSG_CMD_DEBUG_CONFIRM:
            print(f"** Node {ip} set debug mode {data} **")
        elif cmd == MSG_CMD_DEBUG_MSG:
            print(data)
        elif cmd == MSG_CMD_ERROR:
            print(f"\n[{ip}] \033[2;31m{data}\033[0;0m")
        else:
            # Only update prompt if we actually print something
            return

        prompt()

    def handle_user_cmd(self, msg):
        """
        Handles user input commands and sends them to the nodes via UDP
        broadcast.
        """
        cmd = msg[0]
        data = "" if len(msg) == 1 else msg[1]

        if cmd not in AVAILABLE_COMMANDS:
            print(f"Unsupported command: {cmd}")
            return

        if cmd == MSG_CMD_SET_MASTER:
            if len(data) == 0 or len(self.__nodes) == 0:
                print("Invalid address, see registered nodes with 'ls'")
                return

            ips = list(self.__nodes.keys())
            node = ips[0]
            if "." in data:
                node = data
            else:
                try:
                    val = int(data)
                    if val >= len(self.__nodes):
                        print(
                            "Node index out of range, see registered nodes with 'ls'")
                        return
                    node = ips[val]
                except:
                    print("Invalid node index, see registered nodes with 'ls'")
                    return

            self.__socket.sendto(
                str.encode(f"{cmd}|{node}"),
                (self.__broadcast_ip, SOCKET_PORT)
            )
        elif cmd == MSG_CMD_UPDATE:
            if len(data) == 0 or " " in data:
                print("Invalid branch name")
                return

            self.stop_node_timers()
            self.__socket.sendto(
                str.encode(f"{cmd}|{data}"),
                (self.__broadcast_ip, SOCKET_PORT)
            )
        elif cmd == MSG_CMD_START:
            # Start master first, slaves started on confirm
            self.__socket.sendto(
                str.encode(cmd),
                (self.__master_node, SOCKET_PORT)
            )
            self.__is_running = True
        elif cmd == MSG_CMD_STOP:
            self.__socket.sendto(
                str.encode(cmd),
                (self.__broadcast_ip, SOCKET_PORT)
            )
            self.__is_running = False
        elif cmd == MSG_CMD_DEBUG:
            if not self.__is_running:
                print("ROS is not running, please run 'start' first")
                return

            if data.lower() not in ["on", "off"]:
                print("Invalid input, expected 'on' or 'off'")
                return

            self.__socket.sendto(
                str.encode(f"{cmd}|{data}"),
                (self.__broadcast_ip, SOCKET_PORT)
            )
        elif cmd == MSG_CMD_CLEAR_SCREEN:
            print("\033c")
            return
        elif cmd == MSG_CMD_LIST_NODES:
            self.print_nodes()
            return
