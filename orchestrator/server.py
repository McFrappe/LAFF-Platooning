from threading import Timer
from orchestrator.shared import *
from orchestrator.utils import get_broadcast_ip


class Server:
    """
    Server class that handles the communication between the master node and the
    other nodes.
    """

    def __init__(self, socket, gui):
        self.__gui = gui
        self.__broadcast_ip = get_broadcast_ip()
        self.__master_node = None
        self.__is_running = False
        self.__is_debug = False
        self.__nodes = dict()
        self.__hostnames = dict()
        self.__socket = socket
        self.__ordered_nodes = set()

    def __validate_node_input(self, node):
        if len(node) == 0 or len(self.__nodes) == 0:
            self.__gui.output("Invalid address")
            return None

        ips = list(self.__nodes.keys())
        node = ips[0]
        if "." in node:
            return node
        else:
            try:
                val = int(node)
                if val >= len(self.__nodes):
                    self.__gui.output(
                        "Node index out of range")
                    return None
                return ips[val]
            except:
                self.__gui.output("Invalid node index")
                return None

    def __remove_node(self, ip):
        """
        remove a node from the list of connected nodes
        """
        if ip not in self.__nodes.keys():
            return

        del self.__nodes[ip]
        self.__gui.output(
            f"Node {ip} removed from list of nodes due to timeout.")
        self.update_nodes()

    def update_nodes(self):
        """
        Updates the list of connected nodes
        """
        hosts = []
        for ip in self.__nodes.keys():
            if ip in self.__hostnames.keys():
                hosts.append((ip, self.__hostnames[ip]))
            else:
                hosts.append((ip, "Unknown"))

        self.__gui.update_nodes(hosts, self.__master_node)

    def update_status(self):
        """
        Updates the status of the server, i.e., if any data has been updated
        """
        self.__gui.update_status(self.__is_running, self.__is_debug)

    def stop_node_timers(self):
        """
        Stops all node timers
        """
        for ip in self.__nodes.keys():
            if self.__nodes[ip] is None:
                continue
            self.__nodes[ip].cancel()
            self.__nodes[ip] = None

    def start_node_timer(self, ip):
        """
        Starts a timer for a registered node.
        If the timer expires, the node is removed from the list of connected nodes.
        """
        if ip in self.__nodes:
            if self.__nodes[ip] is not None:
                self.__nodes[ip].cancel()
        self.__nodes[ip] = Timer(
            HEARTBEAT_TIMEOUT, self.__remove_node, args=[ip])
        self.__nodes[ip].start()

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
                self.start_node_timer(ip)
                return

            self.start_node_timer(ip)
            self.update_nodes()
        elif cmd == MSG_CMD_HOSTNAME:
            if ip in self.__hostnames.keys():
                return

            self.__hostnames[ip] = data
            self.update_nodes()
        elif cmd == MSG_CMD_START_CONFIRM:
            if self.__master_node == ip:
                self.__gui.socket_output(f"{ip} started, starting slaves")
                self.__socket.sendto(
                    str.encode(MSG_CMD_START),
                    (self.__broadcast_ip, SOCKET_PORT)
                )
            else:
                self.__gui.socket_output(f"{ip} started")
        elif cmd == MSG_CMD_STOP_CONFIRM:
            self.__gui.socket_output(f"{ip} stopped")
        elif cmd == MSG_CMD_UPDATE_START_CONFIRM:
            self.__gui.socket_output(f"{ip} started update")
        elif cmd == MSG_CMD_UPDATE_CONFIRM:
            self.__gui.socket_output(f"{ip} updated")
            self.start_node_timer(ip)
        elif cmd == MSG_CMD_MASTER_CONFIRM:
            self.__gui.socket_output(f"{ip} set to master")
            self.__master_node = ip
            self.update_nodes()
        elif cmd == MSG_CMD_NOT_MASTER_CONFIRM:
            self.__gui.socket_output(f"{ip} set to slave")
        elif cmd == MSG_CMD_ORDER_CONFIRM:
            self.__gui.socket_output(f"{ip} assigned id {data}")
            self.__ordered_nodes.add(ip)
        elif cmd == MSG_CMD_DEBUG_CONFIRM:
            self.__gui.socket_output(f"{ip} set debug mode {data}")
        elif cmd == MSG_CMD_DEBUG_MSG:
            if self.__is_debug:
                self.__gui.output(data)
        elif cmd == MSG_CMD_LIGHTS_CONFIRM:
            self.__gui.output(f"{ip} turned lights {data}")
            self.start_node_timer(ip)
        elif cmd == MSG_CMD_ERROR:
            self.__gui.socket_output(f"{ip} got error")
            self.__gui.output(f"{ip} {data}")

        self.update_status()

    def handle_user_cmd(self, msg):
        """
        Handles user input commands and sends them to the nodes via UDP
        broadcast.
        """
        cmd = msg[0]
        data = "" if len(msg) == 1 else msg[1]

        if cmd not in AVAILABLE_COMMANDS:
            self.__gui.output(f"Unsupported command: {cmd}")
            return

        if cmd == MSG_CMD_SET_MASTER:
            node = self.__validate_node_input(data)
            if node is None:
                return ERROR
            self.__socket.sendto(
                str.encode(f"{cmd}|{node}"),
                (self.__broadcast_ip, SOCKET_PORT)
            )
        elif cmd == MSG_CMD_UPDATE:
            if len(data) == 0 or " " in data:
                self.__gui.output("Invalid branch name")
                return

            self.stop_node_timers()
            self.__socket.sendto(
                str.encode(f"{cmd}|{data}"),
                (self.__broadcast_ip, SOCKET_PORT)
            )
        elif cmd == MSG_CMD_ORDER:
            ips = [self.__validate_node_input(x.strip()) for x in data.split(",")]
            if None in ips:
                return ERROR

            for idx, ip in enumerate(ips):
                self.__socket.sendto(
                    str.encode(f"{MSG_CMD_ORDER}|{idx+1}"),
                    (ip, SOCKET_PORT)
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
            if data.lower() not in ["on", "off"]:
                self.__gui.output("Invalid input, expected 'on' or 'off'")
                return

            self.__is_debug = data.lower() == "on"
            self.__socket.sendto(
                str.encode(f"{cmd}|{data}"),
                (self.__broadcast_ip, SOCKET_PORT)
            )
        elif cmd == MSG_CMD_LIGHTS:
            if data.lower() not in ["on", "off"]:
                self.__gui.output("Invalid input, expected 'on' or 'off'")
                return

            self.stop_node_timers()
            self.__socket.sendto(
                str.encode(f"{MSG_CMD_LIGHTS}|{data.lower()}"),
                (self.__broadcast_ip, SOCKET_PORT)
            )
