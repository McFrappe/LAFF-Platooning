from orchestrator.shared import *
from orchestrator.utils import get_broadcast_ip

class Server:
    """
    Server class that handles the communication between the master node and the
    other nodes.
    """

    def __init__(self, socket):
        self.__broadcast_ip = get_broadcast_ip()
        self.__master_node = None
        self.__is_running = False
        self.__socket = socket
        self.__nodes = []

    def print_nodes(self):
        """
        Prints all connected nodes
        """
        if len(self.__nodes) == 0:
            print("No connected nodes")
            return

        for node in self.__nodes:
            print(node)

    def handle_message(self, msg, addr):
        """
        Handles incoming messages from the nodes. The messages 'msg' are in the
        format of [command, data]
        """
        cmd = msg[0]
        if cmd == MSG_CMD_HEARTBEAT:
            if addr not in self.__nodes:
                self.__nodes.append(addr)
                print(f"Server: Registered node {addr} to list of nodes")

    def handle_user_cmd(self, msg):
        """
        Handles user input commands and sends them to the nodes via UDP
        broadcast.
        """
        cmd = msg[0]
        data = "" if len(msg) == 1 else msg[1]

        if cmd not in AVAILABLE_COMMANDS:
                print(f"Server: Unsupported command: {cmd}")
                return
        elif cmd == MSG_CMD_SET_MASTER:
            if len(data) == 0 or data not in self.__nodes:
                print("Server: Invalid node address for master, see the registered nodes with command 'ls'")
                return

            # TODO: send broadcast to all nodes to set master instead
            self.__socket.sendto(str.encode(cmd), (data, SOCKET_PORT))
        elif cmd == MSG_CMD_START:
            self.__socket.sendto(str.encode(cmd), (self.__broadcast_ip,
                                                   SOCKET_PORT))
            self.__is_running = True
            self.__socket.sendto(str.encode(cmd), (BROADCAST_ADDR,
                                                   SOCKET_PORT))
        elif cmd == MSG_CMD_STOP:
            self.__socket.sendto(str.encode(cmd), (self.__broadcast_ip,
                                                   SOCKET_PORT))
            self.__is_running = False
        elif cmd == MSG_CMD_LIST_NODES:
            self.print_nodes()
            return

