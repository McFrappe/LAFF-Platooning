from orchestrator.shared import *
from orchestrator.utils import get_broadcast_ip

class Server:
    def __init__(self, socket):
        self.__broadcast_ip = get_broadcast_ip()
        self.__master_node = None
        self.__is_running = False
        self.__socket = socket
        self.__nodes = []

    def print_nodes(self):
        if len(self.__nodes) == 0:
            print("No connected nodes")
            return

        for node in self.__nodes:
            print(node)

    def handle_message(self, msg, addr):
        cmd = msg[0]
        if cmd == MSG_CMD_HEARTBEAT:
            if data not in self.__nodes:
                print("New node {data} connected")
                self.__nodes.append(data)

    def handle_user_cmd(self, msg):
        cmd = msg[0]
        data = "" if len(msg) == 1 else msg[1]

        if cmd == MSG_CMD_SET_MASTER:
            if len(data) == 0 or data not in self.__nodes:
                print("Invalid node address for master")
                return
            self.__socket.sendto(str.encode(cmd), (data, SOCKET_PORT))
        elif cmd == MSG_CMD_START:
            self.__socket.sendto(str.encode(cmd), (self.__broadcast_ip, SOCKET_PORT))
            self.__is_running = True
        elif cmd == MSG_CMD_STOP:
            self.__socket.sendto(str.encode(cmd), (self.__broadcast_ip, SOCKET_PORT))
            self.__is_running = False
        else:
            print("Unsupported command")
