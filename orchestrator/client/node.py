import os
import signal
import socket
import subprocess
from orchestrator.shared import *
from orchestrator.utils import get_broadcast_ip

class Node:
    def __init__(self, socket, ip):
        self.__ip = ip
        self.__socket = socket
        self.__is_master = False
        self.__pid = -1
        self.__broadcast_ip = get_broadcast_ip()

    def start(self):
        print("Received start command")

        if self.__is_master:
            make_cmd = "run_rcv_joystick_pi"
        else:
            make_cmd = "run_rcv_pi"

        proc = subprocess.Popen(f"cd {REPO_PATH} && make {make_cmd}", shell=True)
        self.__pid = proc.pid

    def stop(self):
        print("Received stop command")
        if self.__pid == -1:
            return

        os.killpg(self.__pid, signal.SIGTERM)
        self.__pid = -1

    def send_heartbeat(self):
        print("Sending heartbeat")
        self.__socket.sendto(str.encode(MSG_CMD_HEARTBEAT), (self.__broadcast_ip, SOCKET_PORT))

    def handle_message(self, msg):
        cmd = msg[0]
        data = "" if len(msg) == 1 else msg[1]

        if msg == MSG_CMD_SET_MASTER:
            self.__is_master = data == self.__ip
        elif msg == MSG_CMD_START:
            self.start()
        elif msg == MSG_CMD_STOP:
            self.stop()
