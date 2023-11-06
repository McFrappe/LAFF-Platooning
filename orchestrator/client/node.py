import os
import signal
import subprocess
from orchestrator.shared import *
from orchestrator.utils import get_broadcast_ip


class Node:
    """
    Node class that handles the communication between the server and the
    other nodes. The node can be either a master node or a slave node.
    """

    def __init__(self, socket, ip):
        self.__ip = ip
        self.__socket = socket
        self.__is_master = False
        self.__running = False
        self.__broadcast_ip = get_broadcast_ip()

    def start(self):
        """
        Starts the node process and sets the master flag if the node is the
        master node. If the node is already running, this method does nothing.
        """
        if self.__running:
            return

        # TODO: Add when rcdriver is merged
        # if self.__is_master:
        #     make_cmd = "run_rcv_joystick_pi"
        # else:
        #     make_cmd = "run_rcv_pi"
        make_cmd = "run"

        try:
            subprocess.Popen(
                f"make {make_cmd}", shell=True, cwd=REPO_PATH, executable="/bin/bash")
            self.__running = True
        except Exception as e:
            print(f"Failed to start process:\n{e}")

    def stop(self):
        """
        Stops the node process. If the node is not running, this method does
        nothing.
        """
        if not self.__running:
            return

        try:
            subprocess.Popen(
                f"sudo kill -SIGINT $(cat {PID_PATH})", shell=True, executable="/bin/bash")
        except Exception as e:
            print(f"Failed to stop process:\n{e}")

        # Assume the process is already dead
        self.__running = False

    def update(self, branch):
        self.stop()
        subprocess.Popen(
            f"make BRANCH={branch} update", shell=True, cwd=REPO_PATH, executable="/bin/bash")
        self.start()

    def set_master(self, new_master):
        self.__is_master = new_master == self.__ip
        print(f"Set master: {self.__is_master}")

    def send_heartbeat(self):
        """
        Sends a heartbeat message to the master node.
        """
        self.__socket.sendto(
            str.encode(MSG_CMD_HEARTBEAT),
            (self.__broadcast_ip, SOCKET_PORT)
        )

    def handle_message(self, msg):
        """
        Handles incoming messages from the master node. The messages 'msg' are
        in the format of [command, data]
        """
        cmd = msg[0]
        data = "" if len(msg) == 1 else msg[1]
        if cmd == MSG_CMD_SET_MASTER:
            print("Received set master command")
            self.set_master(data)
        elif cmd == MSG_CMD_START:
            print("Received start command")
            self.start()
        elif cmd == MSG_CMD_STOP:
            print("Received stop command")
            self.stop()
        elif cmd == MSG_CMD_UPDATE:
            print(f"Received update command for branch {data}")
            self.update(data)
