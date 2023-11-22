import os
import time
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
        self.__start_time = -1

    def start(self):
        """
        Starts the node process and sets the master flag if the node is the
        master node. If the node is already running, this method does nothing.
        """
        if self.__running:
            return OK

        if self.__is_master:
            make_cmd = "run_rcv_joystick_pi"
        else:
            make_cmd = "run_rcv_pi"

        try:
            subprocess.Popen(
                f"make {make_cmd}", shell=True, cwd=REPO_PATH, executable="/bin/bash")
            self.__socket.sendto(
                str.encode(MSG_CMD_START_CONFIRM),
                (self.__broadcast_ip, SOCKET_PORT)
            )
            self.__running = True
            self.__start_time = time.time()
            return OK
        except Exception as e:
            print(f"Failed to start process:\n{e}")
            return ERROR

    def stop(self):
        """
        Stops the node process. If the node is not running, this method does
        nothing.
        """
        if not self.__running:
            return OK

        # Killing roslaunch before nodes have been fully spawned results
        # in nodes being kept alive. Ensure that all nodes have spawned
        # before stopping.
        time_since_start = time.time() - self.__start_time
        if self.__start_time != -1 and time_since_start < MINIMUM_RUNNING_TIME:
            return ERROR

        try:
            proc = subprocess.Popen(
                f"sudo kill -SIGINT $(cat {PID_PATH})", shell=True, executable="/bin/bash")
            proc.wait()
            self.__socket.sendto(
                str.encode(MSG_CMD_STOP_CONFIRM),
                (self.__broadcast_ip, SOCKET_PORT)
            )
        except Exception as e:
            print(f"Failed to stop process:\n{e}")

        # Assume the process is already dead
        self.__running = False
        self.__start_time = -1
        return OK

    def update(self, branch):
        was_running = self.__running
        if self.stop() != OK:
            return ERROR

        try:
            self.__socket.sendto(
                str.encode(MSG_CMD_UPDATE_START_CONFIRM),
                (self.__broadcast_ip, SOCKET_PORT)
            )
            proc = subprocess.Popen(
                f"make BRANCH={branch} update", shell=True,
                cwd=REPO_PATH, executable="/bin/bash")
            proc.wait()
            self.__socket.sendto(
                str.encode(MSG_CMD_UPDATE_CONFIRM),
                (self.__broadcast_ip, SOCKET_PORT)
            )
        except Exception as e:
            print(f"Failed to update to branch {branch}:\n{e}")
            return ERROR

        if was_running:
            return self.start()

    def set_master(self, new_master):
        self.__is_master = new_master == self.__ip
        print(f"Set master: {self.__is_master}")
        if self.__is_master:
            self.__socket.sendto(
                str.encode(MSG_CMD_MASTER_CONFIRM),
                (self.__broadcast_ip, SOCKET_PORT)
            )

        # store master ip in file and set it to be the environent variable ROS_MASTER_URI
        with open(ROS_MASTER_URI_PATH, "w") as f:
            f.write(new_master)

        return OK

    def send_heartbeat(self):
        """
        Sends a heartbeat message to the master node.
        """
        self.__socket.sendto(
            str.encode(MSG_CMD_HEARTBEAT),
            (self.__broadcast_ip, SOCKET_PORT)
        )
        return OK

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
