import os
import time
import subprocess
from threading import Timer
from orchestrator.shared import *
from orchestrator.utils import get_broadcast_ip
from orchestrator.client.debug_thread import DebugThread


class Node:
    """
    Node class that handles the communication between the server and the
    other nodes. The node can be either a master node or a slave node.
    """

    def __init__(self, socket, ip):
        self.__id = None
        self.__ip = ip
        self.__socket = socket
        self.__is_master = False
        self.__running = False
        self.__broadcast_ip = get_broadcast_ip()
        self.__start_time = -1
        self.__start_confirm_timer = Timer(
            MASTER_STARTUP_WAIT_TIME, self.__confirm_start)
        self.__debug_thread = None
        self.__hostname = self.__get_hostname()

    def __get_hostname(self):
        try:
            proc = subprocess.run(["hostname"], capture_output=True, text=True)
            return proc.stdout[:-1]
        except Exception:
            return None

    def __confirm_start(self):
        self.__socket.sendto(
            str.encode(MSG_CMD_START_CONFIRM),
            (self.__broadcast_ip, SOCKET_PORT)
        )
        self.__running = True

    def __broadcast_error(self, error):
        self.__socket.sendto(
            str.encode(f"{MSG_CMD_ERROR}|{error}"),
            (self.__broadcast_ip, SOCKET_PORT)
        )

    def __broadcast_debug_msg(self, msg):
        self.__socket.sendto(
            str.encode(f"{MSG_CMD_DEBUG_MSG}|{msg}"),
            (self.__broadcast_ip, SOCKET_PORT)
        )

    def start(self):
        """
        Starts the node process and sets the master flag if the node is the
        master node. If the node is already running, this method does nothing.
        """
        if self.__running or self.__start_confirm_timer.is_alive():
            return OK

        if self.__id is None:
            self.__broadcast_error("Please assign vehicle id before starting")
            return ERROR

        if self.__is_master:
            make_cmd = "run_rcv_joystick"
        else:
            make_cmd = "run_rcv_pid"

        try:
            subprocess.Popen(
                f"make {make_cmd}", shell=True, cwd=REPO_PATH, executable="/bin/bash")
            if self.__is_master:
                # If we are the master, we need to wait for it to start to
                # ensure that the ROS master is available for the slaves.
                # Only send confirm (and start slaves) after a few seconds.
                self.__start_confirm_timer.start()
            else:
                self.__confirm_start()

            self.__start_time = time.time()
            return OK
        except Exception as e:
            print(f"Failed to start process:\n{e}")
            self.__broadcast_error(f"Failed to start ROS:\n{e}")
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
            self.__broadcast_error("Need to wait for ROS to fully start before stopping")
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
            self.__broadcast_error(f"Failed to stop ROS:\n{e}")

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
            self.__broadcast_error(f"Failed to update on branch {branch}:\n{e}")
            return ERROR

        if was_running:
            return self.start()

    def set_master(self, new_master):
        """
        Sets the master node. If the node is already the master node, this method does nothing.
        """
        self.__is_master = new_master == self.__ip
        print(f"Set master: {self.__is_master}")

        try:
            # store master ip in file and set it to be the environent variable ROS_MASTER_URI
            with open(ROS_MASTER_URI_PATH, "w") as f:
                f.write(f"http://{new_master}:11311")
        except Exception as e:
            self.__broadcast_error(f"Failed to set master:\n{e}")
            return ERROR

        cmd = MSG_CMD_MASTER_CONFIRM if self.__is_master else MSG_CMD_NOT_MASTER_CONFIRM
        self.__socket.sendto(
            str.encode(cmd),
            (self.__broadcast_ip, SOCKET_PORT)
        )
        return OK

    def set_id(self, new_id):
        """
        Sets the id of the node.
        """
        try:
            with open(VEHICLE_ID_PATH, "w") as f:
                f.write(f"vehicle_{new_id}")
        except Exception as e:
            self.__broadcast_error(f"Failed to set vehicle id:\n{e}")
            return ERROR

        self.__id = new_id
        self.__socket.sendto(
            str.encode(f"{MSG_CMD_ORDER_CONFIRM}|{new_id}"),
            (self.__broadcast_ip, SOCKET_PORT)
        )
        return OK

    def set_debug(self, new_state):
        """
        Enables/disables debug mode.
        """
        try:
            if new_state == "on":
                if self.__debug_thread is not None:
                    return
                self.__debug_thread = DebugThread(self.__broadcast_debug_msg)
                self.__debug_thread.start()
            else:
                if self.__debug_thread is None:
                    return
                self.__debug_thread.stop()
                self.__debug_thread = None
        except Exception as e:
            self.__broadcast_error(f"Failed to turn debug mode {new_state}:\n{e}")
            return ERROR

        self.__socket.sendto(
            str.encode(f"{MSG_CMD_DEBUG_CONFIRM}|{new_state}"),
            (self.__broadcast_ip, SOCKET_PORT)
        )
        return OK

    def set_lights(self, data):
        """
        Toggles the LED lights on the Pixy2 camera on/off based on
        the value of data.
        """
        try:
            state = "true" if data == "on" else "false"
            proc = subprocess.Popen(
                f"make PUBLISH_CMD_ARGS='/lights std_msgs/Bool {state}' publish",
                shell=True, cwd=REPO_PATH, executable="/bin/bash")
            proc.wait()
            self.__socket.sendto(
                str.encode(f"{MSG_CMD_LIGHTS_CONFIRM}|{data}"),
                (self.__broadcast_ip, SOCKET_PORT)
            )
            return OK
        except Exception as e:
            self.__broadcast_error(e)
            return ERROR

    def send_heartbeat(self):
        """
        Sends a heartbeat message to the master node.
        """
        self.__socket.sendto(
            str.encode(f"{MSG_CMD_HEARTBEAT}|{self.__hostname}"),
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
        elif cmd == MSG_CMD_ORDER:
            print(f"Received order command with assigned id {data}")
            self.set_id(data)
        elif cmd == MSG_CMD_DEBUG:
            print(f"Received debug command with new state {data}")
            self.set_debug(data)
        elif cmd == MSG_CMD_LIGHTS:
            print(f"Received lights command with data {data}")
            self.set_lights(data)
