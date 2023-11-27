import os
import time
import threading
import subprocess
from orchestrator.shared import *

class DebugThread(threading.Thread):
    def __init__(self, broadcast_cb):
        threading.Thread.__init__(self)
        self.__broadcast_cb = broadcast_cb
        self.__stop_event = threading.Event()
        self.__proc = None
        self.__running = False

        # The number of stdout lines to skip.
        # This will stop the node from broadcasting the
        # commands in the Makefile.
        self.__skip_count = 3

    def run(self):
        self.__proc = subprocess.Popen(
            f"make debug_listener", shell=True, stdout=subprocess.PIPE,
            cwd=REPO_PATH, executable="/bin/bash")

        with open(DEBUG_PID_PATH, "w") as f:
            f.write(str(self.__proc.pid))

        count = 0
        while not self.__stop_event.is_set():
            msg = self.__proc.stdout.readline()

            if count < self.__skip_count:
                count += 1
                continue

            parsed_msg = msg.decode("utf-8")
            if "," not in parsed_msg:
                continue

            # Only broadcast the data, not the timestamp
            self.__broadcast_cb(parsed_msg[parsed_msg.index(",")+1:].strip("\n"))
            time.sleep(1)

    def stop(self):
        if self.__proc is not None:
            self.__proc.terminate()
        self.__stop_event.set()
        os.remove(DEBUG_PID_PATH)

    def stopped(self):
        return self.__stop_event.is_set()
