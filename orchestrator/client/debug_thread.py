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

    def run(self):
        self.__proc = subprocess.Popen(
            f"make debug_listener", shell=True, stdout=subprocess.PIPE,
            cwd=REPO_PATH, executable="/bin/bash")
        while not self.__stop_event.is_set():
            msg = self.__proc.stdout.readline()
            if not msg:
                break
            self.__broadcast_cb(msg)

    def stop(self):
        if self.__proc is not None:
            self.__proc.terminate()
        self.__stop_event.set()

    def stopped(self):
        return self.__stop_event.is_set()
