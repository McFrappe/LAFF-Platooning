import os
import select
import linuxfd
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

        msg = ""
        count = 0
        timer = linuxfd.timerfd(rtc=True)
        timer_fd = timer.fileno()
        timer.settime(STARTUP_HEARTBEAT_TIMER_EXPIRATION)

        stdout_fd = self.__proc.stdout.fileno()
        flags_stdout = fcntl.fcntl(stdout_fd, fcntl.F_GETFL)
        fcntl.fcntl(stdout_fd, fcntl.F_SETFL, flags_stdout | os.O_NONBLOCK)

        while not self.__stop_event.is_set():
            rs, _, _ = select.select([timer_fd, stdout_fd], [], [])

            for fd in rs:
                if fd == stdout_fd:
                    count += 1
                    msg = self.__proc.stdout.readline()
                else:
                    if count < self.__skip_count:
                        continue

                    parsed_msg = msg.decode("utf-8")
                    if "," not in parsed_msg:
                        continue

                    # Only broadcast the data, not the timestamp
                    self.__broadcast_cb(
                        parsed_msg[parsed_msg.index(",")+1:].strip("\n"))
                    timer.settime(1)

    def stop(self):
        if self.__proc is not None:
            self.__proc.terminate()
        self.__stop_event.set()

    def stopped(self):
        return self.__stop_event.is_set()
