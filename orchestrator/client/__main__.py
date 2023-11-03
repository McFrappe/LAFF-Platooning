import os
import time
import socket
import select
import linuxfd
from orchestrator.shared import *
from orchestrator.utils import get_ip
from orchestrator.client.node import Node

def run():
    local_ip = get_ip()
    s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((local_ip, SOCKET_PORT))

    print(f"Listening on {local_ip}:{SOCKET_PORT}")
    node = Node(s, local_ip)

    timer = linuxfd.timerfd(time.CLOCK_REALTIME)
    timer.settime(0, interval=HEARTBEAT_INTERVAL)
    timer_fd = timer.fileno()

    while True:
        fds = [timer_fd, s]
        rs, ws, _ = select.select(fds, [], [])

        for sock in rs:
            if sock == s:
                (msg, addr) = s.recvfrom(BUFFER_SIZE)
                parsed_msg = [x.strip("\n").lower() for x in msg.decode("utf-8").split(":")]
                node.handle_message(parsed_msg)
            else:
                node.send_heartbeat()

if __name__ == "__main__":
    run()
