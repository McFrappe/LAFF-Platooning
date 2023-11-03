import os
import time
import socket
import select
import linuxfd
from orchestrator.shared import *
from orchestrator.client.node import Node

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        s.connect(('10.254.254.254', 1))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
        return ip

def run():
    local_ip = get_ip()
    s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
                if len(parsed_msg) != 2:
                    print(f"Received broken message {msg} from {addr}")
                    continue
                node.handle_message(parsed_msg)
            else:
                node.send_heartbeat()

if __name__ == "__main__":
    run()
