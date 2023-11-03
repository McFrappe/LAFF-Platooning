import socket
import select
import linuxfd
from orchestrator.shared import *
from orchestrator.utils import get_ip
from orchestrator.client.node import Node

def run():
    """
    Main entry point for a client node. Creates a UDP socket and listens for
    incoming messages from the master node. Also sends heartbeat messages  with
    a specified interval to the server to register itself.
    """
    local_ip = get_ip()
    s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((local_ip, SOCKET_PORT))

    print(f"Node: Listening on {local_ip}:{SOCKET_PORT}")
    node = Node(s, local_ip)
    print("Node: Created node object")

    timer = linuxfd.timerfd(rtc=True)
    timer_fd = timer.fileno()
    timer.settime(STARTUP_HEARTBEAT_TIMER_EXPIRATION)

    while True:
        fds = [timer_fd, s]
        rs, ws, _ = select.select(fds, [], [])

        for sock in rs:
            if sock == s:
                (msg, _) = s.recvfrom(BUFFER_SIZE)
                parsed_msg = [x.strip("\n").lower() for x in msg.decode("utf-8").split(":")]
                node.handle_message(parsed_msg)
            else:
                node.send_heartbeat()
                print("Heartbeat sent")

        timer.settime(HEARTBEAT_TIMER_EXPIRATION)

if __name__ == "__main__":
    run()
