import os
import sys
import select
import socket
from orchestrator.shared import *
from orchestrator.server import Server

def prompt():
    sys.stdout.write("> ")
    sys.stdout.flush()

def run():
    s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind(("0.0.0.0", SOCKET_PORT))

    print(f"Listening on port {SOCKET_PORT}")
    server = Server(s)

    prompt()
    while True:
        fds = [sys.stdin, s]
        rs, _, _ = select.select(fds, [], [])

        for sock in rs:
            if sock == s:
                (msg, addr) = sock.recvfrom(BUFFER_SIZE)
                if addr[0].endswith("255"):
                    continue
                parsed_msg = msg.decode("utf-8").split(":")
                server.handle_message(parsed_msg, addr)
            else:
                msg = sys.stdin.readline()
                parsed_msg = [x.strip("\n").lower() for x in msg.split(" ")]
                if parsed_msg[0] == "q" or parsed_msg[0] == "quit":
                    sys.exit(1)
                elif parsed_msg[0] == "h" or parsed_msg[0] == "help":
                    print(f"Available commands: {MSG_CMD_SET_MASTER}, {MSG_CMD_START}, {MSG_CMD_STOP}, ls")
                    prompt()
                elif parsed_msg[0] == "n" or parsed_msg[0] == "nodes":
                    server.print_nodes()
                    prompt()
                else:
                    server.handle_user_cmd(parsed_msg)
                    prompt()

if __name__ == "__main__":
    run()
