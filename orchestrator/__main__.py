import sys
import select
import socket
import curses
import threading

from orchestrator.gui import GUI
from orchestrator.utils import *
from orchestrator.shared import *
from orchestrator.server import Server

running = True

def run_server(sock, server):
    while running:
        (msg, addr) = sock.recvfrom(BUFFER_SIZE)
        if addr[0].endswith("255"):
            continue

        parsed_msg = msg.decode("utf-8").split("|")
        server.handle_message(parsed_msg, addr)

def run(std_scr):
    """
    Main entry point for the orchestrator. Creates a UDP socket and listens for
    incoming messages
    """
    s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind(("0.0.0.0", SOCKET_PORT))

    gui = GUI(std_scr)
    server = Server(s, gui)
    thread = threading.Thread(target=run_server, args=(s, server))
    thread.daemon = True
    thread.start()

    gui.welcome()

    while True:
        server.update_nodes()
        server.update_status()

        msg = gui.prompt()
        parsed_msg = [x.strip("\n").lower() for x in msg.split(" ")]

        if parsed_msg[0] == "q" or parsed_msg[0] == "quit":
            server.stop_node_timers()
            break
        elif parsed_msg[0] == "h" or parsed_msg[0] == "help":
            gui.help()
        elif parsed_msg[0] == MSG_CMD_CLEAR_SCREEN:
            gui.clear_output()
        elif len(parsed_msg[0]) > 0:
            server.handle_user_cmd(parsed_msg)

    server.stop_node_timers()
    running = False

if __name__ == "__main__":
    curses.wrapper(run)
