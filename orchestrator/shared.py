SOCKET_PORT = 6666
BUFFER_SIZE = 1024

REPO_PATH = "/home/laff/laff-platooning"
PID_PATH = "/tmp/laff.pid"

HEARTBEAT_TIMER_EXPIRATION = 5
STARTUP_HEARTBEAT_TIMER_EXPIRATION = 1

# Node message
MSG_CMD_HEARTBEAT = "hb"
MSG_CMD_REQUEST_MASTER = "rm"

# Server messages
MSG_CMD_SET_MASTER = "master"
MSG_CMD_START = "start"
MSG_CMD_STOP = "stop"
MSG_CMD_UPDATE = "update"
MSG_CMD_LIST_NODES = "ls"

AVAILABLE_COMMANDS = [
    MSG_CMD_START,
    MSG_CMD_STOP,
    MSG_CMD_LIST_NODES,
    MSG_CMD_UPDATE,
    MSG_CMD_SET_MASTER,
]

AVAILABLE_COMMANDS_STR = f"""
Available commands:
- {MSG_CMD_START}: Start ROS on all nodes.
- {MSG_CMD_STOP}: Stop ROS on all nodes.
- {MSG_CMD_LIST_NODES}: List the nodes connected to the server.
- {MSG_CMD_UPDATE} [branch]: Update all nodes to latest code on <branch>
- {MSG_CMD_SET_MASTER} [ip]: Set master node to be controlled with DS4 controller
"""
