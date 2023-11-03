SOCKET_PORT = 6666
BUFFER_SIZE = 1024
BROADCAST_ADDR = "255.255.255.255"

REPO_PATH = "/home/laff/laff-platooning"
HEARTBEAT_INTERVAL = 5

# Node message
MSG_CMD_HEARTBEAT = "hb"
MSG_CMD_REQUEST_MASTER = "rm"

# Server messages
MSG_CMD_SET_MASTER = "master"
MSG_CMD_START = "start"
MSG_CMD_STOP = "stop"
MSG_CMD_LIST_NODES = "ls"

AVAILABLE_COMMANDS = [
    MSG_CMD_SET_MASTER,
    MSG_CMD_START,
    MSG_CMD_STOP,
    MSG_CMD_LIST_NODES
]

AVAILABLE_COMMANDS_STR = f"Available commands: \n- {MSG_CMD_SET_MASTER}: Set a node to master node.\n- {MSG_CMD_START}: Start the nodes.\n- {MSG_CMD_STOP}: Stop the nodes.\n- {MSG_CMD_LIST_NODES}: List the nodes connected to the server.\n"
