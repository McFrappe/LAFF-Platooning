SOCKET_PORT = 6666
BUFFER_SIZE = 1024

REPO_PATH = "/home/laff/laff-platooning"
PID_PATH = "/tmp/laff.pid"
ROS_MASTER_URI_PATH = "/tmp/ROS_MASTER_URI"
VEHICLE_ID_PATH = "/tmp/VEHICLE_ID"

HEARTBEAT_TIMER_EXPIRATION = 5
STARTUP_HEARTBEAT_TIMER_EXPIRATION = 1
MINIMUM_RUNNING_TIME = 10
MASTER_STARTUP_WAIT_TIME = 5

OK = 0
ERROR = 1

# Node message
MSG_CMD_ERROR = "error"
MSG_CMD_HEARTBEAT = "heartbeat"
MSG_CMD_REQUEST_MASTER = "request_master"
MSG_CMD_START_CONFIRM = "start_confirm"
MSG_CMD_STOP_CONFIRM = "stop_confirm"
MSG_CMD_UPDATE_START_CONFIRM = "update_start_confirm"
MSG_CMD_UPDATE_CONFIRM = "update_confirm"
MSG_CMD_MASTER_CONFIRM = "master_confirm"
MSG_CMD_NOT_MASTER_CONFIRM = "not_master_confirm"
MSG_CMD_ORDER_CONFIRM = "order_confirm"

# Server messages
MSG_CMD_SET_MASTER = "master"
MSG_CMD_ORDER = "order"
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
    MSG_CMD_ORDER,
]

AVAILABLE_COMMANDS_STR = f"""
Available commands:
- {MSG_CMD_START}: Start ROS on all nodes.
- {MSG_CMD_STOP}: Stop ROS on all nodes.
- {MSG_CMD_LIST_NODES}: List the nodes connected to the server.
- {MSG_CMD_UPDATE} [branch]: Update all nodes to latest code on <branch>
- {MSG_CMD_SET_MASTER} [ip]: Set master node to be controlled with DS4 controller
- {MSG_CMD_ORDER}: Assign platooning order based on time of conection (master is always first)
"""
