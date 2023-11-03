import socket

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

def get_broadcast_ip():
    local_ip = get_ip()
    return f"{str.join('.', local_ip.split('.')[0:-1])}.255"
