from simple_pid import PID as SimplePID

class PID:
    def __init__(self, kp, ki, kd, setpoint: int):
        self.pid = SimplePID(Kp=kp, Ki=ki, Kd=kd, setpoint=setpoint)

    def update(self, distance: float):
        return self.pid(distance)