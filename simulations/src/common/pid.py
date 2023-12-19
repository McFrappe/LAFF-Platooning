
class PID:
    def __init__(self, kp, ki, kd, dt: int, pid_min: float, pid_max: float,  pid_increase_min: float, pid_increase_max: float):
        self.error = 0
        self.error_integral = 0
        self.error_derivative = 0 

        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

        self.dt = dt
        self.pid_min = pid_min
        self.pid_max = pid_max
        self.pid_increase_min = pid_increase_min
        self.pid_increase_max = pid_increase_max

        self.u = 0

    def update(self, error: float):
        error_prev = self.error
        self.error = error
        self.error_integral += error*self.dt
        self.error_derivative = (error - error_prev)/self.dt

        desired_u = self.Kp*error + self.Ki*self.error_integral + self.Kd*self.error_derivative
        desire_increase = desired_u - self.u

        increase = min(max(desire_increase, self.pid_increase_min), self.pid_increase_max)
        self.u = min(max(self.u + increase, self.pid_min), self.pid_max)

        return self.u
