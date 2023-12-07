import numpy as np
import matplotlib.pyplot as plt

class PID:
    def __init__(self, kp, ki, kd, dt: int, pid_min: float, pid_max: float):
        self.error = 0
        self.error_integral = 0
        self.error_derivative = 0 
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.dt = dt
        self.pid_min = pid_min
        self.pid_max = pid_max

    def update(self, error: float):
        error_prev = self.error
        self.error = error
        self.error_integral += error*self.dt
        self.error_derivative = (error - error_prev)/self.dt # TODO: should be devided with period/dt

        u = self.Kp*error + self.Ki*self.error_integral + self.Kd*self.error_derivative
        return min(max(u, self.pid_min), self.pid_max)


v_t = np.array([0])
v_r = np.array([0])

idle_pwm = 57
max_pwm = 100
pwm = idle_pwm 
num_ticks = 1200
tick_in_s = 0.01

max_acceleration = 0.08
acceleration = 0.08
deceleration = 0.3

k = 30/43 # one step in PWM represent k km/increase

u = 0
e = 0
e_sum = 0
e_prev = 0
e_der = 0

k_p = 1
k_i = 20
k_d = 0.002

pid = PID(k_p, k_i, k_d, dt=tick_in_s, pid_min=-1, pid_max=1)

pid_pwm_min = -1
pid_pwm_max = 1

for tick in range(num_ticks):

    # v_t
    if tick < 100:
        v_t = np.append(v_t, 0)
    elif tick < 800:
        v_t = np.append(v_t, v_t[tick]+acceleration)
    elif tick > 1000 and v_t[tick] != 0:
        v_t = np.append(v_t, v_t[tick]-deceleration)
    else:
        v_t = np.append(v_t, v_t[tick])

    # v_r
    v_r = np.append(v_r, k*(pwm-idle_pwm))

    # PID
    u = pid.update(v_t[tick] - v_r[tick])

    pwm = pwm + u
    print(u)

t = np.arange(len(v_t))
plt.plot(t, v_t, linestyle="-")
plt.plot(t, v_r, linestyle="-")
plt.xlabel("time [10ms]")
plt.ylabel("velocity [km/h]")
plt.show()