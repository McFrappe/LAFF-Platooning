import numpy as np
import matplotlib.pyplot as plt

v_t = np.array([0])
v_r = np.array([0])

idle_pwm = 57
pwm = idle_pwm 
num_ticks = 1200
tick_in_s = 0.01

max_acceleration = 0.08
acceleration = 0.08/2

k = 30/43 # one step in PWM represent k km/increase

u = 0
e = 0
e_sum = 0
e_prev = 0
e_der = 0

k_p = 1
k_i = 0.3
k_d = 0.2

for tick in range(num_ticks):

    # v_t
    if tick < 100:
        v_t = np.append(v_t, 0)
    elif tick < 1000:
        v_t = np.append(v_t, v_t[tick]+acceleration)
    else:
        v_t = np.append(v_t, v_t[tick])

    # v_r
    v_r = np.append(v_r, k*(pwm-idle_pwm))

    # PID
    e_prev = e
    e = v_t[tick] - v_r[tick]
    e_sum = e_sum + e*tick_in_s
    e_der = e - e_prev

    u = k_p*e + k_i*e_sum + k_d*e_der
    pwm = pwm + u
    print(e_sum)

t = np.arange(len(v_t))
plt.plot(t, v_t, linestyle="-")
plt.plot(t, v_r, linestyle="-")
plt.xlabel("time [10ms]")
plt.ylabel("velocity [km/h]")
plt.show()