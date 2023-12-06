from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import matplotlib.pyplot as plt
from scipy import signal
import numpy as np
import math


# -------------- State space system --------------

h_p = 0.5
r = np.array([100000, 100000, 100000])
m = np.array([6000, 6000, 6000])
a = np.array([0, 300000, 50000])

# Mass
mass = m[1]

# Continuous-time state-space matrices
A_continuous = np.array([[-((1 + h_p) - (-1 + h_p) * (1 % 3) / 1) * (r[1] / m[1]), a[1]], [-1 / m[1], 0]])
B_continuous = np.array([[-1, (1 + h_p) * (r[0] / m[0]), -(-1 + h_p) * (r[2] / m[2]), 0, a[2]],
                         [0, 1 / m[0], 0, 0, 0]])
C_continuous = np.array([[1 / mass, 0]])
D_continuous = np.array([[0,0,0,0,0]])

# Discretize the system
dt = 1  # Adjust this according to your sampling time
A_discrete, B_discrete, C_discrete, D_discrete, _ = signal.cont2discrete(
    (A_continuous, B_continuous, C_continuous, D_continuous),
    dt=dt,
    method='zoh'
)


# -------------- Sensor --------------
v_0 = 0
v_1 = 0
d = 0

def read_sensor(tick):
    global v_0
    global d

    if tick < 5000:
        v_0 = tick*0.02 + np.random.randn() + math.sqrt(0.01)
        d = np.random.randn() + math.sqrt(0.01)
        return v_0, d
        #return np.array([[v_0], [d]])
    else:
        v_0 = 5000*0.02 + np.random.randn() + math.sqrt(0.01)
        d = np.random.randn() + math.sqrt(0.01)
        return v_0, d
        #return np.array([[v_0], [d]])


min_change = -0.3#-0.3
max_change = 0.08#0.08

def limit_sensor(old_sensor_value, new_sensor_value):
    change = min(max(new_sensor_value - old_sensor_value, min_change), max_change)
    return old_sensor_value + change


# -------------- Kalman filter --------------

#f = KalmanFilter (dim_x=2, dim_z=2, dim_u=0)
f = KalmanFilter (dim_x=2, dim_z=1, dim_u=0)

f.x = np.array([[0.],    # velocity
                [0.]])   # position
f.F = A_discrete
f.H = C_discrete
f.P *= 0.0001
f.R = 8
f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.5) # is this also used in the sensor read function to get velocity and position

#for tick in range(1000):
#    z = read_sensor(tick)
#    f.predict()
#    x = f.update(z)

zs = []
xs = []
v = 0
v_old = 0
limited_value = 0
for tick in range(10000):
    v_old = limited_value
    v, d = read_sensor(tick)
    limited_value = limit_sensor(v_old, v)
    #z = np.array([[v],[d]])
    #z = np.array([v])
    #xs.append(f.x)
    #f.predict()
    #f.update(z)
    zs.append(limited_value)
    xs.append(v)

#xs = [arr[0, 0] for arr in xs]
#print(xs)
plt.plot(xs, label='velocity')
plt.plot(zs, label='fuck kalman filter, velocity')
plt.legend(loc='best')
plt.show()