import numpy as np
from scipy import signal
from gekko import GEKKO

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


# -------------- MPC --------------

m = GEKKO()
#x,y,u = m.state_space(A_continuous,B_continuous,C_continuous,D=None)
x,y,u = m.state_space(A_discrete,B_discrete,C_discrete,D=None)

# customize names
# MVs
v0 = u[0] # feedforward
p_f = u[1]
p_b = u[2]
d_f = u[3]
d_b = u[4]

# CVs
p_i = y[0]
#d_i = y[1]

# Parameters
mass = 6000
#b = m.Param(value=50)
#K = m.Param(value=0.8)


u[0].lower = 0 # the lower bound. will not be less then
u[0].upper = 60 # the upper bound. will not be greater then
#u[0].dcost = 1
u[0].status = 1 # Feedback

u[1].lower = 0 # the lower bound. will not be less then
u[1].upper = 60*mass # the upper bound. will not be greater then
#u[1].dcost = 1
u[1].status = 1 # Feedback

u[2].lower = 0 # the lower bound. will not be less then
u[2].upper = 60*mass # the upper bound. will not be greater then
u[2].dcost = 1
u[2].status = 1 # MV

u[3].lower = -0.2 # the lower bound. will not be less then
u[3].upper = 5 # the upper bound. will not be greater then
#u[3].dcost = 0.5
u[3].status = 1 # MV

u[4].lower = -0.2 # the lower bound. will not be less then
u[4].upper = 5 # the upper bound. will not be greater then
#u[4].dcost = 0.005
u[4].status = 1 # MV


## CV tuning
# tau = first order time constant for trajectories
y[0].tau = 0.3 # first order time constant for trajectories
#y[1].tau = 60
# tr_init = 0 (dead-band)
#         = 1 (first order trajectory)
#         = 2 (first order traj, re-center with each cycle)
#y[1].tr_init = 0
# targets (dead-band needs upper and lower values)
y[0].upper = 100
y[0].lower = 0
y[0].sphi= 0.03*10 # SPHI = upper set point
y[0].splo= 0.01*10 # SPLO = lower set point
y[0].sp = 0.02*10 # SPLO = lower set point
#y[1].sphi= 5.4
#y[1].splo= 4.6
y[0].status = 1
y[0].tr_init = 0 # dead band = 0
#y[0].value = 0 # dead band
#y[1].status = 1

m.options.imode = 6 # CTL
m.options.nodes = 5 # Higher better accuracy but longer time
m.time = np.linspace(0,10,10) #[0, 0.1, 0.2, 0.4, 1, 1.5, 2, 3, 4]

# feedforward
u[0].status = 0
u[1].status = 0
u[0].value = np.zeros(np.size(m.time))
u[1].value = np.zeros(np.size(m.time))
for i in range(np.size(m.time)):
    u[0].value[i] = 0.02*i + 1
    u[1].value[i] = 0.02*i*mass + 1*mass

#m.solve(disp=False) # (GUI=True)
m.solve(disp=False) # (GUI=True)

print(u[0].value)
print(y[0].value)

# get additional solution information
import json
with open(m.path+'//results.json') as f:
    results = json.load(f)

print(results)


import matplotlib.pyplot as plt

# should v0 be the trajectory?
plt.subplot(2,1,1)
plt.plot(m.time,v0.value,'r-',label=r'$u_0$ as feedforward')
plt.plot(m.time,p_f.value,'b--',label=r'$u_1$ as feedforward')
plt.plot(m.time,p_b.value,'b:',label=r'$u_2$ as MV')
plt.plot(m.time,d_f.value,'g:',label=r'$u_3$ as MV')
plt.plot(m.time,d_b.value,'g:',label=r'$u_4$ as MV')
plt.legend()
plt.subplot(2,1,2)
plt.plot(m.time,p_i.value,'r-',label=r'$y_0$')
plt.legend()
plt.show()