import numpy as np
from gekko import GEKKO

A = np.array([[-.003, 0.039, 0, -0.322],
              [-0.065, -0.319, 7.74, 0],
              [0.020, -0.101, -0.429, 0],
              [0, 0, 1, 0]])

B = np.array([[0.01, 1, 2],
              [-0.18, -0.04, 2],
              [-1.16, 0.598, 2],
              [0, 0, 2]]
            )

C = np.array([[1, 0, 0, 0],
              [0, -1, 0, 7.74]])

#%% Build GEKKO State Space model
m = GEKKO()
x,y,u = m.state_space(A,B,C,D=None)

# customize names
# MVs
mv0 = u[0]
mv1 = u[1]
# Feedforward
ff0 = u[2]
# CVs
cv0 = y[0]
cv1 = y[1]

m.time = [0, 0.1, 0.2, 0.4, 1, 1.5, 2, 3, 4]
m.options.imode = 6
m.options.nodes = 3

u[0].lower = -5
u[0].upper = 5
u[0].dcost = 1
u[0].status = 1

u[1].lower = -5
u[1].upper = 5
u[1].dcost = 1
u[1].status = 1

## CV tuning
# tau = first order time constant for trajectories
y[0].tau = 5
y[1].tau = 8
# tr_init = 0 (dead-band)
#         = 1 (first order trajectory)
#         = 2 (first order traj, re-center with each cycle)
y[0].tr_init = 0
y[1].tr_init = 0
# targets (dead-band needs upper and lower values)
# SPHI = upper set point
# SPLO = lower set point
y[0].sphi= -8.5
y[0].splo= -9.5
y[1].sphi= 5.4
y[1].splo= 4.6

y[0].status = 1
y[1].status = 1

# feedforward
u[2].status = 0
u[2].value = np.zeros(np.size(m.time))
u[2].value[3:] = 2.5

m.solve() # (GUI=True)


# also create a Python plot
import matplotlib.pyplot as plt

plt.subplot(2,1,1)
plt.plot(m.time,mv0.value,'r-',label=r'$u_0$ as MV')
plt.plot(m.time,mv1.value,'b--',label=r'$u_1$ as MV')
plt.plot(m.time,ff0.value,'g:',label=r'$u_2$ as feedforward')
plt.legend()
plt.subplot(2,1,2)
plt.plot(m.time,cv0.value,'r-',label=r'$y_0$')
plt.plot(m.time,cv1.value,'b--',label=r'$y_1$')
plt.legend()
plt.show()