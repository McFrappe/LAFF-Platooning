import numpy as np
from scipy import signal
import scipy.signal
import scipy.optimize
import matplotlib.pyplot as plt


# -------------- MPC Parameters --------------

M = 10  # Control horizon
P = 20  # Prediction horizon
DeltaT = 1  # Sampling rate


# -------------- Test system --------------

G = scipy.signal.lti([1], [15, 8, 1])
ss = G.to_ss()
x0 = np.zeros(ss.A.shape[0])
u = np.ones(M)
print("START Test system")
print("array Test system")
print([u, np.repeat(u[-1], P - M)])
print("concat Test system")
print(np.concatenate([u, np.repeat(u[-1], P - M)]))
print("END Test system")


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


# -------------- My system --------------

#ss = signal.StateSpace(A_discrete, B_discrete, C_discrete, D_discrete)
#x0 = np.zeros(ss.A.shape[0])
#u = np.ones((M, 5))
#print(x0)
#print("START My system")
#print("u array Test system")
#print([u])
#print("array My system")
#print([u, np.repeat(u[:, -1:], P - M, axis=1)])
#print("concat My system")
#print(np.concatenate([u, np.repeat(u[:, -1:], P - M, axis=1)], axis=1))
#print("hack My system")
#print(np.concatenate([u, np.tile(u[-1, :], (P-M, 1))]))
#print("END My system")


# -------------- MPC Implementation --------------

tcontinuous = np.linspace(0, P*DeltaT, 1000)  # some closely spaced time points
tpredict = np.arange(0, P*DeltaT, DeltaT)   # discrete points at prediction horizon
print(tpredict)

tau_c = 1
r = 1 - np.exp(-tpredict/tau_c)


def extend(u):
    """We optimise the first M values of u but we need P values for prediction"""
    return np.concatenate([u, np.tile(u[-1, :], (P-M, 1))])
    #return np.concatenate([u, np.repeat(u[-1], P-M)])
def prediction(u, t=tpredict, x0=x0):
    """Predict the effect of an input signal"""
    t, y, x = scipy.signal.lsim(ss, u, t, X0=x0, interp=False)
    return y

def objective(u, x0=x0):
    """Calculate the sum of the square error for the cotnrol problem"""
    print(u)
    print(x0)
    y = prediction(extend(u))
    return sum((r - y)**2)

def cost_function(u, x0=x0):
    pass

#print(extend(u))
#print(prediction(extend(u)))
print(objective(u))
print(extend(u))
print(x0)

print("AAAAAAAAAAAA")
result = scipy.optimize.minimize(objective, u)
uopt = result.x
#print(result.fun)

ucont = extend(uopt)[((tcontinuous-0.01)//DeltaT).astype(int)]


def plotoutput(ucont, uopt):
    plt.figure()
    plt.plot(tcontinuous, ucont)
    plt.xlim([0, DeltaT*(P+1)])
    plt.figure()
    plt.plot(tcontinuous, prediction(ucont, tcontinuous), label='Continuous response')
    plt.plot(tpredict, prediction(extend(uopt)), '-o', label='Optimized response')
    plt.plot(tpredict, r, label='Set point')
    plt.legend()
    plt.show()

plotoutput(ucont, uopt)
