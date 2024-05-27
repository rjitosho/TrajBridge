import numpy as np
from numpy import savetxt
import matplotlib.pyplot as plt
from scipy import signal

def time(t0, t1):
    return np.arange(t0, t1, 0.05)
# print(t0)

def traj_u_step(t0, t1, u):
    len = time(t0, t1).shape[0]
    return np.ones(len) * u

def traj_u_line(t0, t1, x0, x1):
    len = time(t0, t1).shape[0]
    return np.linspace(x0, x1, len)

x0 = traj_u_step(0, 7, 0)
x1 = traj_u_line(7, 8, 0, 3)
x2 = traj_u_step(8, 13, 3)
x = np.concatenate((x0, x1, x2))
t = time(0, 13)

y = np.zeros(len(t))
z = 1.5*np.ones(len(t))

# assemble trajectory
X = np.zeros((14,len(t)))
X[0,:] = t
X[1,:] = x
X[2,:] = y
X[3,:] = z
X[7,:] = np.ones(len(t))

# plot
fig, ax = plt.subplots()
ax.plot(t, X[1:4,:].T)
# plt.show()

# fig, ax = plt.subplots()
# ax.plot(X[1,:], X[2,:])
# plt.show()

# save trajectory
np.savetxt('Traj0527_3_1s.csv', X, delimiter=',', fmt='%1.3f')