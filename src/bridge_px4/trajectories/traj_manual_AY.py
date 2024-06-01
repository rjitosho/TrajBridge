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

x0 = traj_u_step(0, 5, 0)
x1 = traj_u_line(5, 5.5, 0, 1.5)
x2 = traj_u_step(5.5, 5.75, 1.5)
x3 = traj_u_line(5.75, 6.25, 1.5, 0)
x4 = traj_u_step(6.25, 13, 0)
x = np.concatenate((x0, x1, x2, x3, x4))
t = time(0, 13)

y = np.zeros(len(t))
z = 1.75*np.ones(len(t))

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
plt.show()

# fig, ax = plt.subplots()
# ax.plot(X[1,:], X[2,:])
# plt.show()

# save trajectory
np.savetxt('/home/oem/StanfordMSL/TrajBridge/src/bridge_px4/trajectories/Traj0531_pip1.csv', X, delimiter=',', fmt='%1.3f')