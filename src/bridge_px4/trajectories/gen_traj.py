import numpy as np
from numpy import savetxt
import matplotlib.pyplot as plt

def step(t0, DT, x, y=None):
    x[t0:t0+DT] = np.zeros(DT) 
    x[t0+DT:t0+2*DT] = np.ones(DT)
    x[t0+2*DT:t0+3*DT] = np.zeros(DT)
    x[t0+3*DT:t0+4*DT] = -np.ones(DT)

    if y is not None:
        y[t0:t0+DT] = np.zeros(DT)
        y[t0+DT:t0+2*DT] = np.ones(DT)
        y[t0+2*DT:t0+3*DT] = np.zeros(DT)
        y[t0+3*DT:t0+4*DT] = -np.ones(DT)
    return x, y

def pattern(t0, dt, wait, period, x, y, z):
    t = np.arange(0.0, period, dt)
    t1 = int(t0+wait/dt)
    t2 = int(t0+wait/dt+period/dt)

    x[t0:t1] = np.zeros(t1-t0)
    y[t0:t1] = np.ones(t1-t0)
    z[t0:t1] = 1.5*np.ones(t1-t0)

    x[t1:t2] = np.sin(2*np.pi*t*2/period)
    y[t1:t2] = np.cos(2*np.pi*t*3/period)
    z[t1:t2] = 1.5 + 0.5*np.sin(2*np.pi*t*1/period)
    return x, y, z

# sinusoids
# T = 10.0
# t = np.arange(0.0, T, 0.05)
# x = np.sin(2*np.pi*t*2/T)
# y = np.cos(2*np.pi*t*3/T)
# z = np.ones(len(t))

# vine preland - go diagonally in x and z
# T = 100.0
# t = np.arange(0.0, T, 0.05)
# x = np.minimum(.3*t, 1.2*np.ones(len(t)))
# y = np.zeros(len(t))
# z = np.maximum(1.5-.3*t, 0.3*np.ones(len(t)))

# sysID suite
dt = 0.05
T_step = 4
T_step_period = 2.0
T_wait = 2.0
T_pattern = 15.0

T = T_step * 6 + T_wait + T_pattern
t = np.arange(0.0, T, dt)
x = np.zeros(len(t))
y = np.zeros(len(t))
z = np.zeros(len(t))
DT = int(T_step_period/2/dt)

x = step(0, DT, x)[0]
y = step(4*DT, DT, y)[0]
z = step(8*DT, DT, z)[0]

x,y = step(12*DT, DT, x, y)
x,z = step(16*DT, DT, x, z)
y,z = step(20*DT, DT, y, z)

x,y,z = pattern(24*DT, dt, T_wait, T_pattern, x, y, z)

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

# save trajectory
np.savetxt('sysID.csv', X, delimiter=',', fmt='%1.3f')