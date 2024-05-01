import numpy as np
from numpy import savetxt
import matplotlib.pyplot as plt

def step(t0, DT, x, y=None, x_offset=0, y_offset=0, x_scale=1, y_scale=1):
    x[t0:t0+DT] = np.zeros(DT) + x_offset
    x[t0+DT:t0+2*DT] = x_scale*np.ones(DT) + x_offset
    x[t0+2*DT:t0+3*DT] = np.zeros(DT) + x_offset
    x[t0+3*DT:t0+4*DT] = -x_scale*np.ones(DT) + x_offset

    if y is not None:
        y[t0:t0+DT] = np.zeros(DT) + y_offset
        y[t0+DT:t0+2*DT] = y_scale*np.ones(DT) + y_offset
        y[t0+2*DT:t0+3*DT] = np.zeros(DT) + y_offset
        y[t0+3*DT:t0+4*DT] = -y_scale*np.ones(DT) + y_offset
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

# figure 8
# T = 10.0
# t = np.arange(0.0, T, 0.05)
# period = 10.0
# x = np.sin(2*np.pi/period*t)
# y = np.sin(2*np.pi/period*t) * np.cos(2*np.pi/period*t)
# z = 0.5*np.ones(len(t))

# cirlce
# T = 10.0
# t = np.arange(0.0, T, 0.05)
# x = np.sin(t)
# y = np.cos(t)
# z = 0.5*np.ones(len(t))

# sinusoids
# T = 9.0
# t = np.arange(0.0, T, 0.05)
# x = np.sin(2*np.pi*t*2/T)
# y = np.cos(2*np.pi*t*3/T)
# z = 1.5*np.ones(len(t))

# vine preland - go diagonally in x and z
# T = 100.0
# t = np.arange(0.0, T, 0.05)
# x = np.minimum(.4*t, 1.75*np.ones(len(t)))
# y = np.zeros(len(t))
# z = np.maximum(1.5-.3*t, 0.15*np.ones(len(t)))

# sysID suite
dt = 0.05
T_step = 8.0
T_wait = 2.0
T_pattern = 15.0
T_swinging = 13.0

T = T_step * 3 + T_wait + T_pattern + T_swinging
t = np.arange(0.0, T, dt)
x = np.zeros(len(t))
y = np.zeros(len(t))
z = 1.5*np.ones(len(t))
DT = int(T_step/4/dt)

x = step(0, DT, x)[0]
y = step(4*DT, DT, y)[0]
z = step(8*DT, DT, z, x_offset=1.5, x_scale=.4)[0]

x,y,z = pattern(12*DT, dt, T_wait, T_pattern, x, y, z)

# swinging
t_start = int(21*DT)
t_no_offset = t[t_start:] - t[t_start]
# x[t_start:] = 0.1*t_no_offset*np.sin(2*np.pi*t_no_offset*6/T_swinging) # v2
# x[t_start:] = 0.2*t_no_offset*np.sin(2*np.pi*t_no_offset*6/T_swinging) # v3
x[t_start:] = 0.2*t_no_offset*np.sin(2*np.pi*t_no_offset*6/10) # v4
z[t_start:] = 1.5*np.ones(len(t_no_offset))

# moving growing sinusoids
# T = 10.0
# t = np.arange(0.0, T, 0.05)
# x = 0.2*t
# y = 0.1*t*np.sin(2*np.pi*t*6/T)
# z = 1.5*np.ones(len(t))

# handmade swingup
# T = 10.0
# t = np.arange(0.0, T, 0.05)
# x = 0.25*t*np.sin(2*np.pi*t*6/T)
# x[108:] = x[108] + 0.25*(t[108:]-t[108])
# x[128:] = 0*x[128:] + x[128]
# y = 0*t
# z = 1.5*np.ones(len(t))

# sinusoids
# T = 10.0
# t = np.arange(0.0, T, 0.05)
# x = np.clip(0.3*t, 0, 1)*np.sin(2*np.pi*t*6/T)
# y = 0*t
# z = 1.5*np.ones(len(t))

# # shifting sinusoid
# x[150:] = x[150:] + t[150:] - t[150]

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

fig, ax = plt.subplots()
ax.plot(X[1,:], X[2,:])
plt.show()

# save trajectory
np.savetxt('sysID_w_swinging_v4.csv', X, delimiter=',', fmt='%1.3f')