import numpy as np
from numpy import savetxt
import matplotlib.pyplot as plt


# Generate a trajectory
T = 100.0
t = np.arange(0.0, T, 0.05)

# sinusoids
# x = np.sin(2*np.pi*t*2/T)
# y = np.cos(2*np.pi*t*3/T)
# z = np.ones(len(t))

# vine preland - go diagonally in x and z
x = np.minimum(.3*t, 1.2*np.ones(len(t)))
y = np.zeros(len(t))
z = np.maximum(1.5-.3*t, 0.3*np.ones(len(t)))

# plot trajectory
plt.plot(x,z)
plt.axis('equal')
plt.show()

# assemble trajectory
X = np.zeros((14,len(t)))
X[0,:] = t
X[1,:] = x
X[2,:] = y
X[3,:] = z
X[7,:] = np.ones(len(t))

# save trajectory
np.savetxt('preland.csv', X, delimiter=',', fmt='%1.3f')