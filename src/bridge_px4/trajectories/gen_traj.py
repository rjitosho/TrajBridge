import numpy as np
from numpy import savetxt
import matplotlib.pyplot as plt


# Generate a trajectory
T = 10.0
t = np.arange(0.0, T, 0.05)

x = np.sin(2*np.pi*t*2/T)
y = np.cos(2*np.pi*t*3/T)
z = np.ones(len(t))

# plot trajectory
plt.plot(x,y)
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
np.savetxt('test.csv', X, delimiter=',', fmt='%1.3f')