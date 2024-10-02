import numpy as np
import matplotlib.pyplot as plt
from numpy import savetxt

# Parameters
T = 20 * 60
initial_period = 12
final_period = 4
ramp_duration = 60
time_step = 0.05

# Time vector
t = np.arange(0, T * time_step, time_step)

# Calculate period at each time step
period = initial_period - ((initial_period - final_period) / ramp_duration) * np.minimum(t, ramp_duration)

# Calculate angle theta using the period
theta = np.cumsum(2 * np.pi / period) * time_step  # Integrate the period to get theta

# Calculate x, y, and z coordinates of the trajectory
x = np.sin(theta)
y = np.cos(theta) * np.sin(theta)
z = 1.5 * np.ones_like(x) + 0.25 * np.sin(2 * np.pi * t / 15)

# Plot x vs y for the figure 8 trajectory
plt.figure()
plt.plot(x, y, label="desired trajectory")

# Plot x, y, and z over time
plt.figure()
plt.plot(t, x, label="x (m)")
plt.plot(t, y, label="y (m)")
plt.plot(t, z, label="z (m)")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Trajectory Over Time")
plt.legend()

# Show the plots
plt.show()

# save trajectory
X = np.zeros((14,len(t)))
X[0,:] = t
X[1,:] = x
X[2,:] = y
X[3,:] = z
X[7,:] = np.ones(len(t))
np.savetxt('fig8_period12to4in60s.csv', X, delimiter=',', fmt='%1.3f')