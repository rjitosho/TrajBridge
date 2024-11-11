import numpy as np

# Load the CSV file
address = 'src/bridge_px4/trajectories/motion_plan_kick_2by2_NLtipz_empty_long_vine_x1.5_z0.75_T50.csv'
data = np.genfromtxt(address, delimiter=',')

# Subtract from each element in the 4th row (index 3)
data[3, :] -= 0.04

# Save the modified array to a new CSV file
output_address = 'src/bridge_px4/trajectories/motion_plan_kick_2by2_NLtipz_empty_long_vine_x1.5_z0.71_T50.csv'
np.savetxt(output_address, data, delimiter=',', fmt='%1.3f')
