import numpy as np

# Load the CSV file
address = 'Traj0604_test.csv'
data = np.genfromtxt(address, delimiter=',')

# Subtract 0.1 from each element in the 4th row (index 3)
data[3, :] -= 0.1

# Save the modified array to a new CSV file
output_address = 'output_test.csv'
np.savetxt(output_address, data, delimiter=',', fmt='%1.3f')
