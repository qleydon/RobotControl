import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Generate some sample data for 3D plot
x = [1.0, 2.0, 3.0, 4.0]
y = [1.0, 2.0, 3.0, 4.0]
z = [1.0, 2.0, 3.0, 4.0]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the 3D data
ax.plot(x, y, z, color='blue', linewidth=1.0, label='3D Plot')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show the legend
ax.legend()

# Show the plot
plt.show()