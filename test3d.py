import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

import matplotlib

matplotlib.use("TkAgg")

# Define the initial positions of the robot (example data)
x = [1, 2, 3, 4, 5, 6]
y = [2, 3, 4, 5, 6, 7]
z = [0, 1, 2, 3, 4, 5]

# Create a 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter(x, y, z, s=100)  # Set the marker size with the 's' parameter

# Set labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# Function to update the plot with new positions
def update(frame):
    # Update the robot's positions (modify this part according to your data)
    for i in range(len(x)):
        x[i] += 0.1
        y[i] += 0.1
        z[i] += 0.1

    # Update the scatter plot with new positions
    sc._offsets3d = (x, y, z)

# Create the animation
animation = FuncAnimation(fig, update, frames=range(100), interval=100)

# Save the animation as a GIF
#animation.save('robot_animation.gif', writer='imagemagick')

# Show the plot (optional)
plt.show()