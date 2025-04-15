import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

def homogeneous_transform_around_center(center: np.ndarray, axis: np.ndarray, theta: float) -> np.ndarray:
    """Create a 4x4 homogeneous transformation matrix."""
    center = np.array(center, dtype=float)
    axis = axis / np.linalg.norm(axis)
    ux, uy, uz = axis
    c = np.cos(theta)
    s = np.sin(theta)
    t = 1 - c
    R = np.array([
        [t*ux*ux + c,      t*ux*uy - uz*s, t*ux*uz + uy*s],
        [t*ux*uy + uz*s,   t*uy*uy + c,    t*uy*uz - ux*s],
        [t*ux*uz - uy*s,   t*uy*uz + ux*s, t*uz*uz + c]
    ])
    T_to_origin = np.eye(4)
    T_to_origin[:3, 3] = -center
    T_rotation = np.eye(4)
    T_rotation[:3, :3] = R
    T_back = np.eye(4)
    T_back[:3, 3] = center
    T = T_back @ T_rotation @ T_to_origin
    return T

def apply_transform_to_point(T: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Apply the transformation matrix to a 3D point."""
    point_h = np.append(point, 1)
    point_transformed_h = T @ point_h
    return point_transformed_h[:3]

# Define the square and parameters
center = np.array([1.0, 2.0, 3.0])  # Center of rotation
axis = np.array([0.0, 0.0, 1.0])    # Z-axis
side_length = 1.0

# Square vertices (in XY-plane at Z=3, centered around [1, 2, 3])
square_base = np.array([
    [0.5, 0.5, 0],    # Top-right
    [-0.5, 0.5, 0],   # Top-left
    [-0.5, -0.5, 0],  # Bottom-left
    [0.5, -0.5, 0]    # Bottom-right
])
square = square_base + center
square = np.vstack([square, square[0]])  # Close the square

# Set up the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim([-1, 3])
ax.set_ylim([-1, 4])
ax.set_zlim([2, 4])
ax.grid(True)

# Plot the center point
center_plot = ax.plot([center[0]], [center[1]], [center[2]], 'ro', markersize=10, label='Center')[0]

# Initial square plot (will be updated)
square_plot, = ax.plot(square[:, 0], square[:, 1], square[:, 2], 'b-', linewidth=2, label='Square')
ax.legend()

# Animation function
def update(frame):
    theta = 2 * np.pi * frame / 100  # 0 to 360 degrees over 100 frames
    T = homogeneous_transform_around_center(center, axis, theta)
    
    # Apply transformation to each vertex
    transformed_square = np.array([apply_transform_to_point(T, p) for p in square])
    
    # Update the square plot
    square_plot.set_data(transformed_square[:, 0], transformed_square[:, 1])
    square_plot.set_3d_properties(transformed_square[:, 2])
    
    return square_plot,

# Create animation
ani = animation.FuncAnimation(fig, update, frames=100, interval=50, blit=True)

plt.show()