import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from bezier2d import BezierCurve
import numpy as np

def homogeneous_transform_around_center(center: np.ndarray, axis: np.ndarray, theta: float) -> np.ndarray:
    """
    Create a 4x4 homogeneous transformation matrix for rotation around a center.

    Args:
        center: (cx, cy, cz) The point to rotate around.
        axis: (ux, uy, uz) Unit vector defining the axis of rotation.
        theta: Rotation angle in degrees.

    Returns:
        4x4 homogeneous transformation matrix.
    """
    center = np.array(center, dtype=float)
    axis = axis / np.linalg.norm(axis)  # Normalize rotation axis
    ux, uy, uz = axis

    theta = np.radians(theta)  # Convert to radians
    c = np.cos(theta)
    s = np.sin(theta)
    t = 1 - c

    # Rotation matrix (Rodrigues' formula)
    R = np.array([
        [t*ux*ux + c,      t*ux*uy - uz*s, t*ux*uz + uy*s],
        [t*ux*uy + uz*s,   t*uy*uy + c,    t*uy*uz - ux*s],
        [t*ux*uz - uy*s,   t*uy*uz + ux*s, t*uz*uz + c]
    ])

    # Homogeneous transformation
    T_to_origin = np.eye(4)
    T_to_origin[:3, 3] = -center

    T_rotation = np.eye(4)
    T_rotation[:3, :3] = R

    T_back = np.eye(4)
    T_back[:3, 3] = center

    T = T_back @ T_rotation @ T_to_origin  # Full transformation matrix
    return T

def apply_transform_to_point(T: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transformation matrix to a 3D point."""
    point_h = np.append(point, 1)  # Convert to homogeneous coordinates
    transformed_point_h = T @ point_h
    return transformed_point_h[:3]  # Extract (x, y, z)


def translate_point_along_leg_direction(point: np.ndarray, theta_offset: float, theta: float, distance: float) -> np.ndarray:
    """Translate a point along a leg's direction, adjusted by theta."""
    total_theta = np.radians(theta_offset + theta)
    direction = np.array([np.cos(total_theta), np.sin(total_theta), 0])  # XY-plane only
    translation = direction * distance
    return point + translation


# Define rotation parameters
center = np.array([0, 0, 0])  # Rotate around the hexapod center
axis = np.array([0, 0, 1])    # Rotate around the Z-axis
theta = -55.691                 # Rotation angle in degrees

# Setup control points (from your given code)
start = np.array([-97,0,0])

#start = np.array([-60.5, -89, 0])
# control_points = {
#     "start": start + np.array([50, -50, -25]),
#     "lift": start + np.array([50, -50, -75]),
#     "peak": start + np.array([150, -150, -200]),
#     "lower": start + np.array([200, -200, -75]),
#     "touchdown": start + np.array([100, -100, -25]),
#     "grounded": start + np.array([50, -50, 0]),
#     "sliding": start + np.array([10, -10, 0]),
#     "return": start
# }


# control_points = {
#     "start": start,
#     "lift": start + np.array([10, 0, 25]),
#     "peak": start + np.array([20, 0, 75]),
#     "lower": start + np.array([25, 0, 25]),  # The moment before grounding
#     "touchdown": start + np.array([25, 0, 25]),  # The moment before grounding
#     "grounded": start + np.array([25, 0, 0]),    # Fully on the ground
#     "sliding": start+ np.array([35, 0, 0]),     # Sliding before reset
#     "return":start,
# }

control_points = {
    "start": start,
    "lift": start + np.array([0, 10, 25]),
    "peak": start + np.array([0, 20, 75]),
    "lower": start + np.array([0, 25, 25]),  # The moment before grounding
    "touchdown": start + np.array([0, 25, 25]),  # The moment before grounding
    "grounded": start + np.array([0, 25, 0]),    # Fully on the ground
    "sliding": start+ np.array([0, 35, 0]),     # Sliding before reset
    "return":start,
}


bezier = BezierCurve(control_points, num_pts=100)
bezier = bezier.curve()

# Compute the transformation matrix

T = homogeneous_transform_around_center(center, axis, theta)

# Apply transformation to all control points
# rotated_control_points = {key: apply_transform_to_point(T, point) for key, point in bezier.items()}
rotated_control_points = np.array([apply_transform_to_point(T, point) for point in bezier])

# Print the transformed points
# for key, point in rotated_control_points.items():
#     print(f"{key}: {point}")

## Second set of rotated control points

center = np.array([0, 0, 0])  # Rotate around the hexapod center
axis = np.array([0, 0, 1])    # Rotate around the Z-axis
theta = 45                    # Rotation angle in degrees

T = homogeneous_transform_around_center(center, axis, theta)
rotatedv2 = np.array


# Create 3D plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Plot original control points (before rotation)
original_points = np.array(list(control_points.values()))
ax.scatter(original_points[:, 0], original_points[:, 1], original_points[:, 2], color='blue', label="Original Points")
ax.plot(original_points[:, 0], original_points[:, 1], original_points[:, 2], 'b-', alpha=0.5)


#|||| FOR THE DICTIONARY ||||
# Plot rotated control points (after rotation) 
# rotated_points = np.array(list(rotated_control_points.values()))
# ax.scatter(rotated_points[:, 0], rotated_points[:, 1], rotated_points[:, 2], color='red', label="Rotated Points")
# ax.plot(rotated_points[:, 0], rotated_points[:, 1], rotated_points[:, 2], 'r-', alpha=0.5)

# Plot rotated Bezier curve points (after rotation)
ax.scatter(rotated_control_points[:, 0], rotated_control_points[:, 1], rotated_control_points[:, 2], color='red', label="Rotated Bezier Points")
ax.plot(rotated_control_points[:, 0], rotated_control_points[:, 1], rotated_control_points[:, 2], 'r-', alpha=0.5)


# Plot the center of rotation
ax.scatter(*center, color='green', s=100, label="Rotation Center")

# Labels and legend
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")
ax.set_title("3D Rotation of Control Points Around Center")
ax.legend()

plt.show()
