import numpy as np
import matplotlib.pyplot as plt

def rotate_bezier(curve_points, center, angle_degrees):
    """Rotates a Bezier curve around a center point.

    Args:
        curve_points: A list of tuples representing the Bezier curve's control points [(x0, y0), (x1, y1), ...].
        center: A tuple representing the center of rotation (cx, cy).
        angle_degrees: The rotation angle in degrees.

    Returns:
        A list of tuples representing the rotated Bezier curve's control points.
    """
    cx, cy = center
    angle_radians = np.radians(angle_degrees)
    cos_theta = np.cos(angle_radians)
    sin_theta = np.sin(angle_radians)

    # Rotation matrix
    rotation_matrix = np.array([
        [cos_theta, -sin_theta, cx * (1 - cos_theta) + cy * sin_theta],
        [sin_theta, cos_theta, cy * (1 - cos_theta) - cx * sin_theta],
        [0, 0, 1]
    ])

    rotated_points = []
    for x, y in curve_points:
        # Convert to homogeneous coordinates
        point_matrix = np.array([[x], [y], [1]])
        
        # Apply rotation
        rotated_matrix = np.dot(rotation_matrix, point_matrix)
        
        # Convert back to Cartesian coordinates
        rotated_x, rotated_y = rotated_matrix[0, 0], rotated_matrix[1, 0]
        rotated_points.append((rotated_x, rotated_y))

    return rotated_points

# Example Usage:
curve_points = [(1, 1), (2, 4), (5, 3), (6, 1)]
center = (3, 2)
angle_degrees = 90

rotated_curve = rotate_bezier(curve_points, center, angle_degrees)

print("Original curve points:", curve_points)
print("Rotated curve points:", rotated_curve)

# Plotting for visualization
x, y = zip(*curve_points)
rx, ry = zip(*rotated_curve)

plt.plot(x, y, marker='o', linestyle='-', label='Original Bezier Curve')
plt.plot(rx, ry, marker='x', linestyle='--', label='Rotated Bezier Curve')
plt.plot(center[0], center[1], marker='*', color='red', label='Rotation Center')
plt.legend()
plt.title('Rotating Bezier Curve')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.grid(True)
plt.show()