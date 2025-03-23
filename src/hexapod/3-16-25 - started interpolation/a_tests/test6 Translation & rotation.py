import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.special import comb  


class BezierCurve:
    def __init__(self, control_points, num_pts=100):
        self.control_points = list(control_points.values())
        self.num_pts = num_pts

    def curve(self):
        t = np.linspace(0, 1, self.num_pts)
        n = len(self.control_points) - 1
        curve_points = np.zeros((self.num_pts, 3))
        for i in range(self.num_pts):
            point = np.zeros(3)
            for j, cp in enumerate(self.control_points):
                b = comb(n, j) * (t[i]**j) * ((1 - t[i])**(n - j))
                point += b * cp
            curve_points[i] = point
        return curve_points

def get_radial_direction(coxa_pos: np.ndarray) -> np.ndarray:
    """Compute the radial direction unit vector from center [0, 0, 0] to Coxa in XY-plane."""
    direction = np.array([coxa_pos[0], coxa_pos[1], 0])
    return direction / np.linalg.norm(direction)


def adjust_point_away_from_coxa(point: np.ndarray, direction: np.ndarray, distance: float) -> np.ndarray:
    """Adjust a point away from the Coxa along the radial direction."""
    translation = direction * distance
    return point + translation

def homogeneous_transform_around_center(center: np.ndarray, axis: np.ndarray, theta: float) -> np.ndarray:
    """Create a 4x4 homogeneous transformation matrix for rotation around a center."""
    center = np.array(center, dtype=float)
    axis = axis / np.linalg.norm(axis)
    ux, uy, uz = axis
    theta = np.radians(theta)  # Convert degrees to radians
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
    return T_back @ T_rotation @ T_to_origin

def apply_transform_to_point(T: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transformation matrix to a 3D point."""
    point_h = np.append(point, 1)
    transformed_point_h = T @ point_h
    return transformed_point_h[:3]

def rotate_bezier_curve(curve: np.ndarray, center: np.ndarray, theta: float) -> np.ndarray:
    """Rotate a Bézier curve around a center point on the Z-axis."""
    axis = np.array([0, 0, 1])  # Z-axis
    T = homogeneous_transform_around_center(center, axis, theta)
    return np.array([apply_transform_to_point(T, point) for point in curve])

# Hexapod leg data
leg_data = {
    "LR": {"coxa_coord": np.array([-60.5, -89, 0]), "theta_offset": 214.309},
    "LM": {"coxa_coord": np.array([-97, 0, 0]), "theta_offset": 270},
    "LF": {"coxa_coord": np.array([-60.5, 89, 0]), "theta_offset": 325.931},
    "RF": {"coxa_coord": np.array([60.5, 89, 0]), "theta_offset": 34.0685},
    "RM": {"coxa_coord": np.array([97, 0, 0]), "theta_offset": 90},
    "RR": {"coxa_coord": np.array([60.5, -89, 0]), "theta_offset": 147.172}
}

# Neutral effector offset
neutral_effector_offset = np.array([0, 0, -160])

# Define control points relative to start position
def get_bezier_control_points(start_pos: np.ndarray) -> dict:
    """Define control points for a Bézier curve."""
    return {
        "start": start_pos,
        "lift": start_pos + np.array([0, 20, 25]),
        "peak": start_pos + np.array([0, 50, 75]),
        "lower": start_pos + np.array([0, 75, 25]),
        "touchdown": start_pos + np.array([0, 85, 25]),
        "grounded": start_pos + np.array([0, 85, 0]),
        "sliding": start_pos + np.array([0, 35, 0]),
        "return": start_pos,
    }

# Parameters
distance_away = 100.0  # Distance to shift outward from Coxa
rotation_angle = 45.0  # Rotation angle in degrees (positive = counterclockwise, negative = clockwise)

# Compute Bézier curves
original_curves = {}
translated_curves = {}
rotated_curves = {}
for leg_name, data in leg_data.items():
    coxa_pos = data["coxa_coord"]
    start_pos = coxa_pos + neutral_effector_offset
    print(f"start_pos: {start_pos}")
    # Original Bézier curve
    control_points = get_bezier_control_points(start_pos)
    bezier = BezierCurve(control_points, num_pts=100)
    original_curves[leg_name] = bezier.curve()
    
    # Translated Bézier curve (shifted away from Coxa)
    radial_dir = get_radial_direction(coxa_pos)
    translated_start = adjust_point_away_from_coxa(start_pos, radial_dir, distance_away)
    translated_control_points = get_bezier_control_points(translated_start)
    translated_bezier = BezierCurve(translated_control_points, num_pts=100)
    translated_curves[leg_name] = translated_bezier.curve()
    
    # Rotated Bézier curve (around translated start position)
    rotated_curves[leg_name] = rotate_bezier_curve(translated_curves[leg_name], translated_start, rotation_angle)

# Create 3D plot
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Plot hexapod center
center = np.array([0, 0, 0])
ax.scatter(*center, color='black', s=100, label="Hexapod Center")

# Colors for each leg
colors = {'RR': 'b', 'LR': 'm', 'RF': 'g', 'LF': 'r', 'LM': 'c', 'RM': 'y'}

# Plot Coxa positions and curves
for leg_name, data in leg_data.items():
    coxa_pos = data["coxa_coord"]
    orig_curve = original_curves[leg_name]
    trans_curve = translated_curves[leg_name]
    rot_curve = rotated_curves[leg_name]
    
    # Coxa position
    ax.scatter(coxa_pos[0], coxa_pos[1], coxa_pos[2], color=colors[leg_name], label=leg_name)
    # Original curve
    ax.plot(orig_curve[:, 0], orig_curve[:, 1], orig_curve[:, 2], '-', color=colors[leg_name], alpha=0.3)
    # Translated curve
    ax.plot(trans_curve[:, 0], trans_curve[:, 1], trans_curve[:, 2], '--', color=colors[leg_name], alpha=0.5)
    # Rotated curve
    ax.plot(rot_curve[:, 0], rot_curve[:, 1], rot_curve[:, 2], ':', color=colors[leg_name], 
            label=f'{leg_name} Rotated {rotation_angle}°')

# Labels and legend
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")
ax.set_title(f"3D Bézier Curves Translated and Rotated {rotation_angle}° Around Z-Axis")
ax.legend()

# Set axis limits
ax.set_xlim(-150, 150)
ax.set_ylim(-150, 150)
ax.set_zlim(-170, 80)

plt.show()