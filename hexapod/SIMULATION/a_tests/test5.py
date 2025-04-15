import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.special import comb

# Simple Bézier curve implementation
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
    direction = np.array([coxa_pos[0], coxa_pos[1], 0])  # Ignore Z
    return direction / np.linalg.norm(direction)

def adjust_point_away_from_coxa(point: np.ndarray, direction: np.ndarray, distance: float) -> np.ndarray:
    """Adjust a point away from the Coxa along the radial direction."""
    translation = direction * distance
    return point + translation

# Hexapod leg data
leg_data = {
    "RR": {"coxa_coord": np.array([60.5, -89, 0]), "theta_offset": 147.172},
    "LR": {"coxa_coord": np.array([-60.5, -89, 0]), "theta_offset": 214.309},
    "RF": {"coxa_coord": np.array([60.5, 89, 0]), "theta_offset": 34.0685},
    "LF": {"coxa_coord": np.array([-60.5, 89, 0]), "theta_offset": 325.931},
    "LM": {"coxa_coord": np.array([-97, 0, 0]), "theta_offset": 270},
    "RM": {"coxa_coord": np.array([97, 0, 0]), "theta_offset": 90}
}

# Neutral effector offset
neutral_effector_offset = np.array([0, 0, -100])

# Define control points relative to start position
def get_bezier_control_points(start_pos: np.ndarray) -> dict:
    """Define control points for a Bézier curve."""
    return {
        "start": start_pos,
        "lift": start_pos + np.array([0, 0, 50]),
        "peak": start_pos + np.array([0, 30, 95]),
        "lower": start_pos + np.array([0, 45, 55]),
        "touchdown": start_pos + np.array([0, 55, 55]),
        "grounded": start_pos + np.array([0, 65, 0]),
        "sliding": start_pos + np.array([0, 65, 0]),
        "return": start_pos,
    }

# Adjustment parameters
distance_away = 30.0  # Distance to shift outward from Coxa

# Compute Bézier curves
original_curves = {}
adjusted_curves = {}
for leg_name, data in leg_data.items():
    coxa_pos = data["coxa_coord"]
    start_pos = coxa_pos + neutral_effector_offset
    
    # Original Bézier curve
    control_points = get_bezier_control_points(start_pos)
    bezier = BezierCurve(control_points, num_pts=100)
    original_curves[leg_name] = bezier.curve()
    
    # Adjusted Bézier curve (shifted away from Coxa)
    radial_dir = get_radial_direction(coxa_pos)
    adjusted_start = adjust_point_away_from_coxa(start_pos, radial_dir, distance_away)
    adjusted_control_points = get_bezier_control_points(adjusted_start)
    adjusted_bezier = BezierCurve(adjusted_control_points, num_pts=100)
    adjusted_curves[leg_name] = adjusted_bezier.curve()

# Create 3D plot
fig = plt.figure(figsize=(10, 8))
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
    adj_curve = adjusted_curves[leg_name]
    
    # Coxa position
    ax.scatter(coxa_pos[0], coxa_pos[1], coxa_pos[2], color=colors[leg_name], label=leg_name)
    # Original curve
    ax.plot(orig_curve[:, 0], orig_curve[:, 1], orig_curve[:, 2], '-', color=colors[leg_name], alpha=0.5)
    # Adjusted curve
    ax.plot(adj_curve[:, 0], adj_curve[:, 1], adj_curve[:, 2], '--', color=colors[leg_name], 
            label=f'{leg_name} Adjusted')

# Labels and legend
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")
ax.set_title(f"3D Bézier Curves Adjusted {distance_away} Units Away from Coxa")
ax.legend()

# Set axis limits
ax.set_xlim(-150, 150)
ax.set_ylim(-150, 150)
ax.set_zlim(-170, 80)

plt.show()