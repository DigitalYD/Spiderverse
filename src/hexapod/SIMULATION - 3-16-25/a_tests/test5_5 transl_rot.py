import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.special import comb 


# Simple Bézier curve implementation (since bezier2d isn’t provided)
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

def translate_point_along_leg_direction(point: np.ndarray, theta_offset: float, theta: float, distance: float) -> np.ndarray:
    """Translate a point along a leg's direction, adjusted by theta."""
    total_theta = np.radians(theta_offset + theta)
    direction = np.array([np.cos(total_theta), np.sin(total_theta), 0])  # XY-plane only
    translation = direction * distance
    return point + translation

# Hexapod leg data (corrected diagonals)
leg_data = {
    "RR": {"coxa_coord": np.array([60.5, -89, 0]), "theta_offset": 147.172},
    "LR": {"coxa_coord": np.array([-60.5, -89, 0]), "theta_offset": 214.309},
    "RF": {"coxa_coord": np.array([60.5, 89, 0]), "theta_offset": 34.0685},
    "LF": {"coxa_coord": np.array([-60.5, 89, 0]), "theta_offset": 325.931},
    "LM": {"coxa_coord": np.array([-97, 0, 0]), "theta_offset": 270},
    "RM": {"coxa_coord": np.array([97, 0, 0]), "theta_offset": 90}
}

# Neutral effector offset (assumed from your Leg class)
neutral_effector_offset = np.array([0, 0, -160])

# Define control points relative to each leg’s effector position
def get_bezier_control_points(start_pos: np.ndarray) -> dict:
    """Define control points for a Bézier curve starting at the effector position."""
    return {
        "start": start_pos,
        "lift": start_pos + np.array([0, 10, 25]),
        "peak": start_pos + np.array([0, 20, 75]),
        "lower": start_pos + np.array([0, 25, 25]),
        "touchdown": start_pos + np.array([0, 25, 25]),
        "grounded": start_pos + np.array([0, 25, 0]),
        "sliding": start_pos + np.array([0, 35, 0]),
        "return": start_pos,
    }

# Translation parameters
theta = 0.0  # No additional rotation
distance = 20.0  # Translate 20 units forward

# Compute Bézier curves for all legs
original_curves = {}
translated_curves = {}
for leg_name, data in leg_data.items():
    coxa_pos = data["coxa_coord"]
    theta_offset = data["theta_offset"]
    start_pos = coxa_pos + neutral_effector_offset
    
    # Original Bézier curve
    control_points = get_bezier_control_points(start_pos)
    bezier = BezierCurve(control_points, num_pts=100)
    original_curves[leg_name] = bezier.curve()
    
    # Translated Bézier curve
    translated_curve = np.array([translate_point_along_leg_direction(point, theta_offset, theta, distance)
                                 for point in original_curves[leg_name]])
    translated_curves[leg_name] = translated_curve

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
    trans_curve = translated_curves[leg_name]
    
    # Coxa position
    ax.scatter(coxa_pos[0], coxa_pos[1], coxa_pos[2], color=colors[leg_name], label=leg_name)
    # Original curve
    ax.plot(orig_curve[:, 0], orig_curve[:, 1], orig_curve[:, 2], '-', color=colors[leg_name], alpha=0.5)
    # Translated curve
    ax.plot(trans_curve[:, 0], trans_curve[:, 1], trans_curve[:, 2], '--', color=colors[leg_name], 
            label=f'{leg_name} Translated')

# Labels and legend
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")
ax.set_title("3D Translation of Bézier Curves for All Legs")
ax.legend()

# Set axis limits
ax.set_xlim(-150, 150)
ax.set_ylim(-150, 150)
ax.set_zlim(-170, 80)

plt.show()