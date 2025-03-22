import numpy as np
import matplotlib.pyplot as plt

# Translation function (from prior response)
def translate_point_along_leg_direction(point: np.ndarray, theta_offset: float, theta: float, distance: float) -> np.ndarray:
    """Translate a point along a leg's direction, adjusted by theta."""
    total_theta = np.radians(theta_offset + theta)
    direction = np.array([np.cos(total_theta), np.sin(total_theta), 0])
    translation = direction * distance
    return point + translation

# Bézier curve function
def bezier_curve(t: np.ndarray, p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> np.ndarray:
    """Compute points along a cubic Bézier curve."""
    t = t.reshape(-1, 1)  # Ensure column vector
    return (1-t)**3 * p0 + 3*(1-t)**2*t * p1 + 3*(1-t)*t**2 * p2 + t**3 * p3

# Hexapod leg data (corrected diagonals)
leg_data = {
    "RR": {"coxa_coord": np.array([60.5, -89, 0]), "theta_offset": 147.172},
    "LR": {"coxa_coord": np.array([-60.5, -89, 0]), "theta_offset": 214.309},
    "RF": {"coxa_coord": np.array([60.5, 89, 0]), "theta_offset": 34.0685},
    "LF": {"coxa_coord": np.array([-60.5, 89, 0]), "theta_offset": 325.931},
    "LM": {"coxa_coord": np.array([-97, 0, 0]), "theta_offset": 270},
    "RM": {"coxa_coord": np.array([97, 0, 0]), "theta_offset": 90}
}

# Segment lengths (for reference)
segment_lengths = {"Coxa": 45, "Femur": 110, "Tibia": 193}

# Neutral effector offset (assumed from Leg class)
neutral_effector_offset = np.array([0, 0, -160])

# Define Bézier control points relative to neutral effector position
def get_bezier_control_points(start_pos: np.ndarray) -> dict:
    """Define control points for a Bézier curve starting at the effector position."""
    return {
        "start": start_pos,
        "lift": start_pos + np.array([20, -50, -40]),   # Lift up and slightly forward
        "peak": start_pos + np.array([40, -75, -100]),  # Peak of swing
        "touchdown": start_pos + np.array([80, -100, 0])  # Touchdown forward
    }

# Parameters
theta = 0.0  # No additional rotation
distance = 20.0  # Translation distance (forward)

# Compute initial effector positions and Bézier curves
effector_positions = {}
original_curves = {}
translated_curves = {}
t = np.linspace(0, 1, 50)  # 50 points along the curve

for leg_name, data in leg_data.items():
    coxa_pos = data["coxa_coord"]
    theta_offset = data["theta_offset"]
    effector_pos = coxa_pos + neutral_effector_offset
    effector_positions[leg_name] = effector_pos
    
    # Original Bézier curve
    control_points = get_bezier_control_points(effector_pos)
    original_curves[leg_name] = bezier_curve(t, control_points["start"], control_points["lift"],
                                             control_points["peak"], control_points["touchdown"])
    
    # Translated Bézier curve
    translated_control_points = {
        key: translate_point_along_leg_direction(pt, theta_offset, theta, distance)
        for key, pt in control_points.items()
    }
    translated_curves[leg_name] = bezier_curve(t, translated_control_points["start"],
                                               translated_control_points["lift"],
                                               translated_control_points["peak"],
                                               translated_control_points["touchdown"])

# Set up the figure and axis
fig, ax = plt.subplots()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Hexapod Bézier Curve Translation (Forward)')
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlim(-150, 150)
ax.set_ylim(-200, 120)

# Plot origin (hexapod center)
ax.plot(0, 0, 'ko', markersize=10, label='Center')

# Plot Coxa and curves
colors = {'RR': 'b', 'LR': 'm', 'RF': 'g', 'LF': 'r', 'LM': 'c', 'RM': 'y'}
for leg_name, data in leg_data.items():
    coxa_pos = data["coxa_coord"]
    orig_curve = original_curves[leg_name]
    trans_curve = translated_curves[leg_name]
    
    # Coxa position
    ax.plot(coxa_pos[0], coxa_pos[1], 'o', color=colors[leg_name], label=leg_name)
    # Original curve
    ax.plot(orig_curve[:, 0], orig_curve[:, 1], '-', color=colors[leg_name], alpha=0.5)
    # Translated curve
    ax.plot(trans_curve[:, 0], trans_curve[:, 1], '--', color=colors[leg_name], label=f'{leg_name} Translated')

ax.legend()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_zlim(-170, 0)
for leg_name in leg_data:
    ax.plot(orig_curve[:, 0], orig_curve[:, 1], orig_curve[:, 2], '-', color=colors[leg_name], alpha=0.5)
    ax.plot(trans_curve[:, 0], trans_curve[:, 1], trans_curve[:, 2], '--', color=colors[leg_name])

from matplotlib.animation import FuncAnimation

plots = {}
for leg_name in leg_data:
    plots[leg_name + "_orig"] = ax.plot([], [], '-', color=colors[leg_name], alpha=0.5)[0]
    plots[leg_name + "_trans"] = ax.plot([], [], '--', color=colors[leg_name])[0]

def update(frame):
    t = frame / 49
    dist = distance * np.sin(2 * np.pi * t)
    for leg_name in leg_data:
        orig_curve = original_curves[leg_name]
        trans_curve = bezier_curve(t, *[translate_point_along_leg_direction(pt, leg_data[leg_name]["theta_offset"], theta, dist)
                                        for pt in get_bezier_control_points(effector_positions[leg_name]).values()])
        plots[leg_name + "_orig"].set_data(orig_curve[:, 0], orig_curve[:, 1])
        plots[leg_name + "_trans"].set_data(trans_curve[:, 0], trans_curve[:, 1])
    return plots.values()

ani = FuncAnimation(fig, update, frames=50, interval=50, blit=True)