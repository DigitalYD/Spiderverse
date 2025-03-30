import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

# Segment lengths (for reference, not directly used here)
segment_lengths = {"Coxa": 45, "Femur": 110, "Tibia": 193}

# Neutral effector position relative to Coxa (assumed from your Leg class)
neutral_effector_offset = np.array([0, 0, -160])  # Z = -160 below Coxa

# Compute initial effector positions (Coxa + neutral offset)
effector_positions = {}
for leg_name, data in leg_data.items():
    effector_positions[leg_name] = data["coxa_coord"] + neutral_effector_offset

# Parameters
theta = 0.0  # No additional rotation
max_distance = 20.0  # Maximum translation distance
steps = 50  # Animation steps

# Set up the figure and axis
fig, ax = plt.subplots()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Hexapod Effector Translation (Forward/Backward)')
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlim(-120, 120)
ax.set_ylim(-120, 120)

# Plot origin (hexapod center)
ax.plot(0, 0, 'ko', markersize=10, label='Center')

# Initialize plots for each leg
plots = {}
for leg_name in leg_data:
    coxa_pos = leg_data[leg_name]["coxa_coord"]
    effector_pos = effector_positions[leg_name]
    # Coxa position (static)
    plots[leg_name + "_coxa"] = ax.plot(coxa_pos[0], coxa_pos[1], 'o', label=leg_name)[0]
    # Effector original position (static)
    plots[leg_name + "_effector"] = ax.plot(effector_pos[0], effector_pos[1], 'x', color='gray')[0]
    # Forward translation (animated)
    plots[leg_name + "_forward"] = ax.plot(effector_pos[0], effector_pos[1], 'g+', markersize=10)[0]
    # Backward translation (animated)
    plots[leg_name + "_backward"] = ax.plot(effector_pos[0], effector_pos[1], 'r+', markersize=10)[0]

ax.legend()

# Animation function
def update(frame):
    t = frame / (steps - 1)  # Normalize to 0-1
    distance = max_distance * np.sin(2 * np.pi * t)  # Oscillate -20 to 20
    
    for leg_name, data in leg_data.items():
        effector_pos = effector_positions[leg_name]
        theta_offset = data["theta_offset"]
        
        # Compute translated effector positions
        pos_forward = translate_point_along_leg_direction(effector_pos, theta_offset, theta-45, distance)
        pos_backward = translate_point_along_leg_direction(effector_pos, theta_offset, theta+45, -distance)
        
        # Update forward plot
        plots[leg_name + "_forward"].set_data([pos_forward[0]], [pos_forward[1]])
        # Update backward plot
        plots[leg_name + "_backward"].set_data([pos_backward[0]], [pos_backward[1]])
    
    return [plots[name] for name in plots if "_coxa" not in name and "_effector" not in name]

# Create animation
ani = FuncAnimation(fig, update, frames=steps, interval=50, blit=True)

plt.show()