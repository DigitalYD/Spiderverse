import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from pod import Pod
from hex_body import Body
from gaits import new_Gait
from config import COXA_ORIGIN_INDEX, FEMUR_ORIGIN_INDEX, TIBIA_ORIGIN_INDEX, EFFECTOR_ORIGIN_INDEX
from coord import Coordinate
from inversekinematics import solve_effector_IK

# Setup hexapod
tripod_gait = new_Gait(0, 1.0)
body = Body(6, Gait=tripod_gait)
body = body.load("hexapod_config.json")
hexapod = Pod(body)

NUM_LEGS = hexapod.body_def.num_legs

# Get total number of Bezier points (assume all legs have same curve length)
NUM_POINTS = hexapod.Legs[0].bezier_curve.num_points  # Adjust if points are stored differently

# Initialize plot
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Hexapod Leg Animation (Matplotlib)")
ax.set_xlabel("X (Left/Right)")
ax.set_ylabel("Y (Forward/Back)")
ax.set_zlabel("Z (Up)")

# Set axis limits based on your hexapod’s range
ax.set_xlim(-500, 500)
ax.set_ylim(-700, 700)
ax.set_zlim(-150, 50)

# Initialize lines and points for each leg
colors = ['r', 'g', 'b', 'c', 'm', 'y']
leg_lines = [[ax.plot([], [], [], f'{colors[i]}-', linewidth=2)[0] for _ in range(3)] for i in range(NUM_LEGS)]  # Coxa-Femur, Femur-Tibia, Tibia-Effector
leg_points = [ax.scatter([], [], [], c=colors[i], s=50, label=f'Leg {i}') for i in range(NUM_LEGS)]
body_line = ax.plot([], [], [], 'k--', label="Hex Body")[0]

# Add legend
ax.legend()

def init():
    """Initialize the animation with empty lines and points."""
    for leg in leg_lines:
        for line in leg:
            line.set_data_3d([], [], [])
    for points in leg_points:
        points._offsets3d = ([], [], [])
    body_line.set_data_3d([], [], [])
    return [line for leg in leg_lines for line in leg] + leg_points + [body_line]

def update(frame):
    """Update the plot for each frame based on Bezier curve index."""
    index = frame % NUM_POINTS  # Loop through Bezier points

    # Update each leg’s position
    for i, leg in enumerate(hexapod.Legs):
        # Get Bezier curve point
        val = leg.bezier_curve.get_point(index)
        effector_target = Coordinate(val[0], val[1], val[2])  # Assuming new_Coordinate = Coordinate

        # Calculate angles and update kinematics
        angles = solve_effector_IK(leg, effector_target)
        hexapod.Legs[i].recalculate_forward_kinematics(angles)

        # Extract joint positions
        joints = leg.Joints
        x = [j.X for j in joints]
        y = [j.Y for j in joints]
        z = [j.Z for j in joints]

        # Update lines (Coxa-Femur, Femur-Tibia, Tibia-Effector)
        leg_lines[i][0].set_data_3d(x[0:2], y[0:2], z[0:2])
        leg_lines[i][1].set_data_3d(x[1:3], y[1:3], z[1:3])
        leg_lines[i][2].set_data_3d(x[2:4], y[2:4], z[2:4])

        # Update points
        leg_points[i]._offsets3d = (x, y, z)

    # Update hex body frame (coxa connections)
    coxa_x = [hexapod.Legs[i].Joints[COXA_ORIGIN_INDEX].X for i in range(NUM_LEGS)]
    coxa_y = [hexapod.Legs[i].Joints[COXA_ORIGIN_INDEX].Y for i in range(NUM_LEGS)]
    coxa_z = [hexapod.Legs[i].Joints[COXA_ORIGIN_INDEX].Z for i in range(NUM_LEGS)]
    coxa_x.append(coxa_x[0])
    coxa_y.append(coxa_y[0])
    coxa_z.append(coxa_z[0])
    body_line.set_data_3d(coxa_x, coxa_y, coxa_z)

    return [line for leg in leg_lines for line in leg] + leg_points + [body_line]

# Create animation
ani = animation.FuncAnimation(
    fig,
    update,
    frames=NUM_POINTS,
    init_func=init,
    blit=True,
    interval=100  # Milliseconds between frames (adjust for speed)
)

plt.show()