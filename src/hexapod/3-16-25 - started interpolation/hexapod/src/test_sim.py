from vpython import *
from pod import Pod
from hex_body import Body
from inversekinematics import solve_effector_IK
from gaits import new_Gait
import numpy as np 
from coord import new_Coordinate

# Create the hexapod instance
tripod_gait = new_Gait(0, 1.0)  # Set gait and speed through Bezier curve
body = Body(6, Gait=tripod_gait)  # Create new hexapod body 
body = body.load("hexapod_config.json")
hexapod = Pod(body)

NUM_LEGS = hexapod.body_def.num_legs
STEPS = hexapod.Legs[0].intermediate_angles.steps  # Number of Bezier interpolation steps

# VPython scene setup
scene = canvas(title="Hexapod Simulation", width=800, height=600, center=vector(0, 0, 0), background=color.white)

# Ground plane
ground = box(pos=vector(0, -10, 0), length=300, height=1, width=300, color=color.green)

# Pentagon dimensions
PENTAGON_RADIUS = 50  # Radius of the pentagon body
PENTAGON_HEIGHT = 5  # Thickness of the hexapod body
ANGLE_OFFSET = 72  # Angle between pentagon points (360°/5)

# Calculate pentagon vertices in XZ plane for proper upright orientation
pentagon_vertices = []
for i in range(5):
    angle_rad = radians(i * ANGLE_OFFSET)
    x = PENTAGON_RADIUS * cos(angle_rad)
    z = PENTAGON_RADIUS * sin(angle_rad)
    pentagon_vertices.append(vector(x, 0, z))  # Ensure the body is in the XZ plane

# Create the pentagon body using spheres and cylinders
pentagon_joints = [sphere(pos=vertex, radius=3, color=color.blue) for vertex in pentagon_vertices]
pentagon_edges = [
    cylinder(pos=pentagon_vertices[i], axis=pentagon_vertices[(i + 1) % 5] - pentagon_vertices[i], radius=1, color=color.blue)
    for i in range(5)
]

# Define joints as spheres and links as cylinders
legs_visuals = []
for i in range(NUM_LEGS):
    base_pos = pentagon_vertices[i % 5] + vector(0, -5, 0)  # Attach legs properly to body
    joints = [
        sphere(pos=base_pos, radius=3, color=color.red),    # Coxa joint
        sphere(pos=base_pos, radius=3, color=color.blue),   # Femur joint
        sphere(pos=base_pos, radius=3, color=color.green),  # Tibia joint
    ]
    
    links = [
        cylinder(pos=base_pos, axis=vector(0, -10, 0), radius=1, color=color.gray(0.5)),  # Coxa-Femur
        cylinder(pos=base_pos, axis=vector(0, -10, 0), radius=1, color=color.gray(0.7)),  # Femur-Tibia
    ]
    
    legs_visuals.append((joints, links))

# Initialize leg positions correctly
for leg in hexapod.Legs:
    leg.Joints[0].angle = 0  # Coxa angle
    leg.Joints[1].angle = radians(45)  # Femur slightly bent
    leg.Joints[2].angle = radians(-45)  # Tibia slightly bent downward

# Real-time animation loop
while True:
    for frame in range(STEPS):
        rate(30)  # Adjust to control real-time speed (30 FPS)

        for i, leg in enumerate(hexapod.Legs):
            # Get the Bezier curve effector target at this step
            val = leg.bezier_curve.get_point(frame)
            effector_target = new_Coordinate(val[0], val[1], val[2])

            # Solve IK to compute joint angles
            angles = solve_effector_IK(leg, effector_target)

            # Compute Forward Kinematics (update joint positions)
            hexapod.Legs[i].recalculate_forward_kinematics(angles)

            # Get updated joint positions
            coxa_pos = vector(leg.Joints[0].X, leg.Joints[0].Y, leg.Joints[0].Z)
            femur_pos = vector(leg.Joints[1].X, leg.Joints[1].Y, leg.Joints[1].Z)
            tibia_pos = vector(leg.Joints[2].X, leg.Joints[2].Y, leg.Joints[2].Z)

            # Update sphere positions (joint locations)
            legs_visuals[i][0][0].pos = coxa_pos  # Coxa joint
            legs_visuals[i][0][1].pos = femur_pos  # Femur joint
            legs_visuals[i][0][2].pos = tibia_pos  # Tibia joint

            # Update cylinders (links) between joints
            legs_visuals[i][1][0].pos = coxa_pos
            legs_visuals[i][1][0].axis = femur_pos - coxa_pos
            
            legs_visuals[i][1][1].pos = femur_pos
            legs_visuals[i][1][1].axis = tibia_pos - femur_pos
