from vpython import *
from pod import Pod
from hex_body import Body
from inversekinematics import solve_effector_IK
from gaits import new_Gait
import numpy as np 
from coord import new_Coordinate
from config import COXA_ORIGIN_INDEX, FEMUR_ORIGIN_INDEX, TIBIA_ORIGIN_INDEX, EFFECTOR_ORIGIN_INDEX

# Create the hexapod instance
tripod_gait = new_Gait(0, 1.0)  
body = Body(6, Gait=tripod_gait)  
body = body.load("hexapod_config.json")
hexapod = Pod(body)


NUM_LEGS = hexapod.body_def.num_legs
STEPS = hexapod.Legs[0].intermediate_angles.steps  

for i in range(NUM_LEGS):
    print(hexapod.body_def.coxa_coords[i])
    
print("-------- JOINTS ---------")
for i in range(NUM_LEGS):
    print(hexapod.Legs[i].Name)
    print(hexapod.Legs[i].Joints)
    
# VPython scene setup
scene = canvas(title="Hexapod Simulation", width=800, height=600, center=vector(0, 0, 0), background=color.white)

# Rotate the ground to match the new orientation
ground = box(pos=vector(0, 0, 0), size=vector(300, 300, 1), color=color.green)
ground.rotate(angle=radians(90), axis=vector(0, 0, 1), origin=vector(0, 0, 0))
ground.pos = vector(0, 0, -50)  # Move it back along Z-axis after rotation

# Hexagon dimensions
HEXAGON_RADIUS = 50  
HEXAGON_HEIGHT = 5  
ANGLE_OFFSET = 60  

# Projection matrix (identity matrix if no custom projection)

# Store joint positions
coxa_positions = []
femur_positions = []
tibia_positions = []
end_effector_positions = []

# Compute joint positions using homogeneous transformation
for i in range(NUM_LEGS):
    
    # Compute homogeneous transformation matrices for each segment
    coxa_x, coxa_y, coxa_z = hexapod.Legs[i].Joints[COXA_ORIGIN_INDEX].X, hexapod.Legs[i].Joints[COXA_ORIGIN_INDEX].Y, hexapod.Legs[i].Joints[COXA_ORIGIN_INDEX].Z 
    femur_x, femur_y, femur_z = hexapod.Legs[i].Joints[FEMUR_ORIGIN_INDEX].X, hexapod.Legs[i].Joints[FEMUR_ORIGIN_INDEX].Y, hexapod.Legs[i].Joints[FEMUR_ORIGIN_INDEX].Z
    tibia_x, tibia_y, tibia_z = hexapod.Legs[i].Joints[TIBIA_ORIGIN_INDEX].X, hexapod.Legs[i].Joints[TIBIA_ORIGIN_INDEX].Y, hexapod.Legs[i].Joints[TIBIA_ORIGIN_INDEX].Z
    end_x, end_y, end_z =  hexapod.Legs[i].Joints[EFFECTOR_ORIGIN_INDEX].X, hexapod.Legs[i].Joints[EFFECTOR_ORIGIN_INDEX].Y, hexapod.Legs[i].Joints[EFFECTOR_ORIGIN_INDEX].Z
    
    # Extract (X, Y, Z) from the fourth column of H
    coxa_positions.append(vector(coxa_x, coxa_y, coxa_z))
    femur_positions.append(vector(femur_x, femur_y, femur_z))
    tibia_positions.append(vector(tibia_x, tibia_y, tibia_z))
    end_effector_positions.append(vector(end_x, end_y, end_z))

print("--- POST CREATION ---")
print(coxa_positions)
print(femur_positions)
print(tibia_positions)
print(end_effector_positions)


# Create the hexagon body using spheres and cylinders
hexagon_joints = [sphere(pos=coxa_positions[i], radius=3, color=color.blue) for i in range(NUM_LEGS)]
hexagon_edges = [
    cylinder(pos=coxa_positions[i], axis=coxa_positions[(i + 1) % NUM_LEGS] - coxa_positions[i], radius=1, color=color.blue)
    for i in range(NUM_LEGS)
]

# Define joints as spheres and links as cylinders
legs_visuals = []
for i in range(NUM_LEGS):
    joints = [
        sphere(pos=coxa_positions[i], radius=3, color=color.red),    
        sphere(pos=femur_positions[i], radius=3, color=color.blue),   
        sphere(pos=tibia_positions[i], radius=3, color=color.green),  
        sphere(pos=end_effector_positions[i], radius=3, color=color.yellow),  
    ]
    
    links = [
        cylinder(pos=coxa_positions[i], axis=femur_positions[i] - coxa_positions[i], radius=1, color=color.gray(0.5)),  
        cylinder(pos=femur_positions[i], axis=tibia_positions[i] - femur_positions[i], radius=1, color=color.gray(0.7)),  
        cylinder(pos=tibia_positions[i], axis=end_effector_positions[i] - tibia_positions[i], radius=1, color=color.gray(0.8)),  
    ]
    
    legs_visuals.append((joints, links))
    
# Real-time animation loop
index = 0
while True:
    for frame in range(STEPS):
        rate(30)  
        
        for i, leg in enumerate(hexapod.Legs):
            
            val = leg.bezier_curve.get_point(index)
            effector_target = new_Coordinate(val[0], val[1], val[2])

            angles = solve_effector_IK(leg, effector_target)

            hexapod.Legs[i].recalculate_forward_kinematics(angles)

            coxa_pos = vector(leg.Joints[COXA_ORIGIN_INDEX].X, leg.Joints[COXA_ORIGIN_INDEX].Y, leg.Joints[COXA_ORIGIN_INDEX].Z)
            femur_pos = vector(leg.Joints[FEMUR_ORIGIN_INDEX].X, leg.Joints[FEMUR_ORIGIN_INDEX].Y, leg.Joints[FEMUR_ORIGIN_INDEX].Z)
            tibia_pos = vector(leg.Joints[TIBIA_ORIGIN_INDEX].X, leg.Joints[TIBIA_ORIGIN_INDEX].Y, leg.Joints[TIBIA_ORIGIN_INDEX].Z)
            end_effector_pos = vector(leg.Joints[EFFECTOR_ORIGIN_INDEX].X, leg.Joints[EFFECTOR_ORIGIN_INDEX].Y, leg.Joints[EFFECTOR_ORIGIN_INDEX].Z)  

            legs_visuals[i][0][0].pos = coxa_pos  
            legs_visuals[i][0][1].pos = femur_pos  
            legs_visuals[i][0][2].pos = tibia_pos  
            legs_visuals[i][0][3].pos = end_effector_pos  

            legs_visuals[i][1][0].pos = coxa_pos
            legs_visuals[i][1][0].axis = femur_pos - coxa_pos
            
            legs_visuals[i][1][1].pos = femur_pos
            legs_visuals[i][1][1].axis = tibia_pos - femur_pos

            legs_visuals[i][1][2].pos = tibia_pos
            legs_visuals[i][1][2].axis = end_effector_pos - tibia_pos
            
        index += 1
        if index >= 50:
            index = 0