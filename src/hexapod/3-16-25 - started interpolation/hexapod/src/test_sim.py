from vpython import *
from pod import Pod
from hex_body import Body
from inversekinematics import solve_effector_IK
from gaits import new_Gait
import numpy as np 
from coord import new_Coordinate
from config import COXA_ORIGIN_INDEX, FEMUR_ORIGIN_INDEX, TIBIA_ORIGIN_INDEX, EFFECTOR_ORIGIN_INDEX

# Set the up direction to the z-axis
scene.up = vector(0, 0, 1)

# Create the hexapod instance
tripod_gait = new_Gait(0, 1.0)  
body = Body(6, Gait=tripod_gait)  
body = body.load("hexapod_config.json")
hexapod = Pod(body)

NUM_LEGS = hexapod.body_def.num_legs
STEPS = hexapod.Legs[0].intermediate_angles.steps  

# VPython scene setup (Z-up mapped to VPython’s Y-up)
scene = canvas(title="Hexapod Simulation", width=800, height=600, center=vector(0, 0, 0), background=color.white)

# Ground plane: Z-up means Z=0 is the ground, mapped to VPython Y=0
ground = box(pos=vector(0, 0, 0), size=vector(300, 1, 300), color=color.green)
ground.pos = vector(0, -50, 0)  # Move ground down along VPython Y-axis (maps to your Z)

# Hexagon dimensions
HEXAGON_RADIUS = 50  
HEXAGON_HEIGHT = 5  
ANGLE_OFFSET = 60  

# Store joint positions (convert Z-up to VPython Y-up)
coxa_positions = []
femur_positions = []
tibia_positions = []
end_effector_positions = []

# Compute joint positions using homogeneous transformation
for i in range(NUM_LEGS):
    # Extract coordinates from your Z-up system
    coxa_x, coxa_y, coxa_z = hexapod.Legs[i].Joints[COXA_ORIGIN_INDEX].X, hexapod.Legs[i].Joints[COXA_ORIGIN_INDEX].Y, hexapod.Legs[i].Joints[COXA_ORIGIN_INDEX].Z 
    femur_x, femur_y, femur_z = hexapod.Legs[i].Joints[FEMUR_ORIGIN_INDEX].X, hexapod.Legs[i].Joints[FEMUR_ORIGIN_INDEX].Y, hexapod.Legs[i].Joints[FEMUR_ORIGIN_INDEX].Z
    tibia_x, tibia_y, tibia_z = hexapod.Legs[i].Joints[TIBIA_ORIGIN_INDEX].X, hexapod.Legs[i].Joints[TIBIA_ORIGIN_INDEX].Y, hexapod.Legs[i].Joints[TIBIA_ORIGIN_INDEX].Z
    end_x, end_y, end_z = hexapod.Legs[i].Joints[EFFECTOR_ORIGIN_INDEX].X, hexapod.Legs[i].Joints[EFFECTOR_ORIGIN_INDEX].Y, hexapod.Legs[i].Joints[EFFECTOR_ORIGIN_INDEX].Z
    
    # Map Z-up (X, Y, Z) to VPython Y-up (X, Z, Y)
    coxa_positions.append(vector(coxa_x, coxa_z, coxa_y))
    femur_positions.append(vector(femur_x, femur_z, femur_y))
    tibia_positions.append(vector(tibia_x, tibia_z, tibia_y))
    end_effector_positions.append(vector(end_x, end_z, end_y))

print("--- POST CREATION ---")
print(f'Coxa: {coxa_positions}')
print(f'Femur: {femur_positions}')
print(f'Tibia: {tibia_positions}')
print(f'End Effector: {end_effector_positions}')

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
            if i == 0:  # Example: animate leg 1
                val = leg.bezier_curve.get_point(index)
                effector_target = new_Coordinate(val[0], val[1], val[2])
                print(f"index:{index}, New Coord: {val}")
                angles = solve_effector_IK(leg, effector_target)

                hexapod.Legs[i].recalculate_forward_kinematics(angles)
                print(f"Leg Joints & positions 4x4 matrix: {leg.Joints}")
                
                print('---------------------------')
                # Map Z-up to VPython Y-up: (X, Y, Z) -> (X, Z, Y)
                coxa_pos = vector(leg.Joints[COXA_ORIGIN_INDEX].X, leg.Joints[COXA_ORIGIN_INDEX].Z, leg.Joints[COXA_ORIGIN_INDEX].Y)
                femur_pos = vector(leg.Joints[FEMUR_ORIGIN_INDEX].X, leg.Joints[FEMUR_ORIGIN_INDEX].Z, leg.Joints[FEMUR_ORIGIN_INDEX].Y)
                tibia_pos = vector(leg.Joints[TIBIA_ORIGIN_INDEX].X, leg.Joints[TIBIA_ORIGIN_INDEX].Z, leg.Joints[TIBIA_ORIGIN_INDEX].Y)
                end_effector_pos = vector(leg.Joints[EFFECTOR_ORIGIN_INDEX].X, leg.Joints[EFFECTOR_ORIGIN_INDEX].Z, leg.Joints[EFFECTOR_ORIGIN_INDEX].Y)  
                
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
                if index >= 100:
                    index = 0