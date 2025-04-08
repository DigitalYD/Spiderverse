from vpython import *
from src.pod import Pod
from src.hex_body import Body
from src.inversekinematics import solve_effector_IK
from src.gaits import new_Gait
import numpy as np 
from src.coord import new_Coordinate
from src.config import COXA_ORIGIN_INDEX, FEMUR_ORIGIN_INDEX, TIBIA_ORIGIN_INDEX, EFFECTOR_ORIGIN_INDEX
from src.gaits import GaitType
from src.bezier2d import BezierCurve



# Create the hexapod instance

# gaits_to_test = [
#     (GaitType.TRIPOD, 1.0, [0, 0, 0, 0, 0, 0]),  # No phase shift
#     (GaitType.WAVE, 0.19, [0, 1, 2, 3, 4, 5]),   # Sequential phase shift
####    (GaitType.TETRAPOD, 0.4, [0, 0, 0, 1, 1, 1]),# Half shifted Not using, requires offset calculation
#     (GaitType.RIPPLE, 0.4, [0, 0, 0, 0, 0, 0])   # No phase shift
# ]
tripod_gait, tripod_phase = GaitType.TRIPOD, [0, 1, 0, 1, 0, 1]
wave_gait, wave_phase = GaitType.WAVE, [0, 1, 2, 3, 4, 5]
ripple_gait, ripple_phase =  GaitType.RIPPLE,  [0, 1, 2, 3, 0, 1]
# tetrapod_gait, tetrapod_phase = GaitType.TETRAPOD,  [0, 0, 0, 1, 1, 1]

body = Body(6, Gait=tripod_gait)  
body = body.load("src/hexapod_config.json") # Overwrites gait set
hexapod = Pod(body)

# print(body.Gait)
# print(hexapod.gait)

    
NUM_LEGS = hexapod.body_def.num_legs
STEPS = hexapod.Legs[0].intermediate_angles.steps  

# VPython scene setup (Z-up mapped to VPython’s Y-up)
scene = canvas(title="Hexapod Simulation (Z-Flipped)", width=800, height=600, center=vector(0, 0, 0), background=color.white)
# Set the up direction to the z-axis (unchanged)
scene.up = vector(0, 1, 0)
scene.forward = -scene.forward
#scene.camera.rotate(angle=90, axis=y)


# Ground plane: Flip Z=0 to Z=0 but adjust position later; VPython Y maps to your Z
ground = box(pos=vector(0, -100, 0), size=vector(300, 1, 300), color=color.green)  # Ground at Y=-100 (your Z=-100)

# Hexagon dimensions
HEXAGON_RADIUS = 50  
HEXAGON_HEIGHT = 5  
ANGLE_OFFSET = 60  

# Store joint positions (convert flipped Z-up to VPython Y-up)
coxa_positions = []
femur_positions = []
tibia_positions = []
end_effector_positions = []

# Compute initial joint positions with Z flipped
for i in range(NUM_LEGS):
    # Extract coordinates from your Z-up system and flip Z
    coxa_x = hexapod.Legs[i].Joints[COXA_ORIGIN_INDEX].X
    coxa_y = hexapod.Legs[i].Joints[COXA_ORIGIN_INDEX].Y
    coxa_z = -hexapod.Legs[i].Joints[COXA_ORIGIN_INDEX].Z  # Flip Z
    
    femur_x = hexapod.Legs[i].Joints[FEMUR_ORIGIN_INDEX].X
    femur_y = hexapod.Legs[i].Joints[FEMUR_ORIGIN_INDEX].Y
    femur_z = -hexapod.Legs[i].Joints[FEMUR_ORIGIN_INDEX].Z  # Flip Z
    
    tibia_x = hexapod.Legs[i].Joints[TIBIA_ORIGIN_INDEX].X
    tibia_y = hexapod.Legs[i].Joints[TIBIA_ORIGIN_INDEX].Y
    tibia_z = -hexapod.Legs[i].Joints[TIBIA_ORIGIN_INDEX].Z  # Flip Z
    
    end_x = hexapod.Legs[i].Joints[EFFECTOR_ORIGIN_INDEX].X
    end_y = hexapod.Legs[i].Joints[EFFECTOR_ORIGIN_INDEX].Y
    end_z = -hexapod.Legs[i].Joints[EFFECTOR_ORIGIN_INDEX].Z  # Flip Z
    
    # Map flipped Z-up (X, Y, -Z) to VPython Y-up (X, -Z, Y)
    coxa_positions.append(vector(coxa_x, coxa_z, coxa_y))
    femur_positions.append(vector(femur_x, femur_z, femur_y))
    tibia_positions.append(vector(tibia_x, tibia_z, tibia_y))
    end_effector_positions.append(vector(end_x, end_z, end_y))

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

# Real-time animation loop with Z flipped
index = 0
current_step = 0
phase_progress = 0.0  # Goes from 0 → 1 in each gait step
gait_step_duration = 20  # Number of frames per gait phase (adjust as needed)

#hexapod.start() # initialize walking

while True:
    for frame in range(STEPS):
        rate(30)  
        ## Code to update bezier curve based on controller goes here
        foot_targets = hexapod.update()  # Get new foot targets from gait manager
        for i, leg in enumerate(hexapod.Legs):
            # Get IK values from the targets
            if foot_targets != None:
                #print(hexapod.currentMode)
                angles = solve_effector_IK(leg, foot_targets[i])
                # Calculate new positions
                leg.recalculate_forward_kinematics(angles)
            ###---------- 
            # Move leg positions based on new theta angles here
            ###---------- 
            # <- Here
            # -----------
            if index == 120:
                hexapod.start()
                
            # if index == 350:
            #     hexapod.stop()

            # if index == 400:
            #     hexapod.setMode = "resetting"
            
            # if index == 550:
            #     hexapod.start()
            
            # Map Z-up with flipped Z (X, Y, -Z) to VPython Y-up (X, -Z, Y)
            coxa_pos = vector(
                leg.Joints[COXA_ORIGIN_INDEX].X,
                -leg.Joints[COXA_ORIGIN_INDEX].Z,  # Flip Z
                leg.Joints[COXA_ORIGIN_INDEX].Y
            )
            femur_pos = vector(
                leg.Joints[FEMUR_ORIGIN_INDEX].X,
                -leg.Joints[FEMUR_ORIGIN_INDEX].Z,  # Flip Z
                leg.Joints[FEMUR_ORIGIN_INDEX].Y
            )
            tibia_pos = vector(
                leg.Joints[TIBIA_ORIGIN_INDEX].X,
                -leg.Joints[TIBIA_ORIGIN_INDEX].Z,  # Flip Z
                leg.Joints[TIBIA_ORIGIN_INDEX].Y
            )
            end_effector_pos = vector(
                leg.Joints[EFFECTOR_ORIGIN_INDEX].X,
                -leg.Joints[EFFECTOR_ORIGIN_INDEX].Z,  # Flip Z
                leg.Joints[EFFECTOR_ORIGIN_INDEX].Y
            )
            
            # Update visual positions
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