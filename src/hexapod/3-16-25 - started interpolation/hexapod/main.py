'''
    This application will start all other needed processes for this project.
    - Hexapod
    - Zigbee Controller input
    - i2c to hexapod
    - camera input
    - etc.

'''
from src.pod import Pod
from src.hex_body import Body
from src.inversekinematics import solve_effector_IK
from src.gaits import new_Gait, GaitType
import numpy as np 
from src.coord import new_Coordinate
from src.config import COXA_ORIGIN_INDEX, FEMUR_ORIGIN_INDEX, TIBIA_ORIGIN_INDEX, EFFECTOR_ORIGIN_INDEX


def main_loop(hexapod):
    '''
        Implement movement and loop here
    '''
    # Test gaits with phase shifts
    gaits_to_test = [
        (GaitType.TRIPOD, 1.0, [0, 0, 0, 0, 0, 0]),  # No phase shift
        (GaitType.WAVE, 0.19, [0, 1, 2, 3, 4, 5]),   # Sequential phase shift
        (GaitType.TETRAPOD, 0.4, [0, 0, 0, 1, 1, 1]),# Half shifted
        (GaitType.RIPPLE, 0.4, [0, 0, 0, 0, 0, 0])   # No phase shift
    ]

    for gait_type, speed_factor, phase_shifts in gaits_to_test:
        hexapod.set_gait(gait_type, speed_factor, phase_shifts)
        hexapod.perform_gait(num_cycles=1)
    
    #  aquire thetas and move hexapod legs 
    while True:
        hexapod.move_leg_bezier_test()
    # In a loop starting here
    # -----
    # receive new message from controller
    # Decode controller
        #bodypos xy = 0
        # bodyrot x,y,z = 0
        # walk_length.x = 0
        # walk_length.y = 0
        # walk_length.roty = 0
        # calculate gait/body balance
    
    # Resets these each iteration to 0 | maybe track it within hexapod
    # hexPos = coord3D()
    # hexRot = coord3D()

    # offsetx, offsety, offset angle

    # Call hexapod inverse kinematics
    # Call hexapod update, or move legs to move
    
    # Gait sequence()
    # body balance
    
    # set hexapod position and rotation to be updated
    # hexapod.update()
    # hexapod.inverse_kinematics()
    # hexapod.move_leg?
    
    '''
        calculate every leg's angles first, then move the legs individually
    '''



# def compute_mount_positions(radius=97):
#     """ Computes the mount positions based on the given angles. """
#     mount_positions = {}
#     mount_angle_degrees = {
#         "LR": 214.309,
#         "LM": 270,
#         "LF": 325.931,
#         "RF": 34.0685,
#         "RM": 90,
#         "RR": 146.172,
#     }
    
#     for leg, angle in mount_angle_degrees.items():
#         angle_rad = np.deg2rad(angle)  # Convert to radians
#         x = radius * np.cos(angle_rad)  # Compute X position
#         y = radius * np.sin(angle_rad)  # Compute Y position
#         mount_positions[leg] = (x, y)
    
#     return mount_positions

if __name__ == "__main__":
    # setup controller stuff here

    # Create the hexapod instance
    tripod_gait = new_Gait(0, 1.0)  
    body = Body(6, Gait=tripod_gait)  
    body = body.load("src/hexapod_config.json")
    hexapod = Pod(body)
    main_loop(hexapod)