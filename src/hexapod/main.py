'''
    This application will start all other needed processes for this project.
    - Hexapod
    - Zigbee Controller input
    - i2c to hexapod
    - camera input
    - etc.

'''
from src.anim_plot import *
from src.hexapod import Hexapod
from src.leg_configs import leg_configs
from src.hexapod_configs import hexapod_configs
from src.coord import *
#from common.new_bezier import bezier_curve
import numpy as np
import time

walk_length = coord3D() 

def main_loop(hexapod, hexapod_animation):
    '''
        Implement movement and loop here
    '''

    #  aquire thetas and move hexapod legs 
    while True:
        # initialize hexapod
        hexapod.inverse_kinematics()
        rads = hexapod.get_rads()
        #print(rads)
        hexapod_animation.update(rads)

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



def compute_mount_positions(radius=97):
    """ Computes the mount positions based on the given angles. """
    mount_positions = {}
    mount_angle_degrees = {
        "LR": 214.309,
        "LM": 270,
        "LF": 325.931,
        "RF": 34.0685,
        "RM": 90,
        "RR": 146.172,
    }
    
    for leg, angle in mount_angle_degrees.items():
        angle_rad = np.deg2rad(angle)  # Convert to radians
        x = radius * np.cos(angle_rad)  # Compute X position
        y = radius * np.sin(angle_rad)  # Compute Y position
        mount_positions[leg] = (x, y)
    
    return mount_positions

if __name__ == "__main__":
    # setup controller stuff here

    
    # Setup control points
    # # Given starting position
    
    # Setup hexapod rotation/movement    
    hexapod = Hexapod(leg_configs, hexapod_configs)

    leg_lengths = { #Adjust thes for the legs (Assumes all legs are the same length)
    "coxa": 45,
    "femur": 110,
    "tibia": 193
    }
    mount_angle = {
        "LR": 214.309,
        "LM": 270,
        "LF": 325.931,
        "RF": 34.0685,
        "RM": 90,
        "RR": 146.172,
    }

    mount_positions = compute_mount_positions()

    hexapod_animation = HexapodAnimator(leg_lengths,mount_positions, mount_angle)

    main_loop(hexapod, hexapod_animation)