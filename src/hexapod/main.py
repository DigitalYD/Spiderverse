'''
    This application will start all other needed processes for this project.
    - Hexapod
    - Zigbee Controller input
    - i2c to hexapod
    - camera input
    - etc.

'''

from src.hexapod import Hexapod
from src.leg_configs import leg_configs
from src.hexapod_configs import hexapod_configs
from src.coord import *
from common.bezier2d import BezierCurve
from common.new_bezier import bezier_curve
import numpy as np
import time


walk_length = coord3D() 

def main_loop(hexapod):
    '''
        Implement movement and loop here
    '''
    # # Given starting position
    start_position = np.array([0,0,-80])

    # # # # Adjust control points relative to the starting position
    control_points = [
        start_position,  # P0: Start (Back, Grounded)
        start_position + np.array([10, 0, 25]),  # P1: Lift up
        start_position + np.array([20, 0, 60]),  # P2: Peak (Highest point)
        start_position + np.array([25, 0.0, 25]),  # P3: Descend
        start_position + np.array([20, 0.0, 0]),  # P4: grounded
        start_position  # P5: Return to Start
    ]
    #Generate Bezier curve
    # time.sleep(1)
    curve_points = bezier_curve(control_points, num_points=200)
    while True:
        for (x,y,z) in curve_points:
            positions = hexapod.inverse_kinematics((x,y,z))

    # for theta1,theta2,theta3 in enumerate(positions):
    #     print(np.degrees(theta1), np.degrees(theta2), np.degrees(theta3))
    #     hexapod.move_joint("LR", "coxa", int(np.degrees(theta1)))
    #     hexapod.move_joint("LR", "femur", int(np.degrees(theta2)))
    #     hexapod.move_joint("LR", "tibia", int(np.degrees(theta3)))
    #     time.sleep(0.1)



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
    
    
    ## These values go into inverse kinematics
    # hexPos = current position + body position + gaitposition
    # hexRot = bodyRotx, z, bodyroty= roty+gaitPos.roty
    pass


if __name__ == "__main__":
    # setup controller stuff here
    
    # Setup hexapod rotation/movement    
    hexapod = Hexapod(leg_configs, hexapod_configs)
    
    main_loop(hexapod)

    
    # x,y,z = int(np.degrees(-77.5)),int(np.degrees(193)),int(np.degrees(134))
    
    # hexapod.move_joint("left_rear", "coxa", int(np.degrees(x)))
    # hexapod.move_joint("left_rear", "femur", int(np.degrees(y)))
    # hexapod.move_joint("left_rear", "tibia", int(np.degrees(z)))

    # control_points = [
    #     np.array([0.0, 0.0, 100.0]),
    #     np.array([0.0, 0.0, 10.0]),
    # ]
    # curve_points = bezier_curve(control_points, num_points=50)
    # for (x,y,z) in curve_points:
    #     theta1, theta2, theta3 = hexapod.move_leg("left_rear", (x,y,z))
    #     print(np.degrees(theta1), np.degrees(theta2), np.degrees(theta3))
    #     hexapod.move_joint("left_rear", "coxa", int(np.degrees(theta1)))
    #     hexapod.move_joint("left_rear", "femur", int(np.degrees(theta2)))
    #     hexapod.move_joint("left_rear", "tibia", int(np.degrees(theta3)))

    time.sleep(2)

    # theta1,theta2,theta3 = hexapod.move_leg("left_rear", (90,75,-18))
    # hexapod.move_joint("left_rear", "coxa", int(np.degrees(theta1)))
    # hexapod.move_joint("left_rear", "femur", int(np.degrees(theta2)))
    # hexapod.move_joint("left_rear", "tibia", int(np.degrees(theta3)))


    # ## Rotate coxa
    # for i in range(0, 65):
    #     hexapod.move_joint("left_rear", "coxa", i)
    
    # time.sleep(1)
    # for i in range(65, -35, -1):
    #     hexapod.move_joint("left_rear", "coxa", i)

    # time.sleep(1)
    # for i in range(-35, 0):
    #     hexapod.move_joint("left_rear", "coxa", i)

    # ## Rotate femur
    # for i in range(0, 0):
    #     hexapod.move_joint("left_rear", "femur", i)
    
    # time.sleep(1)
    # for i in range(65, -35, -1):
    #     hexapod.move_joint("left_rear", "femur", i)

    # time.sleep(1)
    # for i in range(-35, 0):
    #     hexapod.move_joint("left_rear", "femur", i)

    # ## Rotate fibia
    # for i in range(0, 65):
    #     hexapod.move_joint("left_rear", "coxa", i)
    
    # time.sleep(1)
    # for i in range(65, -35, -1):
    #     hexapod.move_joint("left_rear", "coxa", i)

    # time.sleep(1)
    # for i in range(-35, 0):
    #     hexapod.move_joint("left_rear", "coxa", i)
