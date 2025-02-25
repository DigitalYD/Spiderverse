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
from common.bezier2d import BezierCurve
from common.new_bezier import bezier_curve
import numpy as np
import time

if __name__ == "__main__":
    hexapod = Hexapod(leg_configs, hexapod_configs)
    
    
    
    
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
    # control_points = [
    #     np.array([0.0, 0, 0.2]) * 300,    # P0: Start (Back, Grounded)
    #     np.array([0.1, 0.2, 0.2]) * 300,  # P1: Lift up
    #     np.array([0.2, 0.3, 0.4]) * 300,  # P2: Peak (Highest point)
    #     np.array([0.3, 0.2, 0.2]) * 300,  # P3: Descend
    #     np.array([0.2, 0, 0.2]) * 300,    # P4: End (Forward, Grounded)
    #     np.array([0.0, 0, 0.2]) * 300     # P5: Return to Start
    # ]

    # #Generate Bezier curve
    # curve_points = bezier_curve(control_points, num_points=50)
    # for (x,y,z) in curve_points:
    #     theta1, theta2, theta3 = hexapod.move_leg("left_rear", (x,y,z))
    #     print(np.degrees(theta1), np.degrees(theta2), np.degrees(theta3))
    #     hexapod.move_joint("left_rear", "coxa", int(np.degrees(theta1)))
    #     hexapod.move_joint("left_rear", "femur", int(np.degrees(theta2)))
    #     hexapod.move_joint("left_rear", "tibia", int(np.degrees(theta3)))


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
