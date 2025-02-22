'''
    This application will start all other needed processes for this project.
    - Hexapod
    - Zigbee Controller input
    - i2c to hexapod
    - camera input
    - etc.

'''

from src.hexapod import *
 

if __name__ == "__main__":
    hexapod = Hexapod(leg_configs)

    ## Rotate coxa
    for i in range(0, 65):
        hexapod.move_joint("left_rear", "coxa", i)
    
    time.sleep(1)
    for i in range(65, -35, -1):
        hexapod.move_joint("left_rear", "coxa", i)

    time.sleep(1)
    for i in range(-35, 0):
        hexapod.move_joint("left_rear", "coxa", i)

    ## Rotate femur
    for i in range(0, 0):
        hexapod.move_joint("left_rear", "femur", i)
    
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
