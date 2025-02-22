import time
import numpy as np
from adafruit_servokit import ServoKit

class Hexapod:
    def __init__(self):
        '''
            Initialize variables/controllers
        '''
        self.scontrol = ServoKit(channels=16, address=0x40, frequency=60)
        self.scontrol2 = ServoKit(channels=16, address=0x41, frequency=60)

        # setup the leg mappings for each pin of the servo
        self.legs = {
            "front_left": [0, 1, 2],
            "middle_left": [6, 7, 8],
            "rear_left": [9, 10, 11],
            "front_right": [0, 1, 2], # 2nd PCA9865 Right side
            "middle_right": [3, 4, 5],
            "rear_right": [6, 7, 8],
        }

        self.pca_mapping = {
            "front_left": self.scontrol,
            "middle_left": self.scontrol,
            "rear_left": self.scontrol,
            "front_right": self.scontrol2,
            "middle_right": self.scontrol2,
            "rear_right": self.scontrol2
        }

        #pulse range for servos
        self.set_servo_ranges(500,2500)

        self.femur_len = 41
        self.coxa_len =  116
        self.tibia_len = 183
        self.leg_len = self.femur_len + self.coxa_len + self.tibia_len

    def set_servo_ranges(self, min_pulse, max_pulse):
        '''set pulse width range for all servos on the controllers'''
        for i in range(16):
            self.scontrol.servo[i].set_pulse_width_range(min_pulse, max_pulse)
            self.scontrol2.servo[i].set_pulse_width_range(min_pulse, max_pulse)

    def move_leg(self, leg, angles):
        '''
            Move leg based on inverse kinematic angles
        '''
        pca = self.pca_mapping[leg]

        for i, angle in enumerate(angles):
            pca.servo[self.legs[leg][i]].angle = angle
        
    def inv_kin(self, x, y, z):
        '''
            Inverse Kintematics for Coxa, Femur, Tibia
            x: foward/back 
            y: side/side
            z: up/down (leg lift)
        '''

        # compute coxa angle (rotation)
        theta_coxa = np.degrees(np.arctan(y,x))

        #projected length to femur and tibia


        #law of cosines for femure and tibia

