from math import sin, cos, pi, atan2, sqrt, acos

class Leg:
    def __init__(self, leg_index, leg_servo):
        '''
            Each leg is comprised of a coxa, femur, tibia
            ie: servo0: coxa (forward/back)
                servo1: femur (thigh)
                servo2: tibia (kneee) 
        '''
        super().__init__(leg_index)
        self.__leg_servo = leg_servo

        self.femur_len = 41
        self.coxa_len =  116
        self.tibia_len = 183
        self.leg_len = self.femur_len + self.coxa_len + self.tibia_len

    def forward_kinematics(self, x, y, z):
        '''
            Implement forward kinematics for leg movement
        '''
        pass

    def inverse_kinematics(self, x, y , z):
        '''
            Implement inverse kinematics for leg movement
        '''
        pass