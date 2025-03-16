from math import sin, cos, pi, atan2, sqrt, acos
from leg_configs import leg_configs

class Leg:
    def __init__(self, name:str, leg_index:int, coxa:int, femur:int, tibia:int, offsets=None):
        '''
            Each leg is comprised of a coxa, femur, tibia
            ie: servo0: coxa (forward/back)
                servo1: femur (thigh)
                servo2: tibia (kneee) 
            performs kinematics for each leg
        '''
        super().__init__()
        self.name = name
        self.leg_index = leg_index
        self.servos = {
            "coxa" : coxa,
            "femur" : femur,
            "tibia" : tibia
        }

        self._femur_len = 41
        self._coxa_len =  116
        self._tibia_len = 183
        self._leg_len = self.femur_len + self.coxa_len + self.tibia_len
        
        if offsets is None:
            offsets = {
                "coxa" : [0,0,0],
                "femur" : [0,0,0],
                "tibia" : [0,0,0]
            }
        self._offsets = offsets

    def set_offsets(self, offset):
        '''
            Send a dictionary into _offsets
        '''
        self._offsets = offset
        
    def set_offset(self, joint:str, offset):
        '''
            Change a specific value into the joint
        '''
        if joint in self._offsets:            
            self._offsets[joint] = offset
        else:
            raise ValueError(f"Invalid joint name: {joint}")
        
    def get_offsets(self):
        '''
            Returns the offsets dictionary
        '''
        return self._offsets
    
    def get_offset(self, joint:str):
        '''
            Returns the offset at a specific joint on the leg
        '''
        if joint in self._offsets:
            return self._offsets[joint]
        else:
            raise ValueError(f"Invalid joint name: {joint}")

    def forward_kinematics(self, target_x, target_y, target_z):
        '''
            Implement forward kinematics for leg movement
            Compute foot position from joint angles
            don't forget to use offsets in calculation
        '''
        pass
        

    def inverse_kinematics(self, x, y , z):
        '''
            Implement inverse kinematics for leg movement
            Compute joint angles from desired foot positions
            don't forget to use offsets in calculation
        '''
        pass
    
    def move_leg(self, angles):
        '''
            Perhaps this is where I place code to move leg
        '''
        pass