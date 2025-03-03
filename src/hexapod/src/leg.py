from math import sin, cos, pi, atan2, sqrt, acos
from src.leg_configs import leg_configs
from src.servo import Servo
import numpy as np
from src.coord import *

class Leg:
    def __init__(self, name:str, leg_index:int, servo_pins, pulse_min, pulse_max, segment_lengths, toe_offsets, angleoffset):
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
        
        # coord from hexapod
        self.base_position = toe_offsets # Starting position of legs
        self.cur_pos = toe_offsets # Distance from center of body to coxa
        self.angle_offset = angleoffset
        self.leg_lift_height = 30 # might need moved to legs
        self.liftpos = 0 # number of positions a single leg is lefted (3)
        self.cur_angles = []
        
        #print(self.base_position.x, self.base_position.y, self.base_position.z)
        
        ## Uncomment this
        # if leg_index <= 2:
        #     self.servos = {
        #         "coxa" : Servo(servo_pins[0], pca=0x40, pulse_min=pulse_min, pulse_max=pulse_max),
        #         "femur" : Servo(servo_pins[1], pca=0x40, pulse_min=pulse_min, pulse_max=pulse_max),
        #         "tibia" : Servo(servo_pins[2], pca=0x40, pulse_min=pulse_min, pulse_max=pulse_max)
        #     }
        # else:
        #     self.servos = {
        #         "coxa" : Servo(servo_pins[0],  pca=0x41, pulse_min=pulse_min, pulse_max=pulse_max),
        #         "femur" : Servo(servo_pins[1], pca=0x41, pulse_min=pulse_min, pulse_max=pulse_max),
        #         "tibia" : Servo(servo_pins[2], pca=0x41, pulse_min=pulse_min, pulse_max=pulse_max)
        #     }
        
    
        # Initialize segment sizes of hexapod legs
        self._coxa_len = segment_lengths[0]     # Legnth of Coxa
        self._femur_len = segment_lengths[1]    # Length of Femur
        self._tibia_len = segment_lengths[2]    # Length of Tibia 
        self._leg_max_length = self._femur_len + self._coxa_len + self._tibia_len # Length of whole leg
    
        # Initialize thetas
        self._FemurAngle = 0.0
        self._TibiaAngle = 0.0
        self._CoxaAngle = 0.0
        
        # adjust joints to starting positions (May not need this)
        #self._joints = self.inverse_kinematics() # Calculate joint Angles for (endofactor of foot position) 


        
    def set_joint_angles(self, joint, angle):
        '''
            Set the angles of a specific joint in the leg
        '''
        if joint in self.servos:

            self.servos[joint].set_angle(angle)
        else:
            raise ValueError(f"Invalid joint: {joint}")

    def get_angles(self):
        # returns current angles of the joints in a list
        return [self._theta1, self._theta2, self._theta3]
    
    def set_angles(self, angles):
        # set the angles of the hexapod
        # ----------------------------------------------- Kinda want to normalize these to +-90 based off position on[]
        self._theta1, self._theta2, self._theta3 = angles


    def forward_kinematics(self):
        '''
            Implement forward kinematics for leg movement
            Compute foot position from joint angles
            don't forget to use offsets in calculation
        '''
        coxa_angle, femur_angle, tibia_angle = self.cur_angles
        
        # Calculate x/y coordinates
        x1 = self._coxa_len * np.cos(coxa_angle)
        y1 = self._coxa_len * np.sin(self._CoxaAngle)
        
        # Calculate vertical of femur
        G2 = np.sin(femur_angle) * self._femur_len
                
        # Calculate horizontal of femur
        p1 = np.cos(femur_angle) * self._femur_len
        
        # Calculate x & y of femur-tibia
        xc = np.cos(coxa_angle) * p1
        xy = np.sin(coxa_angle) * p1
        
        # Calculate coordinates of tibia-tip law of cosines
        arg = (self._femur_len**2 + hyp**2 - self._tibia_len**2) / (2.0 * self._femur_len * hyp)
        hyp = np.arccos(np.clip(arg, -1, 1))  # Ensure acos input is within valid range

        joint_positions = [
            [],
            [],
            [],
            []
        ]
        return joint positions


    def inverse_kinematics(self, bodyikposition, footpos):
        '''
            Implement inverse kinematics for leg movement
            Compute joint angles from desired foot positions
            don't forget to use offsets in calculation

            #     Z    (up)
            #     |    
            #     |    
            #     |    
            #     O------ y (forward)
            #    /
            #   /
            #  x (right/left)

        '''
        coxa_angle, femur_angle, tibia_angle = 0,0,0
        toepos = coord3D()

        #print("bodyikposition x,y,z",bodyikposition.x, bodyikposition.y, bodyikposition.z)
        
        # update foot position
        footpos.x = footpos.x + bodyikposition.x
        footpos.y = footpos.y + bodyikposition.y
        footpos.z = footpos.z + bodyikposition.z # Ensure correct Z height
        #print(f"Leg {self.name}: Translated foot position -> X: {footpos.x}, Y: {footpos.y}, Z: {footpos.z}")

        # Step 2: Convert foot position to the local leg frame using the coxa's angle offset

        # Apply rotation around the Z-axis to move to leg's local frame
        theta = np.deg2rad(self.angle_offset)
        toepos.x = footpos.x * np.cos(theta) - footpos.y * np.sin(theta)
        toepos.y = footpos.x * np.sin(theta) + footpos.y * np.cos(theta)
        toepos.z = footpos.z  # Maintain correct Z height

        #print(f"Leg {self.name}: Foot position -> X: {toepos.x}, Y: {toepos.y}, Z: {toepos.z}")

        # ##--------------------------
        # #### Variation 1
        # ##--------------------------
        
        # stiance between coxa and toe
        coxatoeDist = np.sqrt(toepos.x**2 + toepos.y**2)
        #print("Distance between coxa and toe: ", coxatoeDist)
        
        # Angle between SW line and ground in rad
        hyp = np.sqrt((coxatoeDist-self._coxa_len)**2 + toepos.z**2)
        
       # Angle between shoulder-hip line and femur
       ## Note: this was atan2 (3/2/25 on first work)
        ika1 = np.clip(np.arctan2(coxatoeDist - self._coxa_len, toepos.z),-1,1)

        # Compute inverse kinematics femur angle
        arg = (self._femur_len**2 + hyp**2 - self._tibia_len**2) / (2.0 * self._femur_len * hyp)
        ika2 = np.arccos(np.clip(arg, -1, 1))  # Ensure acos input is within valid range

        # tibia angle
        arg = (self._tibia_len**2 + self._femur_len**2 - hyp**2) / (2.0 * self._femur_len * hyp)
        tibia_angle = np.arccos(np.clip(arg,-1,1))
       
        # Set angles to solid ints for moving
        femur_angle = int(np.rad2deg((ika1 + ika2) - (np.pi / 2)))
        coxa_angle = int(np.rad2deg(np.arctan2(toepos.x, toepos.y)))
        tibia_angle = int(np.rad2deg((np.pi/2) - tibia_angle))
        
        self.cur_angles = [femur_angle, coxa_angle, tibia_angle]
        #print(int(np.rad2deg(coxa_angle)), int(np.rad2deg(femur_angle)), int(np.rad2deg(tibia_angle)))
  

        ##--------------
        ## Variation 2
        ##--------------

        return coxa_angle, femur_angle, tibia_angle