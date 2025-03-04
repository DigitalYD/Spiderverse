from math import sin, cos, pi, atan2, sqrt, acos
from src.leg_configs import leg_configs
from src.servo import Servo
import numpy as np
from src.coord import *

class Leg:
<<<<<<< HEAD
<<<<<<< HEAD
    def __init__(self, name:str, leg_index:int, servo_pins, pulse_min, pulse_max, segment_lengths, toe_offsets, angleoffset):
=======
<<<<<<< Updated upstream
    def __init__(self, name:str, leg_index:int, servo_pins, pulse_min, pulse_max, segment_lengths, coxa_ofs, angleoffset):
=======
    def __init__(self, name:str, leg_index:int, servo_pins, pulse_min, pulse_max, segment_lengths, toe_offsets, coxa_offset, angleoffset):
>>>>>>> Stashed changes
>>>>>>> 231e5a5 (Auto stash before checking out "HEAD")
=======
    def __init__(self, name:str, leg_index:int, servo_pins, pulse_min, pulse_max, segment_lengths, toe_offsets, coxa_offset, angleoffset):
>>>>>>> e45fea6 (6 leg sim & works)
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
<<<<<<< HEAD
<<<<<<< HEAD
        self.leg_lift_height = 30 # might need moved to legs
        self.liftpos = 0 # number of positions a single leg is lefted (3)
        self.cur_angles = []
        
        #print(self.base_position.x, self.base_position.y, self.base_position.z)
        
=======
<<<<<<< Updated upstream

=======
=======
>>>>>>> e45fea6 (6 leg sim & works)
        self.leg_lift_height = 30 # might need moved to legs
        self.liftpos = 0 # number of positions a single leg is lefted (3)
        self.cur_angles = []
        self.coxa_offset = coxa_offset
        self.coxa_leg_height = 0
<<<<<<< HEAD
        
        #print(self.base_position.x, self.base_position.y, self.base_position.z)
        
>>>>>>> Stashed changes
>>>>>>> 231e5a5 (Auto stash before checking out "HEAD")
=======
>>>>>>> e45fea6 (6 leg sim & works)
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
        self._CoxaAngle = 0.0
        self._FemurAngle = 0.0
        self._TibiaAngle = 0.0

        
<<<<<<< HEAD
        # adjust joints to starting positions (May not need this)
        #self._joints = self.inverse_kinematics() # Calculate joint Angles for (endofactor of foot position) 


        
=======
>>>>>>> e45fea6 (6 leg sim & works)
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
        return [self._CoxaAngle, self._FemurAngle, self._TibiaAngle]
    
    def set_angles(self, angles):
        # set the angles of the hexapod
        self._CoxaAngle, self._FemurAngle, self._TibiaAngle = angles

    def get_leg_lengths(self):
        leg_lengths = {
            "coxa": self._coxa_len,
            "femur": self._femur_len,
            "tibia": self._tibia_len,
        }
        return leg_lengths
    
    def set_leg_lengths(self, coxa, femur, tibia):
        self._coxa_len = coxa
        self._femur_len = femur
        self._tibia_len = tibia

    def forward_kinematics(self):
        '''
            Implement forward kinematics for leg movement
            Compute foot position from joint angles
            don't forget to use offsets in calculation
        '''
<<<<<<< HEAD
<<<<<<< HEAD
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

=======
<<<<<<< Updated upstream
        pass
        
    def normalize_angle(self, angle):
        return angle % (np.pi/2)
    
    def clamp_angle(self,angle, min_angle=0, max_angle=np.pi):
        value = self.normalize_angle(angle)
        return max(min_angle, min(max_angle, value))
=======
=======
>>>>>>> e45fea6 (6 leg sim & works)
        leg_positions = {}

        theta1, theta2, theta3 = self.cur_angles[0], self.cur_angles[1], self.cur_angles[2]
        coxa, femur, tibia = self._coxa_len, self._femur_len, self._tibia_len
        
        # Apply additional rotation to the leg at its mount point
        Xa = self.coxa_offset.x + coxa * np.cos(theta1)
        Ya = self.coxa_offset.y + coxa * np.sin(theta1)
        
        # Calculate vertical component of femur length
        G2 = np.sin(theta2) * femur
        
        # Calculate horizontal component of femur length
        P1 = np.cos(theta2) * femur
        
        # Calculate x and y coordinates of femur-tibia joint
        Xc = np.cos(theta1) * P1
        Yc = np.sin(theta1) * P1
        
        # Calculate H, phi1, phi2, phi3, Pp, P2, Yb, Xb, G1
        # to get the coordinates of the tibia-tip joint
        H = np.sqrt(tibia**2 + femur**2 - 2*tibia*femur*np.cos(np.radians(180) - theta3))
        phi1 = np.acos((femur**2 + H**2 - tibia**2) / (2*femur*H))
        phi2 = np.pi - theta3 - phi1
        phi3 = phi1 - theta2
        Pp = np.cos(phi3) * H
        P2 = Pp - P1
        Yb = np.sin(theta1) * Pp
        Xb = np.cos(theta1) * Pp
        G1 = - np.sin(phi2) * H

        # Create a list of joint locations
        leg_positions = np.array([
            [self.base_position.x, self.base_position.y, 0], # start joint
            [Xa, Ya, 0],  # coxa-femur joint
            [Xa + Xc, Ya + Yc, G2],  # femur-tibia joint
            [Xa + Xb, Ya + Yb, G1]  # tip of the leg
        ])
        return leg_positions

<<<<<<< HEAD
>>>>>>> Stashed changes
>>>>>>> 231e5a5 (Auto stash before checking out "HEAD")
=======
>>>>>>> e45fea6 (6 leg sim & works)

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
<<<<<<< HEAD
<<<<<<< Updated upstream
        coxa_angle, femur_angle, tibia_angle = 0,0,0
<<<<<<< HEAD
        toepos = coord3D()

        #print("bodyikposition x,y,z",bodyikposition.x, bodyikposition.y, bodyikposition.z)
=======
=======
=======
>>>>>>> e45fea6 (6 leg sim & works)
        toepos = coord3D()

        #print("bodyikposition x,y,z",bodyikposition.x, bodyikposition.y, bodyikposition.z)
<<<<<<< HEAD
>>>>>>> Stashed changes
>>>>>>> 231e5a5 (Auto stash before checking out "HEAD")
=======
>>>>>>> e45fea6 (6 leg sim & works)
        
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
        
<<<<<<< HEAD
=======
        #print(toepos.x, toepos.y)

        coxa_rad = np.arctan2(toepos.x, toepos.y)

>>>>>>> e45fea6 (6 leg sim & works)
        # stiance between coxa and toe
        coxatoeDist = np.sqrt(toepos.x**2 + toepos.y**2)
        #print("Distance between coxa and toe: ", coxatoeDist)
        
        # Angle between SW line and ground in rad
<<<<<<< HEAD
<<<<<<< HEAD
        hyp = np.sqrt((coxatoeDist-self._coxa_len)**2 + toepos.z**2)
        
       # Angle between shoulder-hip line and femur
       ## Note: this was atan2 (3/2/25 on first work)
        ika1 = np.clip(np.arctan2(coxatoeDist - self._coxa_len, toepos.z),-1,1)
=======
<<<<<<< Updated upstream
>>>>>>> 231e5a5 (Auto stash before checking out "HEAD")

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

<<<<<<< HEAD
        return coxa_angle, femur_angle, tibia_angle
=======
        #     #compute coxa
        #     theta_coxa = np.atan2(x,y) # + calibration constant here

        #     coxa_angle = theta_coxa
        #     femur_angle = gamma_femur
        #     tibia_angle = phi_tibia
        

            # ##--------------------------
            # #### Variation 4
            # ##--------------------------
            # hf = np.sqrt(y**2 + z**2)

            # # # Law of cosines for phi1
            # phi1 = np.arccos(np.clip((self._femur_len**2 + r3**2 - self._tibia_len**2) / (2.0 * self._femur_len * r3), -1.0, 1.0))
            
            # # Law of cosines for phi3
            # phi3 = np.arccos(np.clip( (self._femur_len**2 + self._tibia_len**2 - r3**2) / (2.0 * self._femur_len * self._tibia_len) , -1.0, 1.0))
            
            # # Adjusted Theta2 (Femur) to be in ±90° range
            # femur_angle = (phi2 - phi1) + np.radians(self._offsets["femur"]) #: geometric projection in femur's plane
            # #femur_angle = (np.pi/2)  - (phi2+phi1) # theta2 is referenced to horizontal

            # # Adjusted Theta3 (Tibia) to be in ±90° range
            # tibia_angle = np.pi - phi3 + np.radians(self._offsets["tibia"]) # Center around 0 instead of 90 || If problematic try (np.pi/2) - phi3

            # ## Variation 2
            # leg_length = np.sqrt(x**2 + z**2)
            # hf = hf = np.sqrt((leg_length - self._coxa_len)**2 / (y**2 + 1e-6))  # Small offset to prevent div by zero
            # a1 = np.atan((leg_length-self._coxa_len)/y)

            # a2= np.arccos(np.clip( (self._tibia_len**2-self._femur_len**2-hf**2)/-2*self._femur_len*self._tibia_len, -1.0, 1.0))
            # femur_angle = (np.pi/2) - (a1+a2)
            # b1 = np.arccos(np.clip((hf**2 - self._tibia_len**2 - self._femur_len**2)/-2*self._femur_len*self._tibia_len,-1.0, 1.0))
            # tibia_angle= (np.pi/2)- b1

            # coxa_angle = np.arctan2(y,x)


        return coxa_angle, femur_angle, tibia_angle
        
=======
=======
>>>>>>> e45fea6 (6 leg sim & works)
        hyp = np.sqrt((coxatoeDist-self._coxa_len)**2 + toepos.z**2)
        
       # Angle between shoulder-hip line and femur
        ika1 = np.clip(np.atan2(coxatoeDist - self._coxa_len, toepos.z),-1,1)

        # Compute inverse kinematics femur angle
        arg = (self._femur_len**2 + hyp**2 - self._tibia_len**2) / (2.0 * self._femur_len * hyp)
        ika2 = np.acos(np.clip(arg, -1, 1))  # Ensure acos input is within valid range

        femur_rad = (ika1 + ika2) - (np.pi / 2)

        # tibia angle
        arg = (self._tibia_len**2 + self._femur_len**2 - hyp**2) / (2.0 * self._femur_len * hyp)
        tibia_rad = np.arccos(np.clip(arg,-1,1))
       
        tibia_rad = (np.pi/2) - tibia_rad

        #print(int(np.rad2deg(coxa_angle)), int(np.rad2deg(femur_angle)), int(np.rad2deg(tibia_angle)))

        ##--------------
        ## Variation 2
        ##--------------
        # # coxa radians
        # coxatoeDist = np.sqrt(toepos.x**2 + toepos.y**2)
        # coxa_rad = np.arctan2(toepos.x, toepos.y)
        # hyp = np.sqrt((coxatoeDist-self._coxa_len)**2 + toepos.z**2)
        # # Angle between coxa-to-foot and femur
        # ika1 = np.arctan2(toepos.z, coxatoeDist - self._coxa_len)
        # ika2 = np.arccos(np.clip((self._femur_len**2 + hyp**2 - self._tibia_len**2) / (2.0 * self._femur_len * hyp), -1, 1))
        # femur_rad = ika1 + ika2 - np.pi / 2  # Convert to correct frame

        # # Step 6: Compute Tibia Angle (`theta3`)
        # tibia_rad = np.arccos(np.clip((self._femur_len**2 + self._tibia_len**2 - hyp**2) / (2.0 * self._femur_len * self._tibia_len), -1, 1))
        # tibia_rad = np.pi - tibia_rad  # Convert to correct relative frame

<<<<<<< HEAD
        self.cur_angles = [
            int(np.rad2deg(coxa_rad)), 
            int(np.rad2deg(femur_rad)), 
            int(np.rad2deg(tibia_rad))
            ]
        return coxa_rad, femur_rad, tibia_rad
>>>>>>> Stashed changes
>>>>>>> 231e5a5 (Auto stash before checking out "HEAD")
=======

        coxa_angle = int(np.rad2deg(coxa_rad))
        femur_angle = int(np.rad2deg(femur_rad))
        tibia_angle = int(np.rad2deg(tibia_rad))

        # print(coxa_rad, femur_rad, tibia_rad)
        # print(coxa_angle, femur_angle, tibia_angle)

        # self.set_joint_angles("coxa", coxa_angle)
        # self.set_joint_angles("femur", femur_angle)
        # self.set_joint_angles("tibia", tibia_angle)
        self.rad_angles = np.array([coxa_rad,femur_rad,tibia_rad])
        self.cur_angles = np.array([coxa_angle, femur_angle, tibia_angle])

        return self.rad_angles, self.cur_angles
>>>>>>> e45fea6 (6 leg sim & works)
