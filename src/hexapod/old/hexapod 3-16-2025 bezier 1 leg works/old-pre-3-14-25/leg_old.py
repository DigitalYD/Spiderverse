from math import sin, cos, pi, atan2, sqrt, acos
from src.leg_configs import leg_configs
from src.servo import Servo
import numpy as np
from src.coord import *
from src.bezier2d import BezierCurve

class Leg:
    def __init__(self, name:str, leg_index:int, servo_pins, pulse_min, pulse_max, segment_lengths, toe_offsets, coxa_offset, angleoffset):
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
        self.num_joints = 4 # (Coxa, Femur, Tibia, End Effector)
        self.z_lift = 50.0 # Height of arc for end effector during swing phase
        self.revert_lift = 20.0 # height of arc for end effector when reverting to normal
        
        # coord from hexapod
        self.base_position = toe_offsets # Starting position of legs
        self.cur_pos = toe_offsets # Distance from center of body to coxa
        self.coxa_angle_offset = angleoffset
        self.liftpos = 0 # number of positions a single leg is lefted (3)
        self.cur_angles = []
        self.coxa_offset = coxa_offset
        self.coxa_leg_height = 0
        self.current_bezier_index = 0


        self.rest_angle = np.array([-20, 0, -80]) # Base resting position

        if self.name in ["LR", "LM", "LF"]:  # Left-side legs
            start_position = np.array([-20, 0, -80])  
        else:  # Right-side legs ("RR", "RM", "RF")
            start_position = np.array([20, 0, -80])

        self.control_points = {
            "start": start_position + np.array([0, 0, 0]),
            "lift": start_position + np.array([10, 0, 25]),
            "peak": start_position + np.array([20, 0, 75]),
            "lower": start_position + np.array([25, 0, 25]),  # The moment before grounding
            "touchdown": start_position + np.array([25, 0, 25]),  # The moment before grounding
            "grounded": start_position + np.array([25, 0, 5]),    # Fully on the ground
            "sliding": start_position+ np.array([30, 0, 0]),     # Sliding before reset
            "return": start_position + np.array([0, 0, 0])
        }
        
        self.bezier_curve = BezierCurve(self.control_points, num_pts = 100)

        ## Uncomment this on hexapod to run motors else error
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

        
    def set_joint_angles(self, joint, angle):
        '''
            Set the angles of a specific joint in the leg
        '''
        if joint in self.servos:
            self.servos[joint].set_angle(int(angle))
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

    # def inverse_kinematics(self, bodyikposition, footpos):
    #     '''
    #         Implement inverse kinematics for leg movement
    #         Compute joint angles from desired foot positions
    #         don't forget to use offsets in calculation

    #         #     Z    (up)
    #         #     |    
    #         #     |    
    #         #     |    
    #         #     O------ y (forward)
    #         #    /
    #         #   /
    #         #  x (right/left)

    #     '''

    #     # ##--------------------------
    #     # #### Variation 1
    #     # ##--------------------------
        
    #     toepos = coord3D()
    #     print(f"BoddyIK Position {self.name}: ",bodyikposition.x, bodyikposition.y, bodyikposition.z)
    #     print(f"BoddyIK Position {self.name}: ", footpos.x, footpos.y, footpos.z)

        
    #     # update foot position
    #     footpos.x = footpos.x + bodyikposition.x
    #     footpos.y = footpos.y + bodyikposition.y
    #     #footpos.z = footpos.z + bodyikposition.z # Ensure correct Z height

    #     #print(f"Leg {self.name}: Translated foot position -> X: {footpos.x}, Y: {footpos.y}, Z: {footpos.z}")

    #     # Apply rotation around the Z-axis to move to leg's local frame
    #     theta = np.deg2rad(self.coxa_angle_offset)
    #     toepos.x = footpos.x * np.cos(theta) - footpos.y * np.sin(theta)
    #     toepos.y = footpos.x * np.sin(theta) + footpos.y * np.cos(theta)
    #     #toepos.z = footpos.z  # Maintain correct Z height

    #     #print(f"Leg {self.name}: Foot position -> X: {toepos.x}, Y: {toepos.y}, Z: {toepos.z}")

    #     # ##--------------------------
    #     # #### Variation 1
    #     # ##--------------------------
        
    #     #print(toepos.x, toepos.y)
    #     # Coxa rotation should be mirrored for rear legs
    #     if self.name in ["RF", "RM", "RR"]:
    #         coxa_rad = -np.arctan2(toepos.y, toepos.x)
    #     else:
    #         coxa_rad = np.arctan2(toepos.y, toepos.x)

    #     # Correct rear legs to move with the front legs
    #     if self.name in ["RR", "LR"]:  
    #         coxa_rad = -coxa_rad 

    #     # distiance between coxa and toe
    #     coxatoeDist = np.sqrt(toepos.x**2 + toepos.y**2)
    #     #print("Distance between coxa and toe: ", coxatoeDist)
        
    #     # Angle between SW line and ground in rad
    #     hyp = np.sqrt((coxatoeDist-self._coxa_len)**2 + toepos.z**2)
        
    #    # Angle between shoulder-hip line and femur
    #     ika1 = np.clip(np.atan2(coxatoeDist - self._coxa_len, toepos.z),-1,1)

    #     # Compute inverse kinematics femur angle
    #     arg = (self._femur_len**2 + hyp**2 - self._tibia_len**2) / (2.0 * self._femur_len * hyp)
    #     ika2 = np.acos(np.clip(arg, -1, 1))  # Ensure acos input is within valid range

    #     femur_rad = (ika1 + ika2) - (np.pi / 2)

    #     # tibia angle
    #     arg = (self._tibia_len**2 + self._femur_len**2 - hyp**2) / (2.0 * self._femur_len * hyp)
    #     tibia_rad = np.arccos(np.clip(arg,-1,1))
       
    #     tibia_rad = (np.pi/2) - tibia_rad

    #     coxa_angle = np.rad2deg(coxa_rad)
    #     femur_angle = np.rad2deg(femur_rad)
    #     tibia_angle = np.rad2deg(tibia_rad)

        #print(int(np.rad2deg(coxa_angle)), int(np.rad2deg(femur_angle)), int(np.rad2deg(tibia_angle)))

        # self.rad_angles = np.array([coxa_rad,femur_rad,tibia_rad])
        # self.cur_angles = np.array([coxa_angle, femur_angle, tibia_angle])

        # return self.rad_angles, self.cur_angles

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

            # Pseudocode
            
            # Body frame
            x = effectorTarget.x - self.coxa_body_offset.x
            y = effectorTarget.y - self.coxa_body_offset.y

            # Eq 1
            # Calculate coxa, convert to degrees (180.0/np.pi), and apply offset,
            coxa_angle = (180.0/np.pi)*np.atan2(y,x) + 360 - self.coxa_angle_offset
            if coxa_angle >= 180:
                coxa_angle = coxa_angle - 360

            
            # Moving to leg Frame
            # Eq 2
            dx = effectorTarget.x - self.coxa_body_offset.x
            dy = effectorTarget.y - self.coxa_body_offset.y

            L1 = np.sqrt((dx**2 + dy**2) - self._coxa_len)
            L2 = effectorTarget.z - leg.femur.z
            L = np.sqrt(L2**2 + L1**2)
            
            # Eq 3
            arg_1 = (L2/L1)
            alpha_1 =  np.clip(np.arccos(arg_1),-1,1) # Clip result to prevent errors

            # Eq 4
            arg_2 = (self._tibia_len**2 - self._femur_len**2 - L**2)/(-2 * self._femur_len * L)
            alpha_2 = np.clip(np.arccos(arg_2), -1,1) # Clip result to prevent errors

            # Eq 5
            alpha = alpha_1 + alpha_2
            
            # Eq 6
            arg_b = (L**2 - self._tibia_len**2 - self._femur_len**2)/(-2*tibia*femur)
            beta = np.clip(np.arccos(arg_b) ,-1,1)
            
            # Perform offsets for each motor, and convert to angles
            femur_rad = 90 - (180.0/np.pi)* alpha
            tibia_rad = (180.0/np.pi) * beta
            '''
            # ##--------------------------
            # #### Variation 2 (Remake 3-7-2025)
            # ##--------------------------
            # Assumed neurtral position is at 0 degrees, else zero offset is required

            # Inverse kin equation 1
            # Body frame
            # x = effectorTarget.x - self.coxa_body_offset.x
            # y = effectorTarget.y - self.coxa_body_offset.y

            # # Eq 1
            # # Calculate coxa, convert to degrees (180.0/np.pi), and apply offset,
            # coxa_angle = (180.0/np.pi)*np.atan2(y,x) + 360 - self.coxa_angle_offset
            # if coxa_angle >= 180:
            #     coxa_angle = coxa_angle - 360

            
            # # Moving to leg Frame
            # # Eq 2
            # dx = effectorTarget.x - self.coxa_body_offset.x
            # dy = effectorTarget.y - self.coxa_body_offset.y

            # L1 = np.sqrt((dx**2 + dy**2) - self._coxa_len)
            # L2 = effectorTarget.z - leg.femur.z
            # L = np.sqrt(L2**2 + L1**2)
            
            # # Eq 3
            # arg_1 = (L2/L1)
            # alpha_1 =  np.clip(np.arccos(arg_1),-1,1) # Clip result to prevent errors

            # # Eq 4
            # arg_2 = (self._tibia_len**2 - self._femur_len**2 - L**2)/(-2 * self._femur_len * L)
            # alpha_2 = np.clip(np.arccos(arg_2), -1,1) # Clip result to prevent errors

            # # Eq 5
            # alpha = alpha_1 + alpha_2
            
            # # Eq 6
            # arg_b = (L**2 - self._tibia_len**2 - self._femur_len**2)/(-2*tibia*femur)
            # beta = np.clip(np.arccos(arg_b) ,-1,1)
            
            # # Perform offsets for each motor, and convert to angles
            # femur_rad = 90 - (180.0/np.pi)* alpha
            # tibia_rad = (180.0/np.pi) * beta

        return 


    def bezier_curve_completed(self):
        """Returns True if the leg's Bézier trajectory is complete."""
        return self.current_bezier_index >= len(self.bezier_curve.cp) - 1

    def get_swing_foot_position(self):
        """Returns the foot position during the swing phase using the Bézier curve."""
        return self.bezier_curve.get_point(self.current_bezier_index)