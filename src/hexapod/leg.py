from math import sin, cos, pi, atan2, sqrt, acos
from src.leg_configs import leg_configs
from src.servo import Servo
import numpy as np

class Leg:
    def __init__(self, name:str, leg_index:int, servo_pins, pulse_min, pulse_max, segment_lengths, offsets=None):
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
        self.base_position = offsets
        self._offsets = offsets

        if leg_index <= 2:
            self.servos = {
                "coxa" : Servo(servo_pins[0], offsets["coxa"], pca=0x40, pulse_min=pulse_min, pulse_max=pulse_max),
                "femur" : Servo(servo_pins[1], offsets["femur"], pca=0x40, pulse_min=pulse_min, pulse_max=pulse_max),
                "tibia" : Servo(servo_pins[2], offsets["tibia"], pca=0x40, pulse_min=pulse_min, pulse_max=pulse_max)
            }
        else:
            self.servos = {
                "coxa" : Servo(servo_pins[0], offsets["coxa"], pca=0x41, pulse_min=pulse_min, pulse_max=pulse_max),
                "femur" : Servo(servo_pins[1], offsets["femur"], pca=0x41, pulse_min=pulse_min, pulse_max=pulse_max),
                "tibia" : Servo(servo_pins[2],offsets["tibia"],  pca=0x41, pulse_min=pulse_min, pulse_max=pulse_max)
            }
        

        # Initialize segment sizes of hexapod legs
        self.segment_lengths = segment_lengths
        self._coxa_len = segment_lengths[0]     # Legnth of Coxa
        self._femur_len = segment_lengths[1]    # Length of Femur
        self._tibia_len = segment_lengths[2]    # Length of Tibia 
        self._leg_length = self._femur_len + self._coxa_len + self._tibia_len # Length of whole leg

        self._side_length = 130.0 # Length from one coxa tonext

        # Initialize thetas
        self._FemurAngle = 0.0
        self._TibiaAngle = 0.0
        self._CoxaAngle = 0.0
        #self._joints = self.forward_kinematics() # Calculate joint Angles for (endofactor of foot position) 

        
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



    def forward_kinematics(self, joint_angles):
        '''
            Implement forward kinematics for leg movement
            Compute foot position from joint angles
            don't forget to use offsets in calculation
        '''
        pass
        
    def normalize_angle(self, angle):
        return angle % (np.pi/2)
    
    def clamp_angle(self,angle, min_angle=0, max_angle=np.pi):
        value = self.normalize_angle(angle)
        return max(min_angle, min(max_angle, value))

    def inverse_kinematics(self, target_position= None):
        '''
            Implement inverse kinematics for leg movement
            Compute joint angles from desired foot positions
            don't forget to use offsets in calculation
        '''
        if target_position is None:
            target_position = self._joints[3]

        x, y, z = target_position
        coxa_angle, femur_angle, tibia_angle = 0,0,0

        # # # Theta1 (rotation around coxa joint)
        # coxa_angle = np.arctan2(y, x) + np.radians(self._offsets["coxa"])

        # # Projected distances
        # r1 = np.sqrt(x**2 + y**2)
        # r2 = z - self._coxa_len

        # # Angle phi2
        # phi2 = np.arctan2(r2, r1)

        # # Distance from femur joint to target
        # r3 = np.sqrt(r1**2 + r2**2)

        # compute target femur-to-toe (l3)
        l0 = np.sqrt(x**2 + y**2) - self._coxa_len
        l3 = np.sqrt(l0**2 + z**2)
        print(l0,l3)
        if ((l3 < (self._tibia_len + self._femur_len)) and (l3 > (self._tibia_len - self._femur_len))):
            #compute tibia angle
            phi_tibia = np.arccos(np.clip((self._femur_len**2 + self._tibia_len**2 - l3**2)/(2*self._femur_len*self._femur_len),-1,1))
            # Add calibration constant here
            phi_tibia = self.clamp_angle(phi_tibia)

            #compute femur
            gamma_femur = np.atan2(z,l0)
            phi_femur = np.arccos(np.clip((self._femur_len**2 + l3**2 - self._tibia_len)/(2*self._femur_len*l3),-1,1))
            # add calibaration consatanthere
            gamma_femur = self.clamp_angle(gamma_femur+phi_femur)

            #compute coxa
            theta_coxa = np.atan2(x,y) # + calibration constant here

            coxa_angle = theta_coxa
            femur_angle = gamma_femur
            tibia_angle = phi_tibia
        

            # ##--------------------------
            # #### Variation 3
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
        return  coxa_angle, femur_angle, tibia_angle