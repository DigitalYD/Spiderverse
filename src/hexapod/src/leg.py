from math import sin, cos, pi, atan2, sqrt, acos
from leg_configs import leg_configs
from servo import Servo
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
        self.joint_angles = offsets

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
        
        self.segment_lengths = segment_lengths
        self._femur_len = segment_lengths[0]
        self._coxa_len = segment_lengths[1]
        self._tibia_len = segment_lengths[2]
        self._leg_len = self._femur_len + self._coxa_len + self._tibia_len
    
        
    def set_joint_angle(self, joint, angle):
        '''
            Set the angle of a specific joint in the leg
        '''
        if joint in self.servos:
            self.servos[joint].set_angle(angle)
        else:
            raise ValueError(f"Invalid joint: {joint}")

    def forward_kinematics(self, joint_angles):
        '''
            Implement forward kinematics for leg movement
            Compute foot position from joint angles
            don't forget to use offsets in calculation
        '''
        theta0, theta1, theta2 = joint_angles
        L1, L2 = self.offsets
        
        x = (L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)) * np.cos(theta0)
        y = (L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)) * np.sin(theta0)
        z = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
        
        return np.array([x, y, z])

    def inverse_kinematics(self, target_position):
        '''
            Implement inverse kinematics for leg movement
            Compute joint angles from desired foot positions
            don't forget to use offsets in calculation
        '''
        x, y, z = target_position
        L1, L2 = self.segment_lengths[0], self.segment_lengths[1]  # Extract segment lengths (Femur, Tibia)

        d = np.sqrt(x**2 + y**2)  # Projected distance in XY plane
        D = (d**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)

        if abs(D) > 1:
            raise ValueError("Target out of reach")

        theta2 = np.arccos(D)  # Tibia joint (knee)
        theta1 = np.arctan2(z, d) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))  # Femur joint
        theta0 = np.arctan2(y, x)  # Coxa joint (hip)

        # Apply offsets from the dictionary
        theta0 += np.radians(self._offsets["coxa"])
        theta1 += np.radians(self._offsets["femur"])
        theta2 += np.radians(self._offsets["tibia"])

        self.joint_angles = [theta0, theta1, theta2]
        return self.joint_angles