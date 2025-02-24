import time
import numpy as np
from threading import Lock
from src.leg import Leg
from src.servo import Servo
from src.leg_configs import leg_configs
from src.hexapod_configs import hexapod_configs

PI = np.pi
DEGTORAD = np.radians(1)
RADTODEG = np.degrees(1)
MAXITER = 250


class Hexapod:
    valid_modes = {"initialize","stand","walk","rotate","shutdown","calibaration"}
    valid_gaits = {"tripod", "wave", "ripple", "quadrupedal", "tetrapod"}
    def __init__(self, leg_configs, hexapod_configs):
        '''
            Initialize variables/controllers
            Body-Level kinematics (gait, stability, coordination)
            modes: initialize, stand, walk, rotate, shutdown, calibration
            gaits: tripod, wave, ripple

            leftRear        Left Front
            [0]     [1]     [2]
                \\    |    /
                ----------
                ----------
               /     |    \\
            [5]     [4]      [3]
            rightRear       Right Front

            hook up legs 5 & 0 at pins 0 on the servoboard.
        '''
        # Initialize basic variables
        self.max_speed = 1.0
        self.legs = {}
        self.mode = "tripod"

        for body_pose, config in hexapod_configs.items():
            self.body_pose = config["pose"]
            print(self.body_pose)
        # setup gaits
        self.tripod_gait_groups = [["left_rear", "right_middle", "left_front"],["right_front","left_middle","right_rear"]]

        # Create leg class
        for leg_name, config in leg_configs.items():
            leg_index = config["leg_index"]
            servo_pins = config["servos"]  # Extract servo pins
            segment_lengths = config["segment_lengths"]
            offsets = config["offsets"]  # Extract joint offsets
            pulse_min = config["pulsemin"]
            pulse_max = config["pulsemax"]
            print(offsets)
            # Create Leg objects (which internally create Servos)
            self.legs[leg_name] = Leg(leg_name, leg_index, servo_pins, pulse_min, pulse_max, segment_lengths, offsets)
    
    def get_legs(self):
        return self.legs
    
    def move_leg(self, leg_id, target_position):
        '''
            get angles from inverse kinematics
        '''
        return self.legs[leg_id].inverse_kinematics(target_position)
    
    
    def move_joint(self, leg, joint, amount):
        if leg in self.legs:
            self.legs[leg].set_joint_angles(joint, amount)
        else:
            raise ValueError(f"Invalid Leg: {leg}")

    def get_leg_positions(self):
        return {name: leg.forward_kinematics(leg.joint_angles) for name, leg in self.legs.items()}
    
    def stand(self):
        self.set_mode("stand")

    def set_mode(self, mode):
        self.mode = mode

    def generate_tripod_step_positions(self, stride_length=10, lift_height=5):
        """Generates step positions for a tripod gait."""
        step_positions = {}
        lift_offset = np.array([0, 0, lift_height])
        forward_offset = np.array([stride_length, 0, 0])
        
        for group in self.tripod_gait_groups:
            for leg_name in group:
                base_position = self.get_leg_positions()[leg_name]
                lifted_position = base_position + lift_offset
                forward_position = lifted_position + forward_offset
                step_positions[leg_name] = forward_position
        
        return step_positions

    # Gaits are below
    def tripod_gait(self, steps):
        '''
            3 legs move at the same time
            ex (0,4,2) (1,3,5)
            "no delay"
        '''
        step_positions = self.generate_tripod_step_positions()
        for group in self.tripod_gait_groups:
            for leg_name in group:
                self.move_leg(leg_name, step_positions[leg_name])


    def triple_gait(self,steps):
        '''
            3 legs at the same time
            1/6 phase offset
            phase_delay = speed/6
            sleep(phase_delay)
        '''
        pass
    def wave_gait(self, steps):
        ''' One leg is lifted at a time 0,1,2,5,4,3'''
        pass

    def ripple_gait(self, steps):
        '''
            One leg at a time 0, 3, 1, 5, 2, 4
            happens at phases see image
            phase 1/6
        '''
        pass
    
    def quad_gait(self, steps):
        pass

    def tetra_gait(self, steps):
        pass


    def walk(self, gait="tripod", direction="", speed=1.0):
        if gait == "tripod":
            self.tripod_gait(direction, speed)
        elif gait == "wave": #also known as ripple
            self.wave_gait(direction, speed)
        elif gait == "ripple":
            self.ripple_gait(direction, speed)
        elif gait == "quadrupedal":
            self.quad_gait(speed)
        elif gait == "tetrapod":
            self.tetra_gait(speed)
        else:
            raise ValueError(f"Invalid gait {gait}")




if __name__ == "__main__":
    hexapod = Hexapod(leg_configs, hexapod_configs)

    #     Z    
    #     |    
    #     |    
    #     |    
    #     O------ X
    #    /
    #   /
    #  Y

    # x, y, z
    # theta1, theta2, theta3 = hexapod.move_leg("left_rear", [10,10,-10])

    # theta1 = np.degrees(theta1)
    # theta2 = np.degrees(theta2)
    # theta3 = np.degrees(theta3)

    # print(theta1, theta2, theta3)

    # hexapod.move_joint("left_rear", "coxa",theta1)
    # hexapod.move_joint("left_rear", "femur",theta2)
    # hexapod.move_joint("left_rear", "tibia",theta3)

    for i in range(0, 90):
        hexapod.move_joint("left_rear", "tibia", i)

    for i in range(0, 160):
        hexapod.move_joint("left_rear", "femur", i)
    
    for i in range(160, 0, -1):
        hexapod.move_joint("left_rear", "femur", i)

    for i in range(90, 0, -1):
        hexapod.move_joint("left_rear", "tibia", i)
    time.sleep(1)
    time.sleep(2)
    for i in range(0, 65):
        hexapod.move_joint("left_rear", "coxa", i)
    
    time.sleep(1)
    for i in range(65, -35, -1):
        hexapod.move_joint("left_rear", "coxa", i)

    time.sleep(1)
    for i in range(-35, 0):
        hexapod.move_joint("left_rear", "coxa", i)