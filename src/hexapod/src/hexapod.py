import time
import numpy as np
from threading import Lock
from concurrent.futures import ThreadPoolExecutor
from src.leg import Leg
from src.servo import Servo
from src.leg_configs import leg_configs
from src.hexapod_configs import hexapod_configs
from src.coord import *


PI = np.pi
DEGTORAD = np.radians(1)
RADTODEG = np.degrees(1)


class Hexapod:
    VALID_MODES = ["initialize","stand","walk","rotate","shutdown","calibaration"]
    VALID_GATES = ["tripod", "wave", "ripple", "quadrupedal", "tetrapod"]
    VALID_LEGS = ["LR","LM", "LF", "RF", "RM","RR"]
    
    def __init__(self, leg_configs, hexapod_configs):
        '''
            Initialize variables/controllers
            Body-Level kinematics (gait, stability, coordination)
            modes: initialize, stand, walk, rotate, shutdown, calibration
            gaits: tripod, wave, ripple

            LR      LM      LF
            [0]     [1]     [2]
                \\    |    /
                ----------
                ----------
               /     |    \\
            [5]     [4]      [3]
            RR      RM       RF

            hook up legs 5 & 0 at pins 0 on the servoboard.
            
            Legs have 3 position/modes
            - "standing" (still), "in_air" (following bezier curve), "slide" (when leg is pushing off on the ground)
        '''
        # Initialize basic variables
        self.max_speed = 1.0
        self.legs = {}
        self.gait = "tripod"
        self.mode = "standing"
        self.leg_mode = "standing" # "standing" (still), "in_air" (following bezier curve), "touchdown", "slide" (when leg is moving on the ground)
        self.turning_radius = float('inf') # straight walk
        self.angular_speed = 0.0 # turning speed
        self.multipler = 0 # multiplier for length of each step (might use)

        # Body Position
        self.body_position = coord3D()
        self.body_rotation = coord3D()
        self.hexapod_height = 0 # How high the hexapod is from the ground

        for body_pose, config in hexapod_configs.items():
            self.body_pose = config["pose"]
            coxa_offsets = config["coxa_offsets"]
        
        self.body_coxa_offsets = {}
        self.rad_angles = {}
        self.leg_angles = {}
        # setup hexapod body positions in 3D space
        self.position = coord3D()
        self.last_position = coord3D()
        self.rotation = coord3D()
        self.last_rotation = coord3D()
        self.step_count = 0
        
        # setup gaits
        self.tripod_gait_sequence = [["LR", "RM", "LF"], ["RF","LM","RR"]]
        self.ripple_gait_sequence = [["LR","RR"],["LM","RM"],["LF","RF"]]
        self.wave_gait_sequence =  [["LR"], ["LM"],["LF"],["RF"],["RM"],["RR"]]
        self.phase_offsets = {
            "tripod": { "LR": 0, "LM": 1, "LF": 0, "RF": 1, "RM": 0, "RR": 1 },  # Two tripod phases
            "ripple":   { "LR": 0, "LM": 1, "LF": 2, "RF": 2, "RM": 1, "RR": 0 },  # Wave gait with 6 phases
            "wave": { "LR": 0, "LM": 2, "LF": 4, "RF": 6, "RM": 8 , "RR": 10  }, # Ripple gait with 12 phases
        }
        self.gait_pos = {}
                
        # setup temp vars
        leg_index = {}
        servo_pins = {}
        angleoffset = {}
        pulse_min = {}
        pulse_max={}
        segment_lengths = {}
    
        # Create leg class
        for leg_name, config in leg_configs.items():

            self.body_coxa_offsets[leg_name] = coord3D()
            self.body_coxa_offsets[leg_name].x = coxa_offsets[leg_name][0] # Distance x from center of body to coxa
            self.body_coxa_offsets[leg_name].y = coxa_offsets[leg_name][1] # Distance y from center of body to coxa
            self.body_coxa_offsets[leg_name].z = coxa_offsets[leg_name][2] # Distance z from center of height of body

            self.gait_pos[leg_name] = coord3D()
            
            leg_index[leg_name] = config["leg_index"]
            servo_pins[leg_name] = config["servos"]  # Extract servo pins
            segment_lengths[leg_name] = config["segment_lengths"]
            pulse_min[leg_name] = config["pulsemin"]
            pulse_max[leg_name] = config["pulsemax"]
            angleoffset[leg_name] = config["angleoffset"]
            self.legs[leg_name] = Leg(leg_name, leg_index[leg_name], servo_pins[leg_name], pulse_min[leg_name], pulse_max[leg_name], segment_lengths[leg_name], coxa_offsets[leg_name], angleoffset[leg_name])


        #  # Initial leg positions [x,y,z] || Will need adjusting
        # temp_vals = {
        #     "LR": {
        #         "x": -np.cos(np.deg2rad(56.172)) * (segment_lengths["LR"][0] + segment_lengths["LR"][1] - segment_lengths["LR"][2]),
        #         "y": -np.sin(np.deg2rad(56.172)) * (segment_lengths["LR"][0] + segment_lengths["LR"][1] - segment_lengths["LR"][2]),
        #         "z": segment_lengths["LR"][2]-30, # Adding -30 brings leg out away from hexapod
        #     },
        #     "LM": {
        #         "x": -(segment_lengths["LM"][0] + segment_lengths["LM"][1]),
        #         "y": 0,  # Middle leg should have zero Y if centered
        #         "z": segment_lengths["LM"][2],
        #     },
        #     "LF": {
        #         "x": -np.cos(np.deg2rad(55.931)) * (segment_lengths["LF"][0] + segment_lengths["LF"][1] - segment_lengths["LR"][2]),
        #         "y": np.sin(np.deg2rad(55.931)) * (segment_lengths["LF"][0] + segment_lengths["LF"][1]  - segment_lengths["LR"][2]),
        #         "z": segment_lengths["LF"][2],
        #     },
        #     "RF": {
        #         "x": np.cos(np.deg2rad(55.931)) * (segment_lengths["RF"][0] + segment_lengths["RF"][1]  - segment_lengths["LR"][2]),
        #         "y": np.sin(np.deg2rad(55.931)) * (segment_lengths["RF"][0] + segment_lengths["RF"][1]  - segment_lengths["LR"][2]),
        #         "z": segment_lengths["RF"][2],
        #     },
        #     "RM": {
        #         "x": (segment_lengths["RM"][0] + segment_lengths["RM"][1]),
        #         "y": 0,  # Middle leg should have zero Y if centered
        #         "z": segment_lengths["RM"][2],
        #     },
        #     "RR": {
        #         "x": np.cos(np.deg2rad(56.172)) * (segment_lengths["RR"][0] + segment_lengths["RR"][1]  - segment_lengths["LR"][2]),
        #         "y": -np.sin(np.deg2rad(56.172)) * (segment_lengths["RR"][0] + segment_lengths["RR"][1]  - segment_lengths["LR"][2]),
        #         "z": segment_lengths["RR"][2],
        #     },
        #     }
        
        # leg_lengths = {}
        # for leg,vals in temp_vals.items():
        #     leg_lengths[leg] = np.sqrt(vals['x']**2 + vals['y']**2 + vals['z']**2)

        # toe_offsets = {}
        # for name, values in temp_vals.items():
        #     # if name == "LR" or name == "LM" or name == "LF":
        #     toe_offsets[name] = coord3D(x=values['x'], y=values['y'], z=values['z'])
        #     self.legs[name] = Leg(name, leg_index[name], servo_pins[name], pulse_min[name], pulse_max[name], segment_lengths[name], toe_offsets[name], coxa_offsets[name], angleoffset[name])
        
        self.leg_states = {leg: "standing" for leg in self.legs}  # "standing", "in_air", "slide" | standing, moving along 2d bezier curve, touchdown/sliding back (could also do for different leg "heights", but later maybe)
        
        self.num_legs = len(self.legs)

    def get_legs(self):
        return self.legs

    def move_individual_leg(self, leg_id):
        '''
            get angles from inverse kinematics
        '''
        self.legs[leg_id].set_joint_angles("coxa", self.leg_angles[leg_id][0])
        self.legs[leg_id].set_joint_angles("femur", self.leg_angles[leg_id][1])
        self.legs[leg_id].set_joint_angles("tibia", self.leg_angles[leg_id][2])

    def move_joint(self, leg, joint, amount):
        ''' Move a specific leg and joint'''
        if leg in self.legs:
            self.legs[leg].set_joint_angles(joint, amount)
        else:
            raise ValueError(f"Invalid Leg: {leg}")

    def get_leg_positions(self):
        ''' May not need this? performs kinematics, should just pull the radians or angles directly'''
        return self.forward_kinematics()

    def set_mode(self, mode):
        ''' Set hexapod's mode'''
        self.mode = mode

    def get_leg_lengths(self):
        ''' Return leg length'''
        legs = {}
        for i, leg_id in enumerate(self.legs):
            legs[leg_id] = self.legs[leg_id].get_leg_lengths()
        return legs
    
    def rotation_2d(self, theta):
        ''' Accepts an angle, converts it to radians and returns rotation 2d'''
        theta = np.deg2rad(theta)
        return np.array([[np.cos(theta), -np.sin[theta]], [np.sin(theta), np.cos(theta)]])
    
    def rot_mat_3d_x(self, theta):
        ''' Pitch angle, converts it to radians and returns '''
        theta = np.deg2rad(theta)
        return np.array([[1, 0, 0],
                         [0, np.cos(theta), -np.sin(theta)],
                         [0, np.sin(theta), np.cos(theta)]])
    
    def rot_mat_3d_y(self, theta):
        ''' Yaw an angle, converts it to radians and returns '''
        theta = np.deg2rad(theta)
        return np.array([[np.cos(theta), 0, np.sin(theta)],
                         [0, 1, 0],
                         [-np.sin(theta), 0, np.cos(theta)]])
    
    def rot_mat_3d_z(self, theta):
        ''' Rolling angle, converts it to radians and returns '''
        theta = np.deg2rad(theta)
        return np.array([[np.cos(theta), -np.sin(theta), 0],
                         [np.sin(theta), np.cos(theta), 0],
                         [0, 0, 1]])
    
    def update_leg_ik(leg, bodyikPosition, footposition):
        ''' function to perform inverse kinematics with threats'''
        return leg, leg.inverse_kinematics(bodyikPosition, footposition)


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


    def move_legs(self):
        '''
            get angles from inverse kinematics
        '''
        active_legs = self.get_active_legs_in_gait()
        for leg_id in active_legs:
            self.legs[leg_id].set_joint_angles("coxa", self.leg_angles[leg_id][0])
            self.legs[leg_id].set_joint_angles("femur", self.leg_angles[leg_id][1])
            self.legs[leg_id].set_joint_angles("tibia", self.leg_angles[leg_id][2])


    def get_active_legs_in_gait(self):
        """Returns a list of legs that should move at this step"""
        active_legs = []
        
        # Select the correct gait sequence
        if self.gait == "tripod":
            gait_sequence = self.tripod_gait_sequence
        elif self.gait == "wave":
            gait_sequence = self.wave_gait_sequence
        elif self.gait == "ripple":
            gait_sequence = self.ripple_gait_sequence
        else:
            return []  # Invalid gait, return empty list

        #  step_count does not exceed gait sequence length
        gait_cycle_length = len(gait_sequence)
        if self.step_count >= gait_cycle_length:
            return []

        # Select the legs that should move in this step
        current_leg_group = gait_sequence[self.step_count]

        for leg in current_leg_group:
            # Ensure previous legs in the sequence have completed "sliding" before moving
            previous_legs_completed = all(
                self.leg_states[l] == "sliding"
                for l in self.legs
                if self.phase_offsets[self.gait].get(l, float('inf')) < self.phase_offsets[self.gait].get(leg, float('inf'))
            )

            if not previous_legs_completed:
                continue  # Prevent current leg from moving until previous legs finish

            active_legs.append(leg)

        #print(f"Active Legs: {active_legs}")
        return active_legs


    def inverse_kinematics(self):
        # call leg inverse kinematics and feed it

        rotation = coord3D()
        body = coord3D()
        rad_angles = {}
        angles = {}

        footposition = {}
        bodyikPosition = {}
        position = {}
        bezier_index = {}
        active_legs = self.get_active_legs_in_gait()
    
        if not active_legs:
            return self.rad_angles

        for leg in active_legs:
            position[leg] = self.legs[leg].get_swing_foot_position()

            # get the current foot position and add the "bezier curve position to it"
            footposition[leg] = coord3D()
            footposition[leg].x = self.legs[leg].cur_pos.x + self.body_position.x + position[leg][0] # self.gait_pos[leg].x
            footposition[leg].y = self.legs[leg].cur_pos.y + self.body_position.y + position[leg][1] # self.gait_pos[leg].y
            footposition[leg].z = self.legs[leg].cur_pos.z + self.body_position.z + position[leg][2] # self.gait_pos[leg].z
            
            # Rotation
            rotation.x = self.body_rotation.x
            rotation.y = self.body_rotation.y 
            rotation.z = self.body_rotation.z + self.gait_pos[leg].roty
            
            # ------------------------
            # Body kinemtatics
            # ------------------------
            body.x = footposition[leg].x + self.body_coxa_offsets[leg].x
            body.y = footposition[leg].y + self.body_coxa_offsets[leg].y
            body.z = footposition[leg].z

            # calculate position corrections using rotation matrix
            # https://en.wikipedia.org/wiki/Rotation_matrix
            Rx = self.rot_mat_3d_x(rotation.x) # pitch
            Ry = self.rot_mat_3d_y(rotation.y) # yaw
            Rz = self.rot_mat_3d_z(rotation.z) # roll
            
            if leg in ["LM","RM"]:
               R = Ry
            else: 
                R = Rz @ Ry @ Rx
            '''[[1. 0. 0.] no rotation
                [0. 1. 0.]
                [0. 0. 1.]]            
            '''
            temp_pos = np.array([body.x, body.y, body.z]) #body position in space
            rotated_point = R @ temp_pos

            bodyikPosition[leg] = coord3D()
            bodyikPosition[leg].from_array(rotated_point)
            
            #print("BodyikPosition: ", bodyikPosition.x, bodyikPosition.y, bodyikPosition.z)
        
            # Send values to legs for leg kinematics
            #self.rad_angles[leg], self.leg_angles[leg] = self.legs[leg].inverse_kinematics(bodyikPosition, footposition)
    
        with ThreadPoolExecutor() as executor:
            # Submit inverse kinematics tasks for each leg
            futures = {
                executor.submit(self.legs[leg].inverse_kinematics, bodyikPosition[leg], footposition[leg]): leg
                for leg in active_legs
            }

            for future in futures:
                leg = futures[future]
                rad_angle, angle = future.result()
                rad_angles[leg] = rad_angle
                angles[leg] = angle
                self.rad_angles[leg] = rad_angles[leg]
                self.leg_angles[leg] = angles[leg]

                # Ensure Bezier index resets properly at "return"
                if self.legs[leg].current_bezier_index >= self.legs[leg].bezier_curve.index_map["return"]:
                    self.leg_states[leg] = "standing"
                    self.legs[leg].current_bezier_index = 0  # Reset Bezier index to start new cycle
                    #print(f"{leg} has reset to 0 index.")

                elif self.legs[leg].current_bezier_index >= self.legs[leg].bezier_curve.index_map["sliding"]:
                    self.leg_states[leg] = "sliding"
                    #print(f" {leg} has reached 'sliding'.")

                elif self.legs[leg].current_bezier_index >= self.legs[leg].bezier_curve.index_map["touchdown"]:
                    self.leg_states[leg] = "touchdown"

                # Only increment Bezier index if not completed
                if not self.legs[leg].bezier_curve_completed():
                    self.legs[leg].current_bezier_index += 1


                # Retain previous angles for legs not currently active
            for leg in self.legs:
                if leg not in active_legs:
                    self.rad_angles[leg] = self.rad_angles.get(leg, self.legs[leg].get_angles())  
                    self.leg_angles[leg] = self.leg_angles.get(leg, self.legs[leg].get_angles())

        # Ensure ALL Active Legs Have Reached "sliding" Before Advancing the Gait to next group
        if all(self.leg_states[leg] == "sliding" for leg in self.get_active_legs_in_gait()):
            gait_cycle_length = max(self.phase_offsets[self.gait].values()) + 1
            self.step_count = (self.step_count + 1) % gait_cycle_length
        return self.rad_angles


    
    def forward_kinematics(self):
        # For each leg calculate the forward kinematics and return
        positions = {}
        for i, name in enumerate (self.legs):
            if name == "LR":
                positions[name] = self.legs[name].forward_kinematics()
            return positions

    def get_rads(self):
        return self.rad_angles
    
    def get_angles(self):
        return self.leg_angles

    
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
    # theta1, theta2, theta3 = hexapod.move_leg("LR", [10,10,-10])

    # theta1 = np.degrees(theta1)
    # theta2 = np.degrees(theta2)
    # theta3 = np.degrees(theta3)

    # print(theta1, theta2, theta3)

    # hexapod.move_joint("LR", "coxa",theta1)
    # hexapod.move_joint("LR", "femur",theta2)
    # hexapod.move_joint("LR", "tibia",theta3)

    # for i in range(0, 90):
    #     hexapod.move_joint("LR", "tibia", i)

    # for i in range(0, 160):
    #     hexapod.move_joint("LR", "femur", i)
    
    # for i in range(160, 0, -1):
    #     hexapod.move_joint("LR", "femur", i)

    # for i in range(90, 0, -1):
    #     hexapod.move_joint("LR", "tibia", i)
    # time.sleep(1)
    # time.sleep(2)
    # for i in range(0, 65):
    #     hexapod.move_joint("LR", "coxa", i)
    
    # time.sleep(1)
    # for i in range(65, -35, -1):
    #     hexapod.move_joint("LR", "coxa", i)

    # time.sleep(1)
    # for i in range(-35, 0):
    #     hexapod.move_joint("LR", "coxa", i)