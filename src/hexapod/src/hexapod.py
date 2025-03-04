import time
import numpy as np
from threading import Lock
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
    
    def __init__(self, leg_configs, hexapod_configs, bezier_points):
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
        '''
        # Initialize basic variables
        self.max_speed = 1.0
        self.legs = {}
        self.mode = "tripod"
        
        self.divfactor = 0 # number of steps a leg is on the floor while walking
        self.multipler = 0 # multiplier for length of each step
        self.bezierpoints = bezier_points # bezier curve points that get rotated around for each leg

        # Body Position
        self.body_position = coord3D()
        self.body_rotation = coord3D()
        self.hexapod_height = 0 # How high the hexapod is from the ground

        for body_pose, config in hexapod_configs.items():
            self.body_pose = config["pose"]
            coxa_offsets = config["coxa_offsets"]
        
        self.body_coxa_offset = {}
        
        # setup hexapod body positions in 3D space
        self.position = coord3D()
        self.rotation = coord3D()
        
        # setup gaits
        self.tripod_gait_groups = [["LR", "RM", "LF"], ["RF","LM","RR"]]
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

            self.body_coxa_offset[leg_name] = coord2D()
            self.body_coxa_offset[leg_name].x = coxa_offsets[leg_name][0] # Distance x from center of body to coxa
            self.body_coxa_offset[leg_name].y = coxa_offsets[leg_name][1] # Distance y from center of body to coxa
            self.gait_pos[leg_name] = coord3D()
            
            
            leg_index[leg_name] = config["leg_index"]
            servo_pins[leg_name] = config["servos"]  # Extract servo pins
            segment_lengths[leg_name] = config["segment_lengths"]
            pulse_min[leg_name] = config["pulsemin"]
            pulse_max[leg_name] = config["pulsemax"]
            angleoffset[leg_name] = config["angleoffset"]
        
        
         # Initial leg positions [x,y,z] || Will need adjusting
        temp_vals = {
            "LR": {
                "x": -np.cos(np.deg2rad(56.172)) * (segment_lengths["LR"][0] + segment_lengths["LR"][1] - segment_lengths["LR"][2]),
                "y": -np.sin(np.deg2rad(56.172)) * (segment_lengths["LR"][0] + segment_lengths["LR"][1] - segment_lengths["LR"][2]),
                "z": segment_lengths["LR"][2]-30, # Adding -30 brings leg out away from hexapod
            },
            "LM": {
                "x": -(segment_lengths["LM"][0] + segment_lengths["LM"][1]),
                "y": 0,  # Middle leg should have zero Y if centered
                "z": segment_lengths["LM"][2],
            },
            "LF": {
                "x": -np.cos(np.deg2rad(55.931)) * (segment_lengths["LF"][0] + segment_lengths["LF"][1] - segment_lengths["LR"][2]),
                "y": np.sin(np.deg2rad(55.931)) * (segment_lengths["LF"][0] + segment_lengths["LF"][1]  - segment_lengths["LR"][2]),
                "z": segment_lengths["LF"][2],
            },
            "RF": {
                "x": np.cos(np.deg2rad(55.931)) * (segment_lengths["RF"][0] + segment_lengths["RF"][1]  - segment_lengths["LR"][2]),
                "y": np.sin(np.deg2rad(55.931)) * (segment_lengths["RF"][0] + segment_lengths["RF"][1]  - segment_lengths["LR"][2]),
                "z": segment_lengths["RF"][2],
            },
            "RM": {
                "x": (segment_lengths["RM"][0] + segment_lengths["RM"][1]),
                "y": 0,  # Middle leg should have zero Y if centered
                "z": segment_lengths["RM"][2],
            },
            "RR": {
                "x": np.cos(np.deg2rad(56.172)) * (segment_lengths["RR"][0] + segment_lengths["RR"][1]  - segment_lengths["LR"][2]),
                "y": -np.sin(np.deg2rad(56.172)) * (segment_lengths["RR"][0] + segment_lengths["RR"][1]  - segment_lengths["LR"][2]),
                "z": segment_lengths["RR"][2],
            },
            }
<<<<<<< HEAD
        
        leg_lengths = {}
        for leg,vals in temp_vals.items():
            leg_lengths[leg] = np.sqrt(vals['x']**2 + vals['y']**2 + vals['z']**2)
        #print(f"leg lengths: ",leg_lengths)
=======
        for name, values in temp_vals.items():
<<<<<<< Updated upstream
            leg_ofs[name] = coord3D(x=values['x'], y=values['y'], z=values['z'])
            self.legs[name] = Leg(name, leg_index[name], servo_pins[name], pulse_min[name], pulse_max[name], segment_lengths[name], leg_ofs[name], angleoffset[name])
=======
            if name == "LR":
                toe_offsets[name] = coord3D(x=values['x'], y=values['y'], z=values['z'])
                self.legs[name] = Leg(name, leg_index[name], servo_pins[name], pulse_min[name], pulse_max[name], segment_lengths[name], toe_offsets[name], coxa_offsets[name], angleoffset[name])
                #print(f"toe positions/offsets:", toe_offsets[name].x, toe_offsets[name].y, toe_offsets[name].z)
        
>>>>>>> Stashed changes
>>>>>>> 231e5a5 (Auto stash before checking out "HEAD")

        toe_offsets = {}
        for name, values in temp_vals.items():
            if name == "LR":
                toe_offsets[name] = coord3D(x=values['x'], y=values['y'], z=values['z'])
                self.legs[name] = Leg(name, leg_index[name], servo_pins[name], pulse_min[name], pulse_max[name], segment_lengths[name], toe_offsets[name], angleoffset[name])
                #print(f"toe positions/offsets:", toe_offsets[name].x, toe_offsets[name].y, toe_offsets[name].z)
        

    def set_bezier_points(self, points:dict):
        self.bezierpoints = points
        
    def get_legs(self):
        return self.legs
    
    def move_legs(self, theta:int):
        '''
            get angles from inverse kinematics
        '''
        for i, leg_id in enumerate(self.legs):
            self.legs[leg_id].set_joint_angles("coxa", theta[0])
            self.legs[leg_id].set_joint_angles("femur", theta[1])
            self.legs[leg_id].set_joint_angles("tibia", theta[2])

    def move_joint(self, leg, joint, amount):
        if leg in self.legs:
            self.legs[leg].set_joint_angles(joint, amount)
        else:
            raise ValueError(f"Invalid Leg: {leg}")

    def get_leg_positions(self):
        return self.forward_kinematics()
    
    def stand(self):
        self.set_mode("stand")

    def set_mode(self, mode):
        self.mode = mode

    def get_leg_lengths(self):
        legs = {}
        for i, leg_id in enumerate(self.legs):
            legs[leg_id] = self.legs[leg_id].get_leg_lengths()
        return legs

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

    def rotation_2d(self, theta):
        '''
            Accepts an angle, converts it to radians and returns
        '''
        theta = np.deg2rad(theta)
        return np.array([[np.cos(theta), -np.sin[theta]], [np.sin(theta), np.cos(theta)]])
    
    def rot_mat_3d_x(self, theta):
        '''
            Accepts an angle, converts it to radians and returns
        '''
        theta = np.deg2rad(theta)
        return np.array([[1, 0, 0],
                         [0, np.cos(theta), -np.sin(theta)],
                         [0, np.sin(theta), np.cos(theta)]])
    
    def rot_mat_3d_y(self, theta):
        '''
            Accepts an angle, converts it to radians and returns
        '''
        theta = np.deg2rad(theta)
        return np.array([[np.cos(theta), 0, np.sin(theta)],
                         [0, 1, 0],
                         [-np.sin(theta), 0, np.cos(theta)]])
    
    def rot_mat_3d_z(self, theta):
        '''
            Accepts an angle, converts it to radians and returns
        '''
        theta = np.deg2rad(theta)
        return np.array([[np.cos(theta), -np.sin(theta), 0],
                         [np.sin(theta), np.cos(theta), 0],
                         [0, 0, 1]])
    

    def inverse_kinematics(self, position):
        # call leg inverse kinematics and feed it
        footposition = coord3D()
        rotation = coord3D()
        body = coord3D()
<<<<<<< HEAD
        positions = []
        for leg in self.VALID_LEGS:
            # Do some body kinematics here, send values to legs to change from global to local frame
=======
<<<<<<< Updated upstream
        for leg in self.VALID_LEGS:
            # Do some body kinematics here, send values to legs to change from global to local frame
            
            # Setup position
            footposition.x = self.legs[leg].cur_pos.x + self.body_position.x + self.gait_pos[leg].x
            footposition.y = self.legs[leg].cur_pos.y + self.body_position.y + self.gait_pos[leg].y
            footposition.z = self.legs[leg].cur_pos.z + self.body_position.z + self.gait_pos[leg].z
            #print(footposition.x, footposition.y, footposition.z)
            # Rotation
            rotation.x = self.body_rotation.x
            rotation.y = self.body_rotation.y + self.gait_pos[leg].roty
            rotation.z = self.body_rotation.z
            
            # ------------------------
            # Body kinemtatics
            # ------------------------
=======
        rad_angles = {}
        for i,leg in enumerate(self.VALID_LEGS):
            # Do some body kinematics here, send values to legs to change from global to local frame
>>>>>>> 231e5a5 (Auto stash before checking out "HEAD")
            if leg == "LR": # Remove this for all legs, change for 3 legs test
                # Setup position
                footposition.x = self.legs[leg].cur_pos.x + self.body_position.x + position[0] # self.gait_pos[leg].x
                footposition.y = self.legs[leg].cur_pos.y + self.body_position.y + position[1] # self.gait_pos[leg].y
                footposition.z = self.legs[leg].cur_pos.z + self.body_position.z + position[2] # self.gait_pos[leg].z
                
                #print(f'footposition: ',footposition.x, footposition.y, footposition.z)
                # Rotation
                rotation.x = self.body_rotation.x
                rotation.y = self.body_rotation.y + self.gait_pos[leg].roty
                rotation.z = self.body_rotation.z
                
                # ------------------------
                # Body kinemtatics
                # ------------------------
                body.x = footposition.x + self.body_coxa_offset[leg].x
                body.y = footposition.y + self.body_coxa_offset[leg].y
                body.z = footposition.z
                
<<<<<<< HEAD
                #print(body.x, body.y, body.z)
=======
>>>>>>> 231e5a5 (Auto stash before checking out "HEAD")
                # calculate position corrections using rotation matrix
                # https://en.wikipedia.org/wiki/Rotation_matrix
                Rx = self.rot_mat_3d_x(rotation.x) # pitch
                Ry = self.rot_mat_3d_y(rotation.y) # yaw
                Rz = self.rot_mat_3d_z(rotation.z) # roll
                
                R = Rz @ Ry @ Rx
                '''[[1. 0. 0.] no rotation
                    [0. 1. 0.]
                    [0. 0. 1.]]            
                '''
                temp_pos = np.array([body.x, body.y, body.z]) #body position in space
                rotated_point = R @ temp_pos
<<<<<<< HEAD
=======
>>>>>>> Stashed changes
>>>>>>> 231e5a5 (Auto stash before checking out "HEAD")

                bodyikPosition = coord3D()
                bodyikPosition.from_array(rotated_point)
                
                #print("BodyikPosition: ", bodyikPosition.x, bodyikPosition.y, bodyikPosition.z)
        
            
<<<<<<< HEAD
                # Send values to legs for leg kinematics
                positions.append(self.legs[leg].inverse_kinematics(bodyikPosition, footposition))
=======
<<<<<<< Updated upstream
            # print(body.x, body.y, body.z)
            # calculate position corrections using rotation matrix
            bodyikPosition = coord3D()
>>>>>>> 231e5a5 (Auto stash before checking out "HEAD")
            
            return positions
    
    def forward_kinematics(self):
        # For each leg calculate the forward kinematics and return
        positions = {}
        for i, name in enumerate (self.legs):
            if name == "LR":
                positions[name] = self.legs[name].forward_kinematics()
            return positions

<<<<<<< HEAD
=======
            #print(bodyikPosition.x, bodyikPosition.y, bodyikPosition.z)
    
        
            # Send values to legs for leg kinematics
            self.legs[leg].inverse_kinematics(bodyikPosition, footposition)
            
            
            
        
    def update(self):
        # get current position of each leg, and the gait position of each leg add with current position
        # Do this for x,y,z positions
        
        '''
            rotio = hexapod rotation (rotio)
            RB_FeetPos = (posio)
        ''' 
        # initialize temp vars
        
        for legs in self.VALID_LEGS:
            # Get values from legs
            pass
        
    def calculate_gait(self):
        '''
            Calculate where each leg is during the gait and return x,y,z,rot
            Each leg has 3 positions that a single leg can be lifted to
        '''
        return
=======
                # Send values to legs for leg kinematics
                rad_angles[leg] = self.legs[leg].inverse_kinematics(bodyikPosition, footposition)
            
            return rad_angles
>>>>>>> Stashed changes
    
>>>>>>> 231e5a5 (Auto stash before checking out "HEAD")
    
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

    for i in range(0, 90):
        hexapod.move_joint("LR", "tibia", i)

    for i in range(0, 160):
        hexapod.move_joint("LR", "femur", i)
    
    for i in range(160, 0, -1):
        hexapod.move_joint("LR", "femur", i)

    for i in range(90, 0, -1):
        hexapod.move_joint("LR", "tibia", i)
    time.sleep(1)
    time.sleep(2)
    for i in range(0, 65):
        hexapod.move_joint("LR", "coxa", i)
    
    time.sleep(1)
    for i in range(65, -35, -1):
        hexapod.move_joint("LR", "coxa", i)

    time.sleep(1)
    for i in range(-35, 0):
        hexapod.move_joint("LR", "coxa", i)