import copy
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Dict
from src.coord import (Coordinate, SegmentLengths, ServoAngles, new_Coordinate, homogeneous_transform_around_center, apply_transform_to_point, 
                   get_transformation_from_matrix, homogeneous_transformation_matrix,
                   get_radial_direction, adjust_point_away_from_coxa, rotate_bezier_curve,
                    translate_point_along_leg_direction, rot_tran_3d_x, rot_tran_3d_y, rot_tran_3d_z, new_servo_angles)
from src.bezier2d import BezierCurve
from src.servo import Servo
from src.inversekinematics import solve_effector_IK
from src.config import POD_Z_HEIGHT, NUM_JOINTS, COXA_ORIGIN_INDEX, FEMUR_ORIGIN_INDEX, TIBIA_ORIGIN_INDEX, EFFECTOR_ORIGIN_INDEX, INTERPOLATION_STEPS



@dataclass 
class intermediateAngles:
    num_joints: int = NUM_JOINTS #should make 3 joints, coxa, femur, tibia
    steps:int = INTERPOLATION_STEPS
    jointAngle: np.ndarray = field(init=False)# = field(default_factory=lambda: np.zeros((num_joints - 1, INTERPOLATION_STEPS), dtype=np.float64))

    def __post_init__(self):
        self.jointAngle = np.zeros((self.steps, self.num_joints), dtype=np.float64)

@dataclass
class intermediateeffectorCoordinates:
    steps: int = INTERPOLATION_STEPS
    coordinates: List[Coordinate] = field(default_factory=lambda: [Coordinate() for _ in range(INTERPOLATION_STEPS)])

@dataclass
class Leg:
    '''Represents a single robotic leg and it's kinematic state'''
    Index: int                                      # leg index (0-Number of legs-1)
    Name:  str                                      # Identify which leg by position
    ###
    ## Setup Servo Motors ||| UNCOMMENT ON HEXAPOD |||
    ###
    Coxa: Servo
    Femur: Servo
    Tibia: Servo
    coxa_angle_offset: float                        # Angle of leg around hexapod body
    coxa_position: Coordinate                       # Positions of coxa from the hexapod center.
    offset_transformation_matrix: np.ndarray        # 4x4 Transform for coxa
    segment_length: SegmentLengths                  # Lengths of coxa, femur, tibia
    servo_angles: ServoAngles                       # Current state of servo angles
    neutral_effector_coord: Coordinate              # Start/rest position of end effector
    num_joints: int = NUM_JOINTS                           # Number of joints each leg has (coxa, femur, tibia, end effector)
    Joints: List[Coordinate] = field(init=False)    # Location of the reference frame origin for each joint
    effector_target: 'Coordinate' = field(default_factory=lambda: Coordinate)  # Unique instance per leg
    intermediate_angles: "intermediateAngles" = field(default_factory=intermediateAngles)
    intermediate_effector_coordinates:'intermediateeffectorCoordinates' = field(default_factory=lambda: intermediateeffectorCoordinates)
    swinginterpolationIndex: int = 0                # Index for current idx of the interpolation table for the leg while in swing phase
    revertinterpolationIndex: int = 0               # current index in revert-to-neutral interpolation
    stanceinterpolationIndex: float = 0.0           # current index in stance-phase interpolation
    sliding: bool = False                           # flag indicating if leg is moving back to neutral position
    Debug: bool = False                             # Set debug mode for printing errors
    bezier_curve: "BezierCurve" = field(init=False) # Bezier curve which is used for walking
    transition_curve: "BezierCurve" = field(init=False) # Bezier curve which is used for walking
    current_leg_phase: str = "initializing"         # initializingneutral, swinging,
    t: float = 0.0                                  # Progress along the full bezier curve [0,1]
    control_points: Dict[str, np.ndarray] = field(init=False)
    # For Real Time
    toe_from_coxa:int = 70
    # For simulation
    # toe_from_coxa = 150 # distance to place bezier curve away fro the hexapod coxa
    step_idx:int = 0
    current_phase:bool = True
    duty_cycle: float = 0.75 # % of time leg is in "Stance"/"swinging"
    
    def __post_init__(self):
        ''' This post init works for Bezier Curve left rear leg'''
        from src.inversekinematics import solve_effector_IK
        self.Joints = [Coordinate() for _ in range(self.num_joints)] # Setup joint positions 
        # Temporary Delete for pod class
        #print("Joints after Init : ", self.Joints)
        # print(f"Joints before Forward Kinematics: {self.Name}, {self.Joints}")
        # Initialize all joints to 0,0,0 initially

        self.recalculate_forward_kinematics(ServoAngles(0, 0, 0))  # Optional: Update Joints
        temp = solve_effector_IK(self, self.neutral_effector_coord)
        self.recalculate_forward_kinematics(temp)
        self.set_initialization_control_points()

        # self.set_walking_control_points()
        # point = self.bezier_curve.get_point(0)
        # point = new_Coordinate(point[0], point[1], point[2])
        # coord = solve_effector_IK(self, point)
        
        # print(f"Joints After Forward Kinematics: {self.Name}, {self.Joints}")
        # set effector target based off standing position of neutral effector
        # self.effector_target = copy.deepcopy(self.neutral_effector_coord) # Start at neutral
        # print(f"Effector_Target: {self.effector_target}")
        # Solve for IK for current coordinate of standing
        #self.servo_angles = solve_effector_IK(self, new_Coordinate(0,0,0))  # Set angles
        # print("Joints after IK : ", self.Joints)
        # print(f'Servo angles: {self.servo_angles}')
        self.servo_indexes = [self.Index * 3, self.Index * 3 + 1, self.Index * 3 + 2]
        #self.recalculate_forward_kinematics(self.servo_angles)  # Optional: Update Joints
        # print("Joints after FK : ", self.Joints)
        #print("------------------------------------")
    
    def set_initialization_control_points(self):
        #All points start just before max distance at an angle, then moves down to standing
        start_pos = np.array([copy.deepcopy(self.neutral_effector_coord.X),copy.deepcopy(self.neutral_effector_coord.Y), copy.deepcopy(self.neutral_effector_coord.Z)])
        coxa_pos = np.array([self.coxa_position.X, self.coxa_position.Y, self.coxa_position.Z])

        # Get translated standing position point.
        from_coxa = coxa_pos + [0,0, POD_Z_HEIGHT]
        # NOTE: Ensure the legs don't hit the middle legs!
        if self.Name == "LR":
            radial_dir = get_radial_direction(coxa_pos, -10) # get direction of the coxa
        elif self.Name == "RR":
            radial_dir = get_radial_direction(coxa_pos, 10)
        else:
            radial_dir = get_radial_direction(coxa_pos)

        neutral_start = adjust_point_away_from_coxa(from_coxa, radial_dir, self.toe_from_coxa)

        # Use the current position, and move to the intended position of each control point (same as walking start point)
        self.neutral_effector_coord = Coordinate(start_pos[0], start_pos[1], start_pos[2])
        
        self.control_points = {
            "start": start_pos,
            "standing": neutral_start
        }
        self.bezier_curve = BezierCurve(self.control_points, num_pts=100)
        if self.Name == "LR":
            print(self.bezier_curve.curve())
        
    def set_walking_control_points(self, stride_length:float = 25.0):
        ''' Setup the control points for a full gait cycle '''
        
        # Setup Control Points
        # Get neutral affector coordinates.
        coxa_pos = np.array([self.coxa_position.X, self.coxa_position.Y, self.coxa_position.Z])

        ####
        # Rotate each bezier curve around the hexapod to fit each leg
        ####
        # Start position is directly below the coxa
        # [0,0,-X], [coxa.x, coxa.y,0]

        # Use the current leg position (should be from standing)
        # start_pos = np.array([self.effector_target.X, self.effector_target.Y, self.effector_target.Z])
        from_coxa = coxa_pos + [0,0, POD_Z_HEIGHT]
        # print(f'Leg start position: {start_pos}')

        # Provide angle offset from coxa for rear legs
        # NOTE: Ensure the legs don't hit the middle legs!
        if self.Name == "LR":
            radial_dir = get_radial_direction(from_coxa, -10) # get direction of the coxa
        elif self.Name == "RR":
            radial_dir = get_radial_direction(from_coxa, 10)
        else:
            radial_dir = get_radial_direction(from_coxa)
        # get the translated start position for each individual leg based off the coxa coord & offset
        
        # NOTE: If needed each leg may need a further distance from coxa to the end effector
        # Do not change "neutral_effector_cord", but offset it using self.toe_coxa, and radial direction if needed
        translated_start = adjust_point_away_from_coxa(from_coxa, radial_dir, 50)
        #print(f"Translated_start: {translated_start}")
        
        # Get the adjusted curves.
        translated_control_points = self.get_adjusted_bezier_control_points(translated_start)
        
        self.bezier_curve = BezierCurve(translated_control_points, num_pts=100)
        self.transition_curve = BezierCurve(translated_control_points, num_pts=100)
        #print(f"Translated_Bezzier: {self.bezier_curve.curve()}")


        # for interpolation. Future implementation addtion for more capabilties
        # theta = np.pi/2
        # r = np.sqrt(self.neutral_effector_coord.X**2 + self.neutral_effector_coord.Y**2)
        # psi = np.arctan2(self.neutral_effector_coord.Y, self.neutral_effector_coord.X)
        # start = np.array([r* np.cos(psi), r*np.sin(psi), self.neutral_effector_coord.Z ])
        # touchdown = np.array([r * np.cos(psi+theta), r*np.sin(psi+theta), self.neutral_effector_coord.Z ])
        # slide_angle = psi + theta + np.pi/12 #extra 15 degrees for slide push
        # slide_radius = r * 1.1 # 10% outward push for torque

        # # Control points for rotation around Z axis, during sliding phase adjust slightly for shifting body
        # self.control_points = {
        #     "start": start,
        #     "lift": start + np.array([0, 0, 0]),
        #     "peak": (start + touchdown) / 2 + np.array([0, 0, 0]),
        #     "lower": touchdown + np.array([0, 0, 0]),
        #     "touchdown": touchdown,
        #     "grounded": touchdown + np.array([0, 0, 0]),  # Adjust z to ground
        #     "sliding": np.array([slide_radius * np.cos(slide_angle), slide_radius * np.sin(slide_angle), 0]),
        #     "return": start  # Back to starting radius, not angle
        # }


    def set_back_control_points(self):
        print("CONTROL POINTS NOT SET")
        coxa_pos = np.array([self.coxa_position.X, self.coxa_position.Y, self.coxa_position.Z])
        
        if self.Name == "FR":
            radial_dir = get_radial_direction(coxa_pos, -10) # get direction of the coxa
        elif self.Name == "FR":
            radial_dir = get_radial_direction(coxa_pos, 10)
        else:
            radial_dir = get_radial_direction(coxa_pos)
        
        # TODO: NOT FINISHED
        exit()
        return

    # Define control points relative to start position
    def get_adjusted_bezier_control_points(self, start_pos: np.ndarray) -> dict:
        ''' Define control points for a Bézier curve. Walking forward motion
            NOTE: Bezier curve is negative for real time and Positive for Simulation 
        '''
        self.control_points = {
            "start": start_pos,
            "lift": start_pos + np.array([0, 10, -70]),
            "peak": start_pos + np.array([0, 50, -150]),
            "lower": start_pos + np.array([0, 75, -70]),
            "touchdown": start_pos + np.array([0, 75, 0]),
            "grounded": start_pos + np.array([0, 75, 0]),
            "sliding": start_pos + np.array([0, 75, 0]),
            "return": start_pos,
        }
        return self.control_points

    def set_rotation_control_points(self, theta:float = np.pi/2):
        ''' set control points for rotation'''
        r = np.sqrt(self.neutral_effector_coord.X**2 + self.neutral_effector_coord.Y**2)
        psi = np.arctan2(self.neutral_effector_coord.Y, self.neutral_effector_coord.X)
        start = np.array([r* np.cos(psi), r*np.sin(psi), self.neutral_effector_coord.Z ])
        touchdown = np.array([r * np.cos(psi+theta), r*np.sin(psi+theta), self.neutral_effector_coord.Z ])
        slide_angle = psi + theta + np.pi/12 #extra 15 degrees for slide push
        slide_radius = r * 1.1 # 10% outward push for torque

        # Control points for rotation around Z axis, during sliding phase adjust slightly for shifting body
        self.control_points = {
            "start": start,
            "lift": start + np.array([0, 0, 25]),
            "peak": (start + touchdown) / 2 + np.array([0, 0, 75]),
            "lower": touchdown + np.array([0, 0, 25]),
            "touchdown": touchdown,
            "grounded": touchdown + np.array([0, 0, -20]),  # Adjust z to ground
            "sliding": np.array([slide_radius * np.cos(slide_angle), slide_radius * np.sin(slide_angle), self.neutral_effector_coord.Z]),
            "return": start  # Back to starting radius, not angle
        }
        self.bezier_curve = BezierCurve(self.control_points)


    def set_reset_control_points(self):
        ''' set control points for reset/standing. '''
        current = np.array([self.effector_target.X, self.effector_target.Y, self.effector_target.Z])
        neutral = np.array([self.neutral_effector_coord.X, self.neutral_effector_coord.Y, self.neutral_effector_coord.Z])
        self.control_points = {
            "start": current,
            "lift": current + np.array([0, 0, 25]), 
            "peak": (current + neutral) /2 + np.array([0, 0, 25]),
            "touchdown": neutral
        }
        self.bezier_curve = BezierCurve(self.control_points)
        self.current_leg_phase = "reset"

    def reset_interpolator(self):
        '''
            Reset both swing and stance interpolation indices.
            # Verify that this needs -1 or not
        '''
        self.swinginterpolationIndex = (INTERPOLATION_STEPS-1) / 2
        self.stanceinterpolationIndex = (INTERPOLATION_STEPS-1) / 2
    
    

    def set_joint_angles(self, joint, angle) -> None:
        '''
            Set the angles of a specific joint in the leg
        '''
        if joint in self.servos:
            self.servos[joint].set_angle(int(angle))
        else:
            raise ValueError(f"Invalid joint: {joint}")                      

    def Zero(self) -> None:
        '''
            Zero out the servo angles & recalculate FK
        '''
        self.servo_angles.Coxa = 0
        self.servo_angles.Femur = 0
        self.servo_angles.Tibia = 0
        self.recalculate_forward_kinematics(self.servo_angles)

    def ground(self, height):
        '''
            Ground the legs at a certain height & recalculate FK
        '''
        self.servo_angles = solve_effector_IK(self, new_Coordinate(self.Joints[EFFECTOR_ORIGIN_INDEX].X,
                                                                   self.Joints[EFFECTOR_ORIGIN_INDEX].Y,
                                                                   height))
        self.recalculate_forward_kinematics(self.servo_angles)
        
        
    def get_joint_origin(self, H: np.ndarray) -> Coordinate:
        ''' Helper to extract the origin (translation part) from a homogeneous transformation matrix. '''
        # The origin is the last column (except the homogeneous 1) of the matrix.
        return Coordinate(X=float(H[0,3]), Y=float(H[1,3]), Z=float(H[2,3]))

    def recalculate_forward_kinematics(self, angles:ServoAngles) -> None:
        '''
            Set the leg angles to the angles aquired from inverse_kinematics
            Aquire positions of all the joints
        '''
        self.servo_angles = angles

        p_coxa = np.array([
                        [1, 0, 0], #rotate coxa around Z axis
                        [0, 0, -1], # aligns coxa to body base plane
                        [0, 1, 0]])
        p_femur = np.eye(3) # identity (no motion)
        p_tibia = np.eye(3) # identity (no motion)

        # Compute matrixes. Converts angle degrees to radians
        h_coxa = homogeneous_transformation_matrix(p_coxa, np.deg2rad(angles.Coxa), self.segment_length.Coxa)
        h_femur = homogeneous_transformation_matrix(p_femur, np.deg2rad(angles.Femur), self.segment_length.Femur)
        h_tibia = homogeneous_transformation_matrix(p_tibia, np.deg2rad(angles.Tibia), self.segment_length.Tibia)

        # Multiply matrices to get cumulative transforms
        h0_1 = self.offset_transformation_matrix.dot(h_coxa) # coxa base to femur
        h1_2 = h0_1.dot(h_femur)                             # femur to Tibia base
        h2_3 = h1_2.dot(h_tibia)                             # tibia to end efffector

        # Extract joint origins in base frame
        self.Joints[COXA_ORIGIN_INDEX] = self.get_joint_origin(self.offset_transformation_matrix)
        self.Joints[FEMUR_ORIGIN_INDEX] = self.get_joint_origin(h0_1)
        self.Joints[TIBIA_ORIGIN_INDEX] = self.get_joint_origin(h1_2)
        self.Joints[EFFECTOR_ORIGIN_INDEX] = self.get_joint_origin(h2_3)
        
        # set current target position to end effector origin
        self.effector_target = self.Joints[EFFECTOR_ORIGIN_INDEX]



    def update_bezier_curve(self, step_height = 50, forward_distance = 10):
        ''' Updates the bezier curve points dynamically
            step_height: Height the leg should lift during a step
            forward_distance: distance the leg moves forward
        '''
        # Get current effector position
        P0 = np.aray([self.effector_target.X, self.effector_target.Y, self.effector_target.Z])

        # forward movement Target
        P3 = np.array([
            P0[0] + forward_distance, # move forward
            P0[1], # stay in same Y-plane
            P0[2]
        ])

        # Ensure target is reachable(clip within leg_length)
        max_reach = self.segment_length.Coxa + self.segment_length.Femur + self.segment_length.Tibia
        if np.linalg.norm(P3[:2]) > max_reach:
            P3[:2] = (P3[:2] / np.linalg.norm(P3[:2])) * max_reach

        # Lift points
        P1 = np.array([P0[0], P0[1], P0[2] + step_height])  # Lift phase
        P2 = np.array([P3[0], P3[1], P3[2] + step_height])  # Lowering phase

        # Update the Bezier curve
        self.bezier_curve = BezierCurve({
            "start": P0, "lift": P1, "peak": P2, "end": P3
        }, num_pts=100)


    def move_leg_with_bezier(self, step_count=100):
        '''
        Moves a leg along its Bezier curve using inverse kinematics.
        This is a test function
        '''
        from src.inversekinematics import solve_effector_IK
        import time
        bezier_points = self.bezier_curve.curve()

        for i in range(step_count): 
            effectorTarget = Coordinate(bezier_points[i][0], bezier_points[i][1], bezier_points[i][2])
            # print(effectorTarget)
            # Ensure the target is within the leg's reachable workspace
            max_reach = self.segment_length.Coxa + self.segment_length.Femur + self.segment_length.Tibia
            if np.linalg.norm([effectorTarget.X, effectorTarget.Y]) > max_reach:
                effectorTarget.X = (effectorTarget.X / np.linalg.norm([effectorTarget.X, effectorTarget.Y])) * max_reach
                effectorTarget.Y = (effectorTarget.Y / np.linalg.norm([effectorTarget.X, effectorTarget.Y])) * max_reach

            # Solve inverse kinematics
            angles = solve_effector_IK(self, effectorTarget)
            self.recalculate_forward_kinematics(angles)
            self.servo_angles = angles
            # Send angles to 
            # print(f"Servo Angles: Coxa{angles.Coxa}, Femur {angles.Femur}, Tibia {angles.Tibia}")
            self.Coxa.set_angle(angles.Coxa)
            self.Femur.set_angle(angles.Femur)
            self.Tibia.set_angle(angles.Tibia)

    def update_swing(self, direction):
        ''' 
            Moves the interpolation index to next element in the table for swing phase until it reaches end
            & wraps back to 0.
            1: end of swing
            0: Interpolate the swing
        '''
        angles = new_servo_angles(self.intermediate_angles.jointAngle[self.swinginterpolationIndex][0], self.intermediate_angles.jointAngle[self.swinginterpolationIndex][1], self.intermediate_angles.jointAngle[self.swinginterpolationIndex][2])
        self.servo_angles = angles
        self.recalculate_forward_kinematics(angles)

        #phase step/ lift legs in swing so modify z, when in swing find a new soln where leg is not touching ground
        phase_step = np.pi/(INTERPOLATION_STEPS-1)
        z = self.Joints[EFFECTOR_ORIGIN_INDEX].Z

        ## Swing from 0 to pi
        phase = phase_step * float(self.swinginterpolationIndex)
        zNew = z * np.sin(phase)
        
        new_angle = solve_effector_IK(self, new_Coordinate(self.Joints[EFFECTOR_ORIGIN_INDEX].X, self.Joints[EFFECTOR_ORIGIN_INDEX].Y, z-zNew))
        self.recalculate_forward_kinematics(new_angle)  # Update Joints with new angles

        self.swinginterpolationIndex += int(direction)

        if direction == 1:
            if self.swinginterpolationIndex >= INTERPOLATION_STEPS:
                self.swinginterpolationIndex = 0
                self.stanceinterpolationIndex = INTERPOLATION_STEPS
        elif direction == -1:
            if self.swinginterpolationIndex <= 0:
                self.swinginterpolationIndex =  INTERPOLATION_STEPS
                self.stanceinterpolationIndex = 0

    def update_stance(self, direction:int, returnFactor:float):
        '''
            Moves index to the previous element in the table till it reaches start. 
            Loops back to end and counts down again
        '''
        angles = new_servo_angles(self.intermediate_angles.jointAngle[self.swinginterpolationIndex][0], self.intermediate_angles.jointAngle[self.swinginterpolationIndex][1], self.intermediate_angles.jointAngle[self.swinginterpolationIndex][2])
        self.servo_angles = angles
        self.recalculate_forward_kinematics(angles)
        
        self.stanceinterpolationIndex -= direction * returnFactor
        
        ## each leg has its own table
        if direction == 1:
            if self.stanceinterpolationIndex <= 0:
                self.stanceinterpolationIndex = INTERPOLATION_STEPS
        elif direction == -1:
            if self.stanceinterpolationIndex >= INTERPOLATION_STEPS:
                self.stanceinterpolationIndex = 0
                self.swinginterpolationIndex = INTERPOLATION_STEPS



    def update(self, delta_t: float, direction: int, gait_pattern: List[int], gait_index: int) -> bool:
        ''' Test update leg: improved functionality moved to pod class '''
        from src.inversekinematics import solve_effector_IK
        # if if neutral or rotation change the control points to gait

        # set gait index
        is_swinging = gait_pattern[gait_index] == 1
        max_t = self.duty_cycle if is_swinging else 1.0 # self.duty_cycle 5/8 if "swinging" else 1.0 speed
    
        # set current index between [0,1]
        self.t = max(0.0, min(max_t, self.t + delta_t * direction))
        
        # Evaluate the position at time t
        pos = self.bezier_curve.evaluate(self.t)
        
        # set the target for the toe from the coordinates
        self.effector_target = Coordinate(pos[0], pos[1], pos[2])

        # compute the inverse kinetmatics of the new target
        angles = solve_effector_IK(self, self.effector_target)

        # set the servo angles
        self.servo_angles = angles
        
        # if the leg has not completed the full bezier curve
        if self.t < max_t:
            return True  # Still moving
        
        # else don't move leg
        self.t = 0.0 if is_swinging else self.duty_cycle  # Reset or hold
        return False

    @property
    def currentlegPhase(self):
        return self.current_leg_phase
    
    @currentlegPhase.setter
    def currentlegPhase(self, new_phase:str = None):
        if new_phase != None:
            self.current_leg_phase = new_phase

    def update_rotation(self, delta_t: float, direction: int) -> bool:
        ''' Advance rotation phase '''
        from inversekinematics import solve_effector_IK

        # set the current phase to rotation bezier curve
        if self.current_leg_phase != "rotation":
            self.set_rotation_control_points()

        # set current time index, target, and invese kinematics
        self.t = max(0.0, min(1.0, self.t + delta_t * direction))
        if self.t < 1.0:
            pos = self.bezier_curve.evaluate(self.t)
            self.effector_target = Coordinate(pos[0], pos[1], pos[2])
            angles = solve_effector_IK(self, self.effector_target)
            self.servo_angles = angles
            return True
        self.t = 0.0
        return False

    def reset_to_neutral(self, delta_t: float) -> bool:
        ''' Reset leg to the neutral standing position '''
        from inversekinematics import solve_effector_IK

        # set new bezier curve to set legs to the reset/neutral position 
        if self.current_leg_phase != "reset":
            self.set_reset_control_points()
        
        # increase t till end of the movement
        self.t += delta_t

        # if t is less than 1.0 or the max of new bezier curve
        if self.t <= 1.0:
            pos = self.bezier_curve.evaluate(self.t)
            self.effector_target = Coordinate(pos) 
            angles = solve_effector_IK(self, self.effector_target)
            self.servo_angles = angles
            return True
        self.t = 0.0
        return False

    def set_debug(self, debug:bool) -> None:
        self.Debug = debug

    def toNeutral(self):
        targetCoord = self.neutral_effector_coord
        startCoord = self.Joints[EFFECTOR_ORIGIN_INDEX]
        
        stepX = (targetCoord.X - startCoord.X) / (INTERPOLATION_STEPS)
        stepY = (targetCoord.Y - startCoord.Y) / (INTERPOLATION_STEPS)
        stepZ = (targetCoord.Z - startCoord.Z) / (INTERPOLATION_STEPS)

        for i in range(INTERPOLATION_STEPS):
            swing = new_Coordinate(startCoord.X + stepX * i, startCoord.Y + stepY * i, startCoord.Z + stepZ * i)
            angles = solve_effector_IK(self, swing)
            
            self.intermediate_angles.jointAngle[i][0] = angles.Coxa
            self.intermediate_angles.jointAngle[i][0] = angles.Femur
            self.intermediate_angles.jointAngle[i][0] = angles.Tibia







## -----------------------------------------------------
##  Test Cases for new leg class without bezier curve
## -----------------------------------------------------
def new_legs(): #test code to add 6 new legs

    # Create the legs with servos
    legs = [
        Leg(
            Index=i,
            Name=f"Leg {i}",
            #Setup Servo Motors ||| UNCOMMENT ON HEXAPOD |||
            Coxa=Servo(i * 3, pca=0x40 if i * 3 < 8 else 0x41),  # Assign PCA based on ID
            Femur=Servo(i * 3 + 1, pca=0x40 if i * 3 + 1 < 8 else 0x41),
            Tibia=Servo(i * 3 + 2, pca=0x40 if i * 3 + 2 < 8 else 0x41),
            coxa_angle_offset=45.0,
            offset_transformation_matrix=np.eye(4), # Creates 4x4 identity matrix for an insert
            segment_length=SegmentLengths(45, 110, 193),
            servo_angles=ServoAngles(),  # Provide default servo angles
            neutral_effector_coord=Coordinate(0, 0, 0)
        )
        for i in range(6)  # 6 legs
    ]

    # Print assigned servo indexes for each leg || Works on Raspi4
    for leg in legs:
       print(f"{leg.Name}: Coxa Servo {leg.Coxa.servo_index}, Femur Servo {leg.Femur.servo_index}, Tibia Servo {leg.Tibia.servo_index}")

    return legs


def new_leg():
    coxa_offset = 214.309
    coxaCoordinate = Coordinate(-60.5, -89, 0)

    offset_matrix = np.array([
                [np.cos(coxa_offset * np.pi/180), -np.sin(coxa_offset*np.pi/180), 0, coxaCoordinate.X],  # x translation
                [np.sin(coxa_offset*np.pi/180), np.cos(coxa_offset*np.pi/180), 0, coxaCoordinate.Y],    # y translation
                [0, 0, 1, coxaCoordinate.Z*np.pi/360],      # z translation (none)
                [0, 0, 0, 1]       # homogeneous row
                ])
    
    leg = Leg(
            Index=1,
            Name=f"Leg {1}",
            #Setup Servo Motors ||| UNCOMMENT ON HEXAPOD |||
            Coxa=Servo(0, pca= 0x40, offset=0),  # Assign PCA based on ID
            Femur=Servo(1, pca= 0x40, offset=-120),
            Tibia=Servo(2, pca= 0x40, offset= 0),
            coxa_angle_offset=coxa_offset,
            offset_transformation_matrix=offset_matrix, # Creates 4x4 identity matrix for an insert
            segment_length=SegmentLengths(45, 110, 193),
            servo_angles=ServoAngles(0,0,0),  # Provide default servo angles
        )

    #print(leg)
    return leg

def test_single_leg_movement(leg):
    """ Test function to manually move each leg's servos using predefined angles. """
    import time
    #print("Starting leg movement test...")

    # Move Coxa to 0 degrees for each leg
    #print(l
    # eg.Coxa)
    leg.Coxa.set_angle(0)  # Move coxa servo to neutral position
    time.sleep(.02)

    time.sleep(1)

    # Move Femur to -120 degrees (lifting the leg)
    #print(leg.Femur)
    leg.Femur.set_angle(-120)  # Lift femur joint
    time.sleep(.02)

    time.sleep(1)

    # Move tibia
    # print(leg.Tibia)
    leg.Tibia.set_angle(-50)  # Reset tibia joint
    time.sleep(.02)

    time.sleep(1)
    # Move Tibia back to 0 degrees
    # print(leg.Tibia)
    leg.Tibia.set_angle(0)  # Reset tibia joint
    time.sleep(.02)

    time.sleep(1)

    # Move Coxa back to neutral (optional reset)

    leg.Coxa.set_angle(0)
    leg.Femur.set_angle(0)
    leg.Tibia.set_angle(0)
    time.sleep(.02)
    # print("Leg movement test completed!")


def test_legs_movement(legs):
    """Test function to manually move each leg's servos using predefined angles."""
    import time
    # print("Starting leg movement test...")
    # Move Coxa to 0 degrees for each leg
    for leg in legs:
        # print(leg.Coxa)
        leg.Coxa.set_angle(0)  # Move coxa servo to neutral position
        time.sleep(.02)

    time.sleep(1)

    # Move Femur to -120 degrees (lifting the leg)
    for leg in legs:
        leg.Femur.set_angle(-120)  # Lift femur joint
        time.sleep(.02)

    time.sleep(1)

    # Move Tibia back to 0 degrees
    for leg in legs:
        leg.Tibia.set_angle(0)  # Reset tibia joint
        time.sleep(.02)

    time.sleep(1)

    # Move Coxa back to neutral (optional reset)
    for leg in legs:
        leg.Coxa.set_angle(0)
        leg.Femur.set_angle(0)
        leg.Tibia.set_angle(0)
        time.sleep(.02)

    # print("Leg movement test completed!")


def test_bezier_curve_movement(leg) -> None:
    # print(leg.bezier_curve.control_points_dict)
    # print (leg.bezier_curve.index_map)
    # print(leg.bezier_curve.curve())
    from inversekinematics import solve_effector_IK
    import time
    if leg.current_leg_phase != "gait":
        leg.set_walking_control_points()

    max_t = 100
    delta_t = 0.01
    direction = 1
    # leg.t = max(0.0, min(max_t, leg.t + delta_t * direction))
    #pos = leg.bezier_curve.evaluate(leg.t)
    leg.t = leg.bezier_curve.index

    # Evaluate the position at time t
    pos = leg.bezier_curve.get_point(leg.t)

    # Lock.t into 1 index for bezier curve  
    leg.bezier_curve.index += 1

    # set the target for the toe from the coordinates
    #print(f"Bezier Curve position at t {leg.t}: {pos}")
    
    leg.effector_target = Coordinate(pos[0], pos[1], pos[2])
    #print(f"leg.effector_target {leg.effector_target}")

    # Update Joints with current angles before IK
    # leg.recalculate_forward_kinematics(leg.servo_angles)
    angles = solve_effector_IK(leg, leg.effector_target)
    leg.Coxa.set_angle(leg.servo_angles.Coxa)
    # time.sleep(.02)
    leg.Femur.set_angle(leg.servo_angles.Femur)
    # time.sleep(.02)
    leg.Tibia.set_angle(leg.servo_angles.Tibia)
    # time.sleep(.02)

    leg.servo_angles = angles
    leg.recalculate_forward_kinematics(angles)  # Update Joints with new angles
    #print("Forward Kinematics of Joints after IK & FK: ", leg.Joints)
    #print('-----------------------------------')

    if leg.t >= max_t:
        leg.t = 0
        leg.bezier_curve.index = 0

def test_stride_vector_movement(leg, nrepeats, x, y, z = 50.0): # n_repeates:int, x:float, y:float):
    import time
    effector_origin = leg.Joints[EFFECTOR_ORIGIN_INDEX]
    xMax = effector_origin.X + x
    xMin = effector_origin.X - x
    yMax = effector_origin.Y + y
    yMin = effector_origin.Y - y

    # For visualization
    xstep = (xMax - xMin) / (INTERPOLATION_STEPS - 1)
    ystep = (yMax - yMin) / (INTERPOLATION_STEPS - 1)
    deltaX = 0
    deltaY = 0

    # Calculate coordinates
    for i in range(INTERPOLATION_STEPS):
        swing = new_Coordinate(xMin + deltaX, yMin + deltaY, z)
        servoAngles = solve_effector_IK(leg, swing)
        #revise below
        leg.intermediate_angles.jointAngle[i][0] = servoAngles.Coxa
        leg.intermediate_angles.jointAngle[i][1] = servoAngles.Femur
        leg.intermediate_angles.jointAngle[i][2] = servoAngles.Tibia
        leg.intermediate_effector_coordinates = new_Coordinate(deltaX + xMin, deltaY + yMin, 0)
        deltaX += xstep
        deltaY += ystep

    # moving forward
    direction = 1 # if 1 Forward, if -1 backwards
    
    for i in range(INTERPOLATION_STEPS):
        # Simulate update swing function direction forward
        angles = new_servo_angles(leg.intermediate_angles.jointAngle[leg.swinginterpolationIndex][0], leg.intermediate_angles.jointAngle[leg.swinginterpolationIndex][1], leg.intermediate_angles.jointAngle[leg.swinginterpolationIndex][2])
        leg.recalculate_forward_kinematics(angles)

        #phase step/ lift legs in swing so modify z, when in swing find a new soln where leg is not touching ground
        phase_step = np.pi/(INTERPOLATION_STEPS-1)
        z = leg.Joints[EFFECTOR_ORIGIN_INDEX].Z

        ## Swing from 0 to pi
        phase = phase_step * float(leg.swinginterpolationIndex)
        zNew = z * np.sin(phase)
        
        new_angle = solve_effector_IK(leg, new_Coordinate(leg.Joints[EFFECTOR_ORIGIN_INDEX].X, leg.Joints[EFFECTOR_ORIGIN_INDEX].Y, z-zNew))
        leg.recalculate_forward_kinematics(new_angle)  # Update Joints with new angles

        leg.swinginterpolationIndex += int(direction)


        if direction == 1:
            if leg.swinginterpolationIndex >= INTERPOLATION_STEPS:
                leg.swinginterpolationIndex = 0
                leg.stanceinterpolationIndex = INTERPOLATION_STEPS
        elif direction == -1:
            if leg.swinginterpolationIndex <= 0:
                leg.swinginterpolationIndex =  INTERPOLATION_STEPS
                leg.stanceinterpolationIndex = 0
        #print(leg.intermediate_angles.jointAngle)


        leg.Coxa.set_angle(new_angle.Coxa)
        time.sleep(.05)
        leg.Femur.set_angle(new_angle.Femur)
        time.sleep(.05)
        leg.Tibia.set_angle(new_angle.Tibia)
        time.sleep(.05)

if __name__ == "__main__":
    hexapod_leg = new_leg()
    
    repeats = 21
    x =20
    y = -20
    z = 50
    while True:
        # test_bezier_curve_movement(hexapod_leg)
        test_stride_vector_movement(hexapod_leg, repeats, x, y, z)
    #   # hexapod_leg.move_leg_with_bezier()
    # test again with bezier curve itself no inverse kinematics
    # hexapod_leg.move_leg_with_bezier()

    # Works on raspi4 
    # hexapod_leg = new_leg()
    # test_single_leg_movement(hexapod_leg)


    # hexapod_legs = new_legs()
    # test_legs_movement(hexapod_legs)