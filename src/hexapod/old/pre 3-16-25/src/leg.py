
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Dict
from coord import *
from bezier2d import BezierCurve
from servo import Servo

NUM_JOINTS = 4
COXA_ORIGIN_INDEX = 0
FEMUR_ORIGIN_INDEX = 1
TIBIA_ORIGIN_INDEX = 2
EFFECTOR_ORIGIN_INDEX = 3


def homogeneous_transformation_matrix(projection_matrix: np.ndarray, theta: float, length: float) -> np.ndarray:
    """Create a homogeneous transformation matrix given a projection matrix, a rotation angle theta (rad), and a displacement length."""
    # Rotation matrix about Z-axis by theta
    R_z = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta),  0],
        [0,               0,                1]
    ])
    # Final rotation = R_z * projection_matrix (3x3)
    R = R_z.dot(projection_matrix)
    # Displacement vector for this segment in the rotated frame
    D = np.array([[length * np.cos(theta)], [length * np.sin(theta)], [0]])
    # Construct 4x4 homogeneous transformation matrix from R and D
    H = np.array([
        [R[0,0], R[0,1], R[0,2], D[0,0]],
        [R[1,0], R[1,1], R[1,2], D[1,0]],
        [R[2,0], R[2,1], R[2,2], D[2,0]],
        [0,      0,      0,      1     ]
    ])
    return H

@dataclass
class IntermediateAngles:
    """IntermediateAngles holds arrays of joint angles for interpolation steps (for Coxa, Femur, Tibia)."""
    # JointAngle[j][i] = angle of joint j at interpolation step i.
    JointAngle: List[List[float]] = field(default_factory=lambda: [[0.0]*100 for _ in range(4-1)])

@dataclass
class Leg:
    '''Represents a single robotic leg and it's kinematic state'''
    Index: int                                      # leg index (0-Number of legs-1)
    Name:  str                                      # Identify which leg by position
    # Setup Servo Motors ||| UNCOMMENT ON HEXAPOD |||
    # Coxa: Servo
    # Femur: Servo
    # Tibia: Servo
    # other vars
    coxa_angle_offset: float                        # Angle of leg around hexapod body
    offset_transformation_matrix: np.ndarray        # 4x4 Transform for coxa
    segment_length: SegmentLengths                 # Lengths of coxa, femur, tibia
    servo_angles: ServoAngles                       # Current state of servo angles
    neutral_effector_coord: Coordinate              # Start/rest position of end effector
    num_joints: int = NUM_JOINTS                           # Number of joints each leg has (coxa, femur, tibia, end effector)
    Joints: List[Coordinate] = field(init=False)    # Location of the reference frame origin for each joint
    effector_target: 'Coordinate' = field(default_factory=lambda: Coordinate())  # Unique instance per leg
    # intermediate_angles: 'IntermediateAngles' = field(default_factory=lambda: IntermediateAngles())  #
    # intermediate_effector_coordinates: List[Coordinate] = field(default_factory=lambda: [Coordinate() for _ in range(100)])
    intermediate_angles: 'ServoAngles' = field(default_factory=lambda: ServoAngles())
    intermediate_effector_coordinates:'ServoAngles' = field(default_factory=lambda: ServoAngles())
    stanceInterpolationIndex: float = 0.0           # current index in stance-phase interpolation
    sliding: bool = False                           # flag indicating if leg is moving back to neutral position
    RevertInterpolationIndex: int = 0               # current index in revert-to-neutral interpolation
    Debug: bool = False                             # Set debug mode for printing errors
    bezier_curve: "BezierCurve" = field(init=False)
    current_phase: str = "neutral"                  # gait(walking), rotation, reset(neutral)
    t: float = 0.0                                  # Progress along the full bezier curve [0,1]
    swing_fraction: float = 5/8                     # Hexapod swings from "start"->"grounded"
    control_points: Dict[str, np.ndarray] = field(init=False)

    def __post_init__(self):
        self.Joints = [Coordinate() for _ in range(self.num_joints)] # Setup joint positions 

        self.neutral_effector_coord = Coordinate(-20,0,-80)
        self.set_gait_control_points()
        self.servo_indexes = [self.Index * 3, self.Index * 3 + 1, self.Index * 3 + 2]
    

    def set_gait_control_points(self, stride_length:float = 10.0):
        ''' Setup the control points for a full gait cycle '''
        start = np.array([self.neutral_effector_coord.X, self.neutral_effector_coord.Y, self.neutral_effector_coord.Z])
        # Setup Control Points
        self.control_points = {
            "start": start + np.array([0, 0, 0]),
            "lift": start + np.array([10, 0, 25]),
            "peak": start + np.array([20, 0, 75]),
            "lower": start + np.array([25, 0, 25]),                 # The moment before grounding
            "touchdown": start + np.array([25, 0, 25]),             # The moment before grounding
            "grounded": start + np.array([25, 0, 0]),               # Fully on the ground
            "sliding": start + np.array([stride_length, 0, 0]),     # Sliding before reset
            "return": start + np.array([0, 0, 0])
        }
        # Initalize bezier curve with control points
        self.bezier_curve = BezierCurve(self.control_points, num_pts=50)
        self.current_phase = "gait"


    def set_rotation_control_points(self, theta:float = np.pi/2):
        ''' set control points for rotation'''
        r = np.squrt(self.neutral_effector_coord.X**2 + self.neutral_effector_coord.Y**2)
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
        self.current_phase = "rotation"
        self.swing_fraction = 5/8 # same as gait for constistancy


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
        self.current_phase = "reset"


    def set_joint_angles(self, joint, angle) -> None:
        '''
            Set the angles of a specific joint in the leg
        '''
        if joint in self.servos:
            self.servos[joint].set_angle(int(angle))
        else:
            raise ValueError(f"Invalid joint: {joint}")                      


    def Zero(self) -> None:
        self.servo_angles = ServoAngles(0,0,0)
        self.recalculate_forward_kinematics(self.servo_angles)


    def get_joint_origin(self, H: np.ndarray) -> Coordinate:
        ''' Helper to extract the origin (translation part) from a homogeneous transformation matrix. '''
        # The origin is the last column (except the homogeneous 1) of the matrix.
        return Coordinate(X=float(H[0,3]), Y=float(H[1,3]), Z=float(H[2,3]))

    def recalculate_forward_kinematics(self, angles:ServoAngles) -> None:
        # standard P & H matrix form

        self.servo_angles = angles

        p_coxa = np.array([[1,0,0], #rotate coxa around Z axis
                        [0,0,-1], # aligns coxa to robot base plane
                        [0,1,0]])
        p_femur = np.eye(3) # identity (no motion)
        p_tibia = np.eye(3) # identity (no motion)

        # Compute matrixes. Converts angle degrees to radians
        h_coxa = homogeneous_transformation_matrix(p_coxa, angles.Coxa*(np.pi/180.0), self.segment_length.Coxa)
        h_femur = homogeneous_transformation_matrix(p_femur, angles.Femur*(np.pi/180.0), self.segment_length.Femur)
        h_tibia = homogeneous_transformation_matrix(p_tibia, angles.Tibia*(np.pi/180.0), self.segment_length.Tibia)

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

    def move_leg_with_bezier(self, step_count=20):
        """
        Moves a leg along its Bezier curve using inverse kinematics.
        This is a test function
        """
        from inversekinematics import solve_effector_IK
        import time
        bezier_points = self.bezier_curve.curve()

        for i in range(step_count):
            effectorTarget = Coordinate(bezier_points[i][0], bezier_points[i][1], bezier_points[i][2])

            # Ensure the target is within the leg's reachable workspace
            max_reach = self.segment_length.Coxa + self.segment_length.Femur + self.segment_length.Tibia
            if np.linalg.norm([effectorTarget.X, effectorTarget.Y]) > max_reach:
                effectorTarget.X = (effectorTarget.X / np.linalg.norm([effectorTarget.X, effectorTarget.Y])) * max_reach
                effectorTarget.Y = (effectorTarget.Y / np.linalg.norm([effectorTarget.X, effectorTarget.Y])) * max_reach

            # Solve inverse kinematics
            angles = solve_effector_IK(self, effectorTarget)

            # Send angles to servos
            self.Coxa.set_angle(angles.Coxa)
            self.Femur.set_angle(angles.Femur)
            self.Tibia.set_angle(angles.Tibia)

            time.sleep(0.05)  # Delay for smooth motion


    def update(self, delta_t: float, direction: int, gait_pattern: List[int], gait_index: int) -> bool:
        ''' Update leg based on gait pattern '''
        from inversekinematics import solve_effector_IK
        # if if neutral or rotation change the control points to gait
        if self.current_phase != "gait":
            self.set_gait_control_points()

        # set gait index
        is_swinging = gait_pattern[gait_index] == 1
        max_t = self.swing_fraction if is_swinging else 1.0 # self.swing_fraction 5/8 if "swinging" else 1.0 speed

        # set current index between [0,1]
        self.t = max(0.0, min(max_t, self.t + delta_t * direction))
        
        # Evaluate the position at time t
        pos = self.bezier_curve.evaluate(self.t)

        # set the target for the toe from the coordinates
        self.effector_target = Coordinate(pos[0], pos[1], pos[2])

        # compute the inverse kinetmatics of the new target
        angles = solve_effector_IK(self.effector_target)

        # set the servo angles
        self.servo_angles = angles
        
        # if the leg has not completed the full bezier curve
        if self.t < max_t:
            return True  # Still moving
        
        # else don't move leg
        self.t = 0.0 if is_swinging else self.swing_fraction  # Reset or hold
        return False

    def update_rotation(self, delta_t: float, direction: int) -> bool:
        ''' Advance rotation phase '''
        from inversekinematics import solve_effector_IK

        # set the current phase to rotation bezier curve
        if self.current_phase != "rotation":
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
        if self.current_phase != "reset":
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
    index:int = 0
    coxaAngle:float = 0.0
    coxaMatrix = np.eye(4)
    servoangle = ServoAngles(0,0,0)
    segment_lengths = SegmentLengths(45,110,193)
    
    leg = Leg(
            Index=index,
            Name="LR",
            # #Setup Servo Motors ||| UNCOMMENT ON HEXAPOD |||
            # Coxa=Servo(i * 3, pca=0x40 if i * 3 < 8 else 0x41),  # Assign PCA based on ID
            # Femur=Servo(i * 3 + 1, pca=0x40 if i * 3 + 1 < 8 else 0x41),
            # Tibia=Servo(i * 3 + 2, pca=0x40 if i * 3 + 2 < 8 else 0x41),
            coxa_angle_offset = coxaAngle,
            offset_transformation_matrix=coxaMatrix,
            segment_length=segment_lengths,
            servo_angles=servoangle,  # Provide default servo angles
            neutral_effector_coord=Coordinate(0, 0, 0)
        )
    

    leg.recalculate_forward_kinematics(servoangle)
    leg.neutral_effector_coord = leg.Joints[EFFECTOR_ORIGIN_INDEX]

    return leg


def test_single_leg_movement(leg):
    """ Test function to manually move each leg's servos using predefined angles. """
    import time
    print("Starting leg movement test...")

    # Move Coxa to 0 degrees for each leg
    leg.Coxa.set_angle(0)  # Move coxa servo to neutral position
    time.sleep(.02)

    time.sleep(1)

    # Move Femur to -120 degrees (lifting the leg)
    leg.Femur.set_angle(-120)  # Lift femur joint
    time.sleep(.02)

    time.sleep(1)

    # Move Tibia back to 0 degrees
    leg.Tibia.set_angle(0)  # Reset tibia joint
    time.sleep(.02)

    time.sleep(1)

    # Move Coxa back to neutral (optional reset)

    leg.Coxa.set_angle(0)
    leg.Femur.set_angle(0)
    leg.Tibia.set_angle(0)
    time.sleep(.02)
    print("Leg movement test completed!")



def test_legs_movement(legs):
    """Test function to manually move each leg's servos using predefined angles."""
    import time
    print("Starting leg movement test...")
    # Move Coxa to 0 degrees for each leg
    for leg in legs:
        print(leg.Coxa)
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

    print("Leg movement test completed!")


def test_bezier_curve_movement(leg) -> None:
        '''Simulated "Pod Update" function for the leg movement '''
        ''' Update leg based on gait pattern '''
        from inversekinematics import solve_effector_IK
        # if if neutral or rotation change the control points to gait
        if leg.current_phase != "gait":
            leg.set_gait_control_points()

        # set gait index
        is_swinging = True
        max_t = 1.0 # self.swing_fraction 5/8 if "swinging" else 1.0 speed
        delta_t = 0.05
        direction = 1

        # set current index between [0,1]
        leg.t = max(0.0, min(max_t, leg.t + delta_t * direction))
        
        # Evaluate the position at time t
        pos = leg.bezier_curve.evaluate(leg.t)

        # set the target for the toe from the coordinates
        leg.effector_target = Coordinate(pos[0], pos[1], pos[2])

        # compute the inverse kinetmatics of the new target
        angles = solve_effector_IK(leg, leg.effector_target)

        # set the servo angles
        leg.servo_angles = angles
        
        leg.Coxa.set_angle(angles.Coxa)
        leg.Femur.set_angle(angles.Femur)
        leg.Tibia.set_angle(angles.Tibia)

        # if the leg has not completed the full bezier curve
        if leg.t < max_t:
            return True  # Still moving
        
        # else don't move leg
        leg.t = 0.0 if is_swinging else leg.swing_fraction  # Reset or hold and wait for slide
        return False

## Also test move_leg_with_bezier(self):



if __name__ == "__main__":


    hexapod_leg = new_leg()
    while True:
        test_bezier_curve_movement(hexapod_leg)

    # test again with bezier curve itself no inverse kinematics
    # hexapod_leg.move_leg_with_bezier()

    # Works on raspi4 
    # hexapod_leg = new_leg()
    # test_single_leg_movement(hexapod_leg)


    #hexapod_legs = new_legs()
    # test_legs_movement(hexapod_legs)