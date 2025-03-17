import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional
from coord import Coordinate, ServoAngles, new_Coordinate, new_servo_angles
from config import POD_Z_HEIGHT, NUM_JOINTS, COXA_ORIGIN_INDEX, FEMUR_ORIGIN_INDEX, TIBIA_ORIGIN_INDEX, EFFECTOR_ORIGIN_INDEX, INTERPOLATION_STEPS, TIBIA_ORIGIN_INDEX, EFFECTOR_ORIGIN_INDEX, Z_LIFT, REVERT_LIFT, Forward, Reverse


@dataclass
class SegmentLengths:
    """ Defines the lengths of each segment in the robot leg (Coxa, Femur, Tibia) """
    Coxa: float = 0.0   # length of the coxa 
    Femur: float = 0.0  # length of the femur 
    Tibia: float = 0.0  # length of the tibia 

def homogeneous_transformation_matrix(projection_matrix: np.ndarray, theta: float, length: float) -> np.ndarray:
    """Create a homogeneous transformation matrix given a projection matrix, a rotation angle theta (rad), and a displacement length."""
    # Rotation matrix about Z-axis by theta
    R_z = np.array([
        [math.cos(theta), -math.sin(theta), 0],
        [math.sin(theta),  math.cos(theta),  0],
        [0,               0,                1]
    ])
    # Final rotation = R_z * projection_matrix (3x3)
    R = R_z.dot(projection_matrix)
    # Displacement vector for this segment in the rotated frame
    D = np.array([[length * math.cos(theta)], [length * math.sin(theta)], [0]])
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
    JointAngle: List[List[float]] = field(default_factory=lambda: [[0.0]*INTERPOLATION_STEPS for _ in range(NUM_JOINTS-1)])

@dataclass
class Leg:
    """Leg represents a single robotic leg and its kinematic state."""
    Index: int                                # leg index (0 to NumLegs-1)
    coxa_offset: float                        # coxa offset angle
    offsettransformationMatrix: np.ndarray    # 4x4 transform for the coxa frame relative to robot base
    segment_lengths: SegmentLengths            # lengths of coxa, femur, tibia segments
    servo_angles: ServoAngles                  # current servo angles of the leg
    NeutralEffectorCoordinate: Coordinate     # end effector coordinate when leg is in neutral position
    Joints: List[Coordinate] = field(default_factory=lambda: [Coordinate() for _ in range(NUM_JOINTS)])
    EffectorTarget: Coordinate = Coordinate() # current target position of the end effector (after movement)
    IntermediateAngles = IntermediateAngles()  # interpolation table for joint angles (swing)
    IntermediateEffectorCoordinates: List[Coordinate] = field(default_factory=lambda: [Coordinate() for _ in range(INTERPOLATION_STEPS)])
    swingInterpolationIndex: int = 0          # current index in swing-phase interpolation
    stanceInterpolationIndex: float = 0.0     # current index in stance-phase interpolation
    IsReverting: bool = False                 # flag indicating if leg is moving back to neutral position
    RevertInterpolationIndex: int = 0         # current index in revert-to-neutral interpolation

    def GetJointOrigin(self, H: np.ndarray) -> Coordinate:
        """Helper to extract the origin (translation part) from a homogeneous transformation matrix."""
        # The origin is the last column (except the homogeneous 1) of the matrix.
        return Coordinate(X=float(H[0,3]), Y=float(H[1,3]), Z=float(H[2,3]))

    def RecalculateForwardKinematics(self, angles: ServoAngles) -> None:
        """Compute the forward kinematics for the leg given joint angles, updating joint coordinates."""
        self.ServoAngles = angles  # update current servo angles
        # Define projection matrices for each joint rotation relative to previous joint's frame (from Go code).
        P_Coxa = np.array([[1, 0, 0],    # Rotation of Coxa around Z axis is in base plane (project onto X-Y plane, with a 90° rotation in Y-axis in the Go code)
                           [0, 0, -1],   # This projection matrix aligns Coxa rotation to the robot base plane.
                           [0, 1, 0]])
        P_Femur = np.eye(3)  # Femur rotates in a plane parallel to Coxa (no projection rotation needed in this simple model).
        P_Tibia = np.eye(3)  # Tibia rotates in the same plane as Femur in this model.
        # Compute homogeneous transformation matrices for each joint segment
        H_Coxa = homogeneous_transformation_matrix(P_Coxa, angles.Coxa * math.pi/180.0, self.SegmentLengths.Coxa)
        H_Femur = homogeneous_transformation_matrix(P_Femur, angles.Femur * math.pi/180.0, self.SegmentLengths.Femur)
        H_Tibia = homogeneous_transformation_matrix(P_Tibia, angles.Tibia * math.pi/180.0, self.SegmentLengths.Tibia)
        # Multiply matrices to get cumulative transforms:
        H0_1 = self.OffsetTransformationMatrix.dot(H_Coxa)   # transform from base to femur base
        H1_2 = H0_1.dot(H_Femur)                            # transform to tibia base
        H2_3 = H1_2.dot(H_Tibia)                            # transform to end effector
        # Extract joint origins in base frame
        self.Joints[COXA_ORIGIN_INDEX]  = self.GetJointOrigin(self.OffsetTransformationMatrix)
        self.Joints[FEMUR_ORIGIN_INDEX] = self.GetJointOrigin(H0_1)
        self.Joints[TIBIA_ORIGIN_INDEX] = self.GetJointOrigin(H1_2)
        self.Joints[EFFECTOR_ORIGIN_INDEX] = self.GetJointOrigin(H2_3)
        # Set current effector target position to the end effector origin
        self.EffectorTarget = self.Joints[EFFECTOR_ORIGIN_INDEX]

    def UpdateSwing(self, direction: int) -> bool:
        """Advance the leg's swing-phase interpolation by one step in the given direction.
        Returns True if a swing phase just finished (end of swing), False otherwise."""
        # Retrieve the next set of angles from the precomputed IntermediateAngles table
        j = self.swingInterpolationIndex  # current index shorthand
        angles = new_servo_angles(
            self.IntermediateAngles.JointAngle[0][j],
            self.IntermediateAngles.JointAngle[1][j],
            self.IntermediateAngles.JointAngle[2][j]
        )
        # Apply forward kinematics for these intermediate angles
        self.RecalculateForwardKinematics(angles)
        # Lift the leg in swing phase: modify the Z target to simulate an arc motion (end effector moves up).
        phase_step = math.pi / (INTERPOLATION_STEPS - 1)  # incremental phase for sine wave
        z_current = self.Joints[EFFECTOR_ORIGIN_INDEX].Z
        phase = phase_step * self.swingInterpolationIndex  # current phase (0 to pi)
        z_lift = Z_LIFT * math.sin(phase)  # vertical lift offset
        # Solve inverse kinematics for the new target (same X,Y, lifted Z)
        target = new_Coordinate(self.Joints[EFFECTOR_ORIGIN_INDEX].X, 
                                self.Joints[EFFECTOR_ORIGIN_INDEX].Y, 
                                z_current - z_lift)
        # SolveEffectorIK returns new angles for the adjusted target.
        # We ignore the error here for simplicity.
        self.ServoAngles, _ = SolveEffectorIK(self, target)
        self.RecalculateForwardKinematics(self.ServoAngles)  # update joint positions with corrected angles
        # Increment swing index in given direction
        self.swingInterpolationIndex += int(direction)
        # Wrap around logic for swing index and stance index when reaching ends of table
        if direction == Forward:
            if self.swingInterpolationIndex >= INTERPOLATION_STEPS:
                # Completed a full swing cycle forward
                self.swingInterpolationIndex = 0
                self.stanceInterpolationIndex = INTERPOLATION_STEPS - 1
                return True  # signal end of swing phase
        else:  # direction == Reverse
            if self.swingInterpolationIndex < 0:
                # Completed a full swing cycle in reverse
                self.swingInterpolationIndex = INTERPOLATION_STEPS - 1
                self.stanceInterpolationIndex = 0
                return True
        return False  # still in swing phase

    def UpdateStance(self, direction: int, stance_return_factor: float) -> None:
        """Advance the leg's stance-phase interpolation (returning leg to starting position) by one step."""
        # Use the current stanceInterpolationIndex (which may be fractional) to get angles
        idx = int(self.stanceInterpolationIndex)
        angles = new_servo_angles(
            self.IntermediateAngles.JointAngle[0][idx],
            self.IntermediateAngles.JointAngle[1][idx],
            self.IntermediateAngles.JointAngle[2][idx]
        )
        self.ServoAngles = angles
        self.RecalculateForwardKinematics(angles)
        # Move stance index opposite to swing (subtract if moving forward, add if reverse) scaled by stance_return_factor
        self.stanceInterpolationIndex -= direction * stance_return_factor
        # Handle wrap-around for stance index
        if direction == Forward:
            if self.stanceInterpolationIndex < 0:
                # if we've gone past the beginning, wrap to end
                self.stanceInterpolationIndex = INTERPOLATION_STEPS - 1
                self.swingInterpolationIndex = 0
        else:  # Reverse direction
            if self.stanceInterpolationIndex >= INTERPOLATION_STEPS:
                # if gone past the end, wrap to start
                self.stanceInterpolationIndex = 0
                self.swingInterpolationIndex = INTERPOLATION_STEPS - 1

    def ResetInterpolator(self) -> None:
        """Reset swing and stance interpolation indices to the midpoint of their range."""
        # Setting to center ensures legs start in middle of their interpolation cycles (for smooth movement start)
        self.swingInterpolationIndex = (INTERPOLATION_STEPS - 1) // 2
        self.stanceInterpolationIndex = float(self.swingInterpolationIndex)

    def Zero(self) -> None:
        """Set all servo angles to 0 degrees and recalc forward kinematics (straighten the leg)."""
        self.ServoAngles = ServoAngles(0, 0, 0)
        self.RecalculateForwardKinematics(self.ServoAngles)

    def Ground(self, height: float) -> Optional[Exception]:
        """Anchor the end effector at a given Z height by adjusting servo angles (inverse kinematics).
        Returns an error (Exception) if no solution is found, otherwise None."""
        target = new_Coordinate(self.Joints[EFFECTOR_ORIGIN_INDEX].X,
                                self.Joints[EFFECTOR_ORIGIN_INDEX].Y,
                                height)
        angles, err = SolveEffectorIK(self, target)
        if err:
            return err  # pass the IK error upward
        self.ServoAngles = angles
        self.RecalculateForwardKinematics(self.ServoAngles)
        return None

    def RevertToNeutral(self) -> None:
        """Prepare the leg's interpolation table to move from current position back to neutral (rest) position."""
        target_coord = self.NeutralEffectorCoordinate
        start_coord = self.Joints[EFFECTOR_ORIGIN_INDEX]
        # Calculate the incremental step in X, Y, Z to go from current to neutral in INTERPOLATION_STEPS steps
        stepX = (target_coord.X - start_coord.X) / (INTERPOLATION_STEPS - 1)
        stepY = (target_coord.Y - start_coord.Y) / (INTERPOLATION_STEPS - 1)
        stepZ = (target_coord.Z - start_coord.Z) / (INTERPOLATION_STEPS - 1)
        # Recompute intermediate joint angles for each step from current position to neutral
        for i in range(INTERPOLATION_STEPS):
            swing_coord = new_Coordinate(
                start_coord.X + stepX * i,
                start_coord.Y + stepY * i,
                start_coord.Z + stepZ * i
            )
            angles, _ = SolveEffectorIK(self, swing_coord, self.debugChannel)
            # Store the angles for this interpolation step
            self.IntermediateAngles.JointAngle[0][i] = angles.Coxa
            self.IntermediateAngles.JointAngle[1][i] = angles.Femur
            self.IntermediateAngles.JointAngle[2][i] = angles.Tibia
        # Once prepared, the leg's IntermediateAngles will be used in UpdateReverting phases.

    def UpdateRevertPhase0(self) -> None:
        """First phase of revert: 'ground' all legs (ensure all legs' feet are on ground plane)."""
        # Use the current revert interpolation index to get intermediate angles (similar to UpdateStance/Swing)
        j = self.RevertInterpolationIndex
        angles = new_servo_angles(
            self.IntermediateAngles.JointAngle[0][j],
            self.IntermediateAngles.JointAngle[1][j],
            self.IntermediateAngles.JointAngle[2][j]
        )
        self.RecalculateForwardKinematics(angles)
        # Move the leg's end effector to the ground height (POD_Z_HEIGHT) at the current X, Y.
        target = new_Coordinate(self.Joints[EFFECTOR_ORIGIN_INDEX].X,
                                self.Joints[EFFECTOR_ORIGIN_INDEX].Y,
                                POD_Z_HEIGHT)
        self.ServoAngles, _ = SolveEffectorIK(self, target, self.debugChannel)
        self.RecalculateForwardKinematics(self.ServoAngles)

    def UpdateRevertPhase1(self) -> int:
        """Second phase of revert: move leg from current grounded position back up to neutral position step by step.
        Returns the leg index that should be processed next (or the same leg if not done)."""
        j = self.RevertInterpolationIndex
        angles = new_servo_angles(
            self.IntermediateAngles.JointAngle[0][j],
            self.IntermediateAngles.JointAngle[1][j],
            self.IntermediateAngles.JointAngle[2][j]
        )
        self.RecalculateForwardKinematics(angles)
        # Advance the interpolation index for reverting
        self.RevertInterpolationIndex += 1
        if self.RevertInterpolationIndex >= INTERPOLATION_STEPS:
            # Finished reverting this leg
            self.IsReverting = False
            self.RevertInterpolationIndex = 0
            return self.Index + 1  # signal to move to next leg
        # If not finished, adjust Z with a small lift to ensure foot clears the ground while moving to neutral
        phase_step = math.pi / (INTERPOLATION_STEPS - 1)
        z_current = self.Joints[EFFECTOR_ORIGIN_INDEX].Z
        phase = phase_step * self.RevertInterpolationIndex
        z_lift = REVERT_LIFT * math.sin(phase)
        target = new_Coordinate(self.Joints[EFFECTOR_ORIGIN_INDEX].X,
                                self.Joints[EFFECTOR_ORIGIN_INDEX].Y,
                                z_current - z_lift)
        self.ServoAngles, _ = SolveEffectorIK(self, target)
        self.RecalculateForwardKinematics(self.ServoAngles)
        return self.Index  # continue with same leg until done
    


def SolveEffectorIK(leg: Leg, effector_target: Coordinate, debug_channel: Optional[List[str]] = None) -> list:
    """Solve the inverse kinematics for a leg to reach a given end effector (foot) target coordinate.
    Returns a tuple (ServoAngles, error). If no solution is found, error will be an Exception explaining the issue."""
    # This function attempts to compute Coxa, Femur, Tibia angles for the leg such that the end effector reaches effector_target.
    # It mirrors the mathematical steps in the Go SolveEffectorIK.
    servo_angles = ServoAngles()
    # 1. Compute planar projection distances
    dx = effector_target.X - leg.Joints[COXA_ORIGIN_INDEX].X  # horizontal delta X from Coxa origin
    dy = effector_target.Y - leg.Joints[COXA_ORIGIN_INDEX].Y  # horizontal delta Y from Coxa origin
    # 2. Coxa angle: angle of the leg in the X-Y plane
    # In Go, they did: Atan2(y, x) + 360 - CoxaSeparationAngle (for multi-leg offset). 
    # Here, CoxaSeparationAngle already accounts for the leg's orientation around the body.
    angle_deg = (180.0/math.pi)*math.atan2(dy, dx) + 360.0 - leg.CoxaSeparationAngle
    servo_angles.Coxa = angle_deg
    if servo_angles.Coxa >= 180.0:
        servo_angles.Coxa -= 360.0  # normalize to [-180, 180) range for servo angle
    # 3. Distance calculations for femur/tibia (in the vertical plane of the leg)
    L1 = math.hypot(dx, dy) - leg.SegmentLengths.Coxa  # horizontal distance from femur joint to target
    L2 = effector_target.Z - leg.Joints[FEMUR_ORIGIN_INDEX].Z  # vertical distance from femur joint to target
    L = math.hypot(L1, L2)  # straight-line distance from femur joint to target
    # 4. Compute angles using triangle geometry (law of cosines etc.)
    # angle alpha1 (angle between L and horizontal plane)
    if L == 0 or abs(L2/L) > 1.0:
        # If target is too close or beyond reach causing invalid ratio for acos, return error
        err = Exception("[IK Solver] ERROR: Unable to find a solution. Target is too far or too close.")
        if debug_channel is not None:
            debug_channel.append(str(err))
        return servo_angles, err
    alpha_1 = math.acos(max(-1.0, min(1.0, L2 / L)))  # clamp argument between -1 and 1 for safety
    # angle alpha2 (per law of cosines for femur-tibia-target triangle)
    # Using lengths: Femur, Tibia, and L (distance from femur joint to target)
    denom = -2 * leg.SegmentLengths.Femur * L
    # Avoid division by zero and domain errors in acos
    if denom == 0:
        err = Exception("[IK Solver] ERROR: Degenerate configuration in IK solver.")
        if debug_channel is not None:
            debug_channel.append(str(err))
        return servo_angles, err
    cos_alpha2 = (leg.SegmentLengths.Tibia**2 - leg.SegmentLengths.Femur**2 - L**2) / denom
    if cos_alpha2 < -1.0 or cos_alpha2 > 1.0:
        # No solution if cos_alpha2 is out of valid range
        err = Exception("[IK Solver] ERROR: Unable to find a solution. Target is too far away.")
        if debug_channel is not None:
            debug_channel.append(str(err))
        return servo_angles, err
    alpha_2 = math.acos(cos_alpha2)
    # 5. Femur angle calculation:
    servo_angles.Femur = 90.0 - (180.0/math.pi) * (alpha_1 + alpha_2)
    # 6. Tibia angle calculation:
    cos_term = (L**2 - leg.SegmentLengths.Femur**2 - leg.SegmentLengths.Tibia**2) / (-2 * leg.SegmentLengths.Tibia * leg.SegmentLengths.Femur)
    # Clamp cos_term to valid range [-1,1] to avoid math domain errors
    cos_term = max(-1.0, min(1.0, cos_term))
    servo_angles.Tibia = 180.0 - (180.0/math.pi) * math.acos(cos_term)


    return servo_angles, None
