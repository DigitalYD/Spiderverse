
import numpy as np
import math
from src.coord import ServoAngles, Coordinate
from src.config import COXA_ORIGIN_INDEX, FEMUR_ORIGIN_INDEX




def solve_effector_IK(leg, effector_target, debug_channel=None):
    """
    Compute inverse kinematics for a hexapod leg to reach an effector target.
    
    Args:
        leg: Leg object with Joints, SegmentLengths, and CoxaSeparationAngle.
        effector_target: Coordinate object with X, Y, Z.
        debug_channel: Optional list to simulate Go's channel for debug messages (default None).
    
    Returns:
        ServoAngles: Computed servo angles (Coxa, Femur, Tibia).
    
    Raises:
        ValueError: If the target is unreachable (NaN in angle calculations).
    """
    servo_angles = ServoAngles()

    # Inverse kinematics equation 1: Coxa angle
    x = effector_target.X - leg.Joints[COXA_ORIGIN_INDEX].X
    y = effector_target.Y - leg.Joints[COXA_ORIGIN_INDEX].Y
    servo_angles.Coxa = (180.0 / math.pi) * np.arctan2(y, x) + 360.0 - leg.coxa_angle_offset
    # If leg.Index is needed: + 360.0 - leg.CoxaSeparationAngle * float(leg.Index)

    if servo_angles.Coxa >= 180:
        servo_angles.Coxa -= 360

    # Inverse kinematics equation 2: Lengths in XY and Z
    dx = effector_target.X - leg.Joints[COXA_ORIGIN_INDEX].X
    dy = effector_target.Y - leg.Joints[COXA_ORIGIN_INDEX].Y
    L1 = np.sqrt(dx * dx + dy * dy) - leg.segment_length.Coxa
    L2 = effector_target.Z - leg.Joints[FEMUR_ORIGIN_INDEX].Z
    L = np.sqrt(L2 * L2 + L1 * L1)

    # Inverse kinematics equation 3: Alpha 1 (angle from horizontal to target)
    alpha_1 = np.arccos(L2 / L)
    if math.isnan(alpha_1):
        if debug_channel is not None:
            debug_channel.append("[IK Solver] ERROR: Unable to find a solution. Target is too far away.")
        raise ValueError("[IK Solver] ERROR: Unable to find a solution. Target is too far away.")

    # Inverse kinematics equation 4: Alpha 2 (femur adjustment)
    alpha_2 = np.arccos(np.clip(
        (leg.segment_length.Tibia * leg.segment_length.Tibia -
         leg.segment_length.Femur * leg.segment_length.Femur -
         L * L) /
        (-2 * leg.segment_length.Femur * L),-1,1)
    )
    if math.isnan(alpha_2):
        if debug_channel is not None:
            debug_channel.append("[IK Solver] ERROR: Unable to find a solution. Target is too far away.")
        raise ValueError("[IK Solver] ERROR: Unable to find a solution. Target is too far away.")

    # Inverse kinematics equation 5: Femur angle
    servo_angles.Femur = 90 - (180.0 / np.pi) * (alpha_1 + alpha_2)
    # Inverse kinematics equation 6: Tibia angle
    # Works in simulator
    servo_angles.Tibia = 180 - (180.0 / math.pi) * np.arccos( np.clip(
        (L * L - leg.segment_length.Femur * leg.segment_length.Femur - leg.segment_length.Tibia * leg.segment_length.Tibia) /
        (-2 * leg.segment_length.Tibia * leg.segment_length.Femur),-1,1)
    )

    leg.servo_angles = servo_angles
    #print(servo_angles)


    ## Test more on the middle leg, may need to make an exception for it, or rotate motor on leg manually    
    return servo_angles
    
def sim_solve_effector_IK(leg, effector_target, debug_channel=None):
    """
    Compute inverse kinematics for a hexapod leg to reach an effector target.
    
    Args:
        leg: Leg object with Joints, SegmentLengths, and CoxaSeparationAngle.
        effector_target: Coordinate object with X, Y, Z.
        debug_channel: Optional list to simulate Go's channel for debug messages (default None).
    
    Returns:
        ServoAngles: Computed servo angles (Coxa, Femur, Tibia).
    
    Raises:
        ValueError: If the target is unreachable (NaN in angle calculations).
    """
    servo_angles = ServoAngles()

    # Inverse kinematics equation 1: Coxa angle
    x = effector_target.X - leg.Joints[COXA_ORIGIN_INDEX].X
    y = effector_target.Y - leg.Joints[COXA_ORIGIN_INDEX].Y
    servo_angles.Coxa = (180.0 / math.pi) * np.arctan2(y, x) + 360.0 - leg.coxa_angle_offset
    # If leg.Index is needed: + 360.0 - leg.CoxaSeparationAngle * float(leg.Index)

    if servo_angles.Coxa >= 180:
        servo_angles.Coxa -= 360

    # Inverse kinematics equation 2: Lengths in XY and Z
    dx = effector_target.X - leg.Joints[COXA_ORIGIN_INDEX].X
    dy = effector_target.Y - leg.Joints[COXA_ORIGIN_INDEX].Y
    L1 = np.sqrt(dx * dx + dy * dy) - leg.segment_length.Coxa
    L2 = effector_target.Z - leg.Joints[FEMUR_ORIGIN_INDEX].Z
    L = np.sqrt(L2 * L2 + L1 * L1)

    # Inverse kinematics equation 3: Alpha 1 (angle from horizontal to target)
    alpha_1 = np.arccos(L2 / L)
    if math.isnan(alpha_1):
        if debug_channel is not None:
            debug_channel.append("[IK Solver] ERROR: Unable to find a solution. Target is too far away.")
        raise ValueError("[IK Solver] ERROR: Unable to find a solution. Target is too far away.")

    # Inverse kinematics equation 4: Alpha 2 (femur adjustment)
    alpha_2 = np.arccos(np.clip(
        (leg.segment_length.Tibia * leg.segment_length.Tibia -
         leg.segment_length.Femur * leg.segment_length.Femur -
         L * L) /
        (-2 * leg.segment_length.Femur * L),-1,1)
    )
    if math.isnan(alpha_2):
        if debug_channel is not None:
            debug_channel.append("[IK Solver] ERROR: Unable to find a solution. Target is too far away.")
        raise ValueError("[IK Solver] ERROR: Unable to find a solution. Target is too far away.")

    # # Inverse kinematics equation 5: Femur angle
    servo_angles.Femur = 90 - (180.0 / np.pi) * (alpha_1 + alpha_2)
    # Inverse kinematics equation 6: Tibia angle
    # Works in simulator
    servo_angles.Tibia = 180 - (180.0 / math.pi) * np.arccos( np.clip(
        (L * L - leg.segment_length.Femur * leg.segment_length.Femur - leg.segment_length.Tibia * leg.segment_length.Tibia) /
        (-2 * leg.segment_length.Tibia * leg.segment_length.Femur),-1,1)
    )
    leg.servo_angles = servo_angles