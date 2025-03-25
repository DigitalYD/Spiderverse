
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
    
    ## Test more on the middle leg, may need to make an exception for it, or rotate motor on leg manually
        
    
    return servo_angles


# def solve_effector_IK(leg, effectorTarget:Coordinate, debug:bool = False) -> ServoAngles: 
#     ##--------------------------
#     ## Variation 1 Bezier Curve
#     ##--------------------------
#     servoAngle = ServoAngles()
    
#     # Inverse kin equation 1
#     x = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
#     y = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y

#     # Eq 1
#     # Calculate coxa, convert to degrees (180.0/np.pi), and apply offset,
#     servoAngle.Coxa = np.degrees(np.arctan2(y, x))
#     servoAngle.Coxa = servoAngle.Coxa % 360 # Wrap it around to 360
#     if servoAngle.Coxa >= 180:
#         servoAngle.Coxa = servoAngle.Coxa - 360 # bring coxa angle back within 0-360 degrees may need to clip smaller like 180
#     #servoAngle.Coxa = max(0, min(360, servoAngle.Coxa))  # Assuming 0-180° range


#     # Moving to leg Frame
#     # Eq 2
#     dx = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
#     dy = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y

#     L1 = np.sqrt((dx**2 + dy**2)) - leg.segment_length.Coxa  # Avoid division by zero
#     L2 = effectorTarget.Z - leg.Joints[FEMUR_ORIGIN_INDEX].Z

#     # # Lock femur/Tibia during sliding phase
#     # if leg.current_leg_phase == "sliding":
#     #     print("sliding")
#     #     servoAngle.Femur = -100
#     #     servoAngle.Tibia = 20
#     # else:
#     L = np.sqrt(L2**2 + L1**2)

#     # print(f"L1: {L1}, L2: {L2}, L: {L}")
#     # Eq 3
#     alpha_1 = np.arccos(L2 / L)  # -π/2 to π/2

#     if math.isnan(alpha_1):
#         return servoAngle
    
#     alpha_2 = np.arccos((leg.segment_length.Tibia**2 - 
#                                  leg.segment_length.Femur**2 - 
#                                  L**2) / 
#                                 (-2 * leg.segment_length.Femur * L))
#     if math.isnan(alpha_2):
#         return servoAngle
    
    
#     # Perform offsets for each motor, and convert to angles
#     # if effectorTarget.Z > 0:
#     servoAngle.Femur =  - np.degrees(np.pi/4 - (alpha_1 + alpha_2))  # 0° = up
#     # else:
#     #     servoAngle.Femur = np.degrees((alpha_1 - alpha_2))
    
#     servoAngle.Tibia = (np.degrees(np.pi/2 - np.arccos(np.clip(
#                               (L**2 -
#                               leg.segment_length.Femur**2 - 
#                               leg.segment_length.Tibia**2 / 
#                               (-2 * leg.segment_length.Tibia * leg.segment_length.Femur)), -1, 1))))              # 0° = straight
#     print(servoAngle)
#     if debug == True:
#       print(f"Prior to making x/y: endo_target.X: {effectorTarget.X}, endo_target.Y: {effectorTarget.Y}")
#       print(f"x: {x}, y: {y}")
#       print(f"endo_target.X: {effectorTarget.X}, endo_target.Y: {effectorTarget.Y}")
#       print(f"Coxa.X: {leg.Joints[COXA_ORIGIN_INDEX].X}, Coxa.Y: {leg.Joints[COXA_ORIGIN_INDEX].Y}")
#       print("Effector Target X: ", effectorTarget.X)
#       print("Effector Target Y: ", effectorTarget.Y)
#       print(f"Coxa.X: {leg.Joints[COXA_ORIGIN_INDEX].X}, Coxa.Y: {leg.Joints[COXA_ORIGIN_INDEX].Y}")
#       print("dx: ", dx, " dy: ", dy)
#       print("Segment lengths",leg.segment_length) # comes to 45. 
#       print(f"Alpha_1: {alpha_1}, Alpha_2: {alpha_2}, Alpha: {servoAngle.Femur}, Beta: {servoAngle.Tibia}")
#       print("Servo Angle after IK: ", servoAngle)
    
#     return servoAngle


# def solve_effector_IK(leg, effectorTarget: Coordinate, debug: bool = False) -> ServoAngles:
#     ##--------------------------
#     ## Variation 1 Bezier Curve with Three Cases for Beta
#     ##--------------------------
#     servoAngle = ServoAngles()
    
#     # Inverse kinematics equation 1: Coxa angle (theta1)
#     print(f"Leg Coxa Origin {leg.Joints[COXA_ORIGIN_INDEX].X}, {leg.Joints[COXA_ORIGIN_INDEX].Y}")
#     x = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
#     y = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y
#     print(f'X, Y: Effector - Coxa Origin {x}, {y}')


#     #get vector from coxa and effector
#     vec = np.array([x,y])
    
#     #convert offset to rad
#     theta = np.radians(leg.coxa_angle_offset)

#     # rot matrix to local leg frame
#     rot_mat = np.array([
#         [np.cos(theta), -np.sin(theta)],
#         [np.sin(theta), np.cos(theta)]
#     ])
#     local_vec = rot_mat @ vec

#     servoAngle.Coxa = np.degrees(np.arctan2(local_vec[1], local_vec[0]))
#     # Calculate coxa angle, convert to degrees, and apply offset
#     # servoAngle.Coxa = np.degrees(np.arctan2(y, x))
#     print(f"Coxa angle (local): {servoAngle.Coxa:.2f} degrees")
#     if servoAngle.Coxa > 180:
#         servoAngle.Coxa -= 360
#     elif servoAngle.Coxa < -180:
#         servoAngle.Coxa += 360


#     # print(f"Coxa Angle arctan2: {servoAngle.Coxa}")
#     # servoAngle.Coxa = (servoAngle.Coxa) % 180  # Normalize to [-180, 180)
#     # servoAngle.Coxa = np.clip(servoAngle.Coxa, -90, 90)  # Adjust based on servo spec

#     # if servoAngle.Coxa >= 180:
#     #     servoAngle.Coxa = servoAngle.Coxa - 360  # Normalize to -180° to 180°
#     #servoAngle.Coxa = max(0, min(180, servoAngle.Coxa))  # Assuming 0-180° range
    
#     # Moving to leg frame
#     dx = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
#     dy = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y


#     #L1 = np.sqrt(local_vec[0]**2 + local_vec[1]**2) - leg.segment_length.Coxa  # Horizontal distance minus coxa length
#     L1 = np.linalg.norm(local_vec) - leg.segment_length.Coxa
#     L2 = effectorTarget.Z - leg.Joints[FEMUR_ORIGIN_INDEX].Z  # Vertical distance
#     L = np.sqrt(L1**2 + L2**2)  # Total distance from femur origin to target

#     # Compute phi and theta (interpreted from your alpha_1 and alpha_2)
#     alpha_1 = np.arctan2(L2 , L1)   # Angle from horizontal to target line
#     print(f'phi: {alpha_1}')
    
#     if math.isnan(alpha_1):
#         if debug:
#             print("Phi is NaN, returning default servo angles")
#         return servoAngle

#     theta = np.arccos(np.clip( (leg.segment_length.Tibia**2 - leg.segment_length.Femur**2 - L**2) /
#                       (-2 * leg.segment_length.Femur * L) ,-1,1))
    
#     if math.isnan(theta):
#         if debug:
#             print("Theta is NaN, returning default servo angles")
#         return servoAngle

#     # Handle three cases for beta (femur angle adjustment)
#     if effectorTarget.Z > leg.Joints[COXA_ORIGIN_INDEX].Z:
#         # Case 3: End effector above coxa
#         beta = alpha_1 + theta
#     else:
#         # Case 1 & 2: Below coxa, beta = theta - phi
#         beta = theta - alpha_1  # Positive if phi < theta (Case 1), negative if phi > theta (Case 2)

#     # Compute femur angle (theta2) using beta
#     servoAngle.Femur = np.degrees(beta)  # Directly use beta in degrees, adjust offset if needed

#     # Compute tibia angle (theta3) as per original
#     numerator = L**2 - leg.segment_length.Femur**2 - leg.segment_length.Tibia**2
#     denominator = -2 * leg.segment_length.Tibia * leg.segment_length.Femur
#     if denominator == 0:
#         if debug:
#             print("Denominator zero in tibia calculation, returning default servo angles")
#         return servoAngle
    
#     arg = np.clip(numerator / denominator, -1, 1)  # Ensure argument is valid for arccos
#     gamma = np.arccos(arg)
#     servoAngle.Tibia = np.degrees(gamma)

#     servoAngle.Coxa = int(servoAngle.Coxa)
#     servoAngle.Femur = int(servoAngle.Femur)
#     servoAngle.Tibia = int(servoAngle.Tibia)


#     print(f"ServoAngles: {servoAngle}")
#     # Debugging output
#     if debug:
#         print(f"Effector Target: X={effectorTarget.X}, Y={effectorTarget.Y}, Z={effectorTarget.Z}")
#         print(f"Coxa Origin: X={leg.Joints[COXA_ORIGIN_INDEX].X}, Y={leg.Joints[COXA_ORIGIN_INDEX].Y}, Z={leg.Joints[COXA_ORIGIN_INDEX].Z}")
#         print(f"x={x}, y={y}, dx={dx}, dy={dy}")
#         print(f"L1={L1}, L2={L2}, L={L}")
#         print(f"Phi={phi}, Theta={theta}, Beta={beta}")
#         print(f"Servo Angles: {servoAngle}")
#         print(f"Segment Lengths: Coxa={leg.segment_length.Coxa}, Femur={leg.segment_length.Femur}, Tibia={leg.segment_length.Tibia}")

#     return servoAngle

