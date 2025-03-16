
from coord import ServoAngles, Coordinate
from leg import Leg, COXA_ORIGIN_INDEX, FEMUR_ORIGIN_INDEX
import numpy as np

def solve_effector_IK(leg:Leg, effectorTarget:Coordinate) -> ServoAngles:
    ##--------------------------
    #### Variation 2 (Remake 3-7-2025)
    ##--------------------------
    servoAngle = ServoAngles()
    print(f"Leg lengths: {leg.segment_length}")
    print(f"Leg joints before IK: {leg.Joints}")
    # Inverse kin equation 1
    #print(f"Prior to making x/y: endo_target.X: {effectorTarget.X}, endo_target.Y: {effectorTarget.Y}")
    x = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
    y = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y

    # print(f"x: {x}, y: {y}")
    # print(f"endo_target.X: {effectorTarget.X}, endo_target.Y: {effectorTarget.Y}")
    # print(f"Coxa.X: {leg.Joints[COXA_ORIGIN_INDEX].X}, Coxa.Y: {leg.Joints[COXA_ORIGIN_INDEX].Y}")

    # Eq 1
    # Calculate coxa, convert to degrees (180.0/np.pi), and apply offset,
    # do the angle

    # servoAngle.Coxa = (180.0/np.pi) * np.atan2(y,x) + 360 - leg.coxa_angle_offset
    # if servoAngle.Coxa >= 180:
    #     servoAngle.Coxa = servoAngle.Coxa - 360

    # servoAngle.Coxa = np.degrees(np.atan2(y, x)) if (x != 0 or y != 0) else 0
    servoAngle.Coxa = np.degrees(np.atan2(y, x)+ 360-leg.coxa_angle_offset) if (x != 0 or y != 0) else 0
    # if servoAngle.Coxa >= 180:
    #     servoAngle.Coxa = servoAngle.Coxa - 360
    #servoAngle.Coxa = max(0, min(360, servoAngle.Coxa))  # Assuming 0-180° range
    # print(f"Coxa.Angle: {servoAngle.Coxa}")

    # Moving to leg Frame
    # Eq 2
    # print("Effector Target X: ", effectorTarget.X)
    # print("Effector Target Y: ", effectorTarget.Y)
    # print(f"Coxa.X: {leg.Joints[COXA_ORIGIN_INDEX].X}, Coxa.Y: {leg.Joints[COXA_ORIGIN_INDEX].Y}")
    dx = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
    dy = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y

    # print("dx: ", dx, " dy: ", dy)
    # print("Segment lengths",leg.segment_length) # comes to 45. 
    L1 = np.sqrt(dx**2 + dy**2) - leg.segment_length.Coxa  # Avoid division by zero
    #L1 = max(0.1, np.sqrt(dx**2 + dy**2) - leg.segment_length.Coxa)  # Avoid division by zero
    L2 = effectorTarget.Z - leg.Joints[FEMUR_ORIGIN_INDEX].Z

    # # Lock femur/Tibia during sliding phase
    # if leg.current_leg_phase == "sliding":
    #     print("sliding")
    #     servoAngle.Femur = -100
    #     servoAngle.Tibia = 20
    # else:
    L = np.sqrt(L2**2 + L1**2)

    print(f"L1: {L1}, L2: {L2}, L: {L}")
    # Eq 3

    alpha_1 = np.arccos(-L2 / L)  # -π/2 to π/2
    # alpha_2 = np.acos(((leg.segment_length.Tibia**2 - 
    #                              leg.segment_length.Femur**2 - 
    #                              L**2) / 
    #                             (-2 * leg.segment_length.Femur * L)))
    alpha_2 = np.arccos(np.clip((leg.segment_length.Tibia**2 - 
                                 leg.segment_length.Femur**2 - 
                                 L**2) / 
                                (-2 * leg.segment_length.Femur * L), -1, 1))
    
    # beta = np.acos((L**2 -
    #                   leg.segment_length.Femur**2 - 
    #                   leg.segment_length.Tibia**2 / 
    #                   (-2 * leg.segment_length.Tibia * leg.segment_length.Femur)))

    
    # Perform offsets for each motor, and convert to angles
    servoAngle.Femur = np.degrees((alpha_1 + alpha_2))  # 0° = up
    servoAngle.Tibia = np.degrees(np.arccos(np.clip(
                              (L**2 -
                              leg.segment_length.Femur**2 - 
                              leg.segment_length.Tibia**2 / 
                                (-2 * leg.segment_length.Tibia * leg.segment_length.Femur)), -1, 1)))              # 0° = straight
    # servoAngle.Femur = max(0, min(180, servoAngle.Femur))
    # servoAngle.Tibia = max(0, min(180, servoAngle.Tibia))


    print(f"Alpha_1: {alpha_1}, Alpha_2: {alpha_2}, Alpha: {servoAngle.Femur}, Beta: {servoAngle.Tibia}")

    print("Servo Angle after IK: ", servoAngle)
    return servoAngle

# def solve_effector_IK(leg: Leg, effectorTarget: Coordinate) -> ServoAngles:
#     servoAngle = ServoAngles()
#     x = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
#     y = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y
#     z = effectorTarget.Z - leg.Joints[COXA_ORIGIN_INDEX].Z

#     servoAngle.Coxa = (180.0/np.pi) * np.atan2(y, x)
#     if servoAngle.Coxa < 0:
#         servoAngle.Coxa += 360
#     servoAngle.Coxa = max(0, min(180, servoAngle.Coxa))

#     L1 = max(0.1, np.sqrt(x**2 + y**2) - leg.segment_length.Coxa)
#     L2 = z
#     L = np.sqrt(L1**2 + L2**2)

#     # if leg.current_leg_phase == "sliding":
#     #     servoAngle.Femur = leg.servo_angles.Femur
#     #     servoAngle.Tibia = leg.servo_angles.Tibia
#     # else:
#     alpha_1 = np.arctan2(-L2, L1)
#     alpha_2 = np.arccos(np.clip((leg.segment_length.Femur**2 + L**2 - leg.segment_length.Tibia**2) / 
#                                 (2 * leg.segment_length.Femur * L), -1, 1))
#     alpha = alpha_1 + alpha_2
#     beta = np.arccos(np.clip((leg.segment_length.Tibia**2 + leg.segment_length.Femur**2 - L**2) / 
#                                 (2 * leg.segment_length.Tibia * leg.segment_length.Femur), -1, 1))

#     servoAngle.Femur = (180.0/np.pi) * (np.pi/2 - alpha)  # 0° = up
#     servoAngle.Tibia = (180.0/np.pi) * beta              # 0° = straight
#     servoAngle.Femur = max(0, min(180, servoAngle.Femur))
#     servoAngle.Tibia = max(0, min(180, servoAngle.Tibia))

#     return servoAngle