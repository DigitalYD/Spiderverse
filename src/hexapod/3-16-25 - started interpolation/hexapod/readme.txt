All files are on the raspi4 in the hexapod. Does not sync with this repo.



    "coxa_angle_offset": {
        "LR": 214.309,
        "LM": 270,
        "LF": 325.931,
        "RF": 34.0685,
        "RM": 90,
        "RR": 147.172
    },



# If doesnt work keep for future bezier work, but make a copy and adjust for interpolation
# def solve_effector_IK(leg, effectorTarget: Coordinate, debug: bool = False) -> ServoAngles:
#     servoAngle = ServoAngles()
    
#     # Coxa angle (rotation in XY plane)
#     x = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
#     y = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y
#     servoAngle.Coxa = np.degrees(np.arctan2(y, x) - leg.coxa_angle_offset)
#     # servoAngle.Coxa = np.degrees(np.arctan2(y, x) + 360 - leg.coxa_angle_offset) % 360
#     if servoAngle.Coxa >= 180:
#         servoAngle.Coxa = 180 - servoAngle.Coxa
#     # servoAngle.Coxa = max(0, min(180, servoAngle.Coxa))  # Limit to 0-180°

#     # Move to leg frame
#     dx = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
#     dy = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y
#     L1 = max(0.1, np.sqrt(dx**2 + dy**2) - leg.segment_length.Coxa)  # Prevent negative length
#     L2 = effectorTarget.Z - leg.Joints[FEMUR_ORIGIN_INDEX].Z
#     L = np.sqrt(L1**2 + L2**2)

#     # Calculate angles with continuity
#     alpha_1 = np.arccos(-L2 / L)
#     alpha_2 = np.arccos(np.clip((leg.segment_length.Tibia**2 - leg.segment_length.Femur**2 - L**2) / 
#                                 (-2 * leg.segment_length.Femur * L), -1, 1))
    
#     servoAngle.Femur = np.degrees(alpha_1 + alpha_2)
#     servoAngle.Femur = max(0, min(180, servoAngle.Femur))  # Limit to 0-180°

#     # Tibia angle with previous state consideration
#     beta = np.arccos(np.clip((L**2 - leg.segment_length.Femur**2 - leg.segment_length.Tibia**2) / 
#                              (-2 * leg.segment_length.Tibia * leg.segment_length.Femur), -1, 1))
#     prev_tibia = leg.servo_angles.Tibia if leg.servo_angles.Tibia is not None else 0
#     servoAngle.Tibia = np.degrees(beta)
#     if abs(servoAngle.Tibia - prev_tibia) > 90:  # Prevent large jumps
#         servoAngle.Tibia = 180 - servoAngle.Tibia
#     servoAngle.Tibia = max(0, min(180, servoAngle.Tibia))  # Limit to 0-180°

#     if debug:
#         print(f"Target: {effectorTarget}, Servo Angles: {servoAngle}")
#         print(f"L1: {L1}, L2: {L2}, L: {L}, Alpha_1: {alpha_1}, Alpha_2: {alpha_2}")

#     return servoAngle


# def solve_effector_IK_Interpolation(leg, effectorTarget:Coordinate, debug:bool = False) -> ServoAngles: 
#     ##--------------------------
#     # Variation 2 for Interpolation
#     # Requires further modifications to better suit interpolation
#     ##--------------------------
#     servoAngle = ServoAngles()
#     # Inverse kin equation 1
#     x = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
#     y = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y

#     # print(f"x: {x}, y: {y}")
#     # print(f"endo_target.X: {effectorTarget.X}, endo_target.Y: {effectorTarget.Y}")
#     # print(f"Coxa.X: {leg.Joints[COXA_ORIGIN_INDEX].X}, Coxa.Y: {leg.Joints[COXA_ORIGIN_INDEX].Y}")

#     # Eq 1
#     # Calculate coxa, convert to degrees (180.0/np.pi), and apply offset,
#     # servoAngle.Coxa = (180.0/np.pi) * np.atan2(y,x) + 360 - leg.coxa_angle_offset
#     # if servoAngle.Coxa >= 180:
#     #     servoAngle.Coxa = servoAngle.Coxa - 360

#     # servoAngle.Coxa = np.degrees(np.atan2(y, x)) if (x != 0 or y != 0) else 0
#     servoAngle.Coxa = np.degrees(np.atan2(y, x)+ 360-leg.coxa_angle_offset) if (x != 0 or y != 0) else 0
#     # if servoAngle.Coxa >= 180:
#     #     servoAngle.Coxa = servoAngle.Coxa - 360
#     #servoAngle.Coxa = max(0, min(360, servoAngle.Coxa))  # Assuming 0-180° range
    
#     # Moving to leg Frame
#     # Eq 2
#     dx = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
#     dy = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y

#     L1 = np.sqrt(dx**2 + dy**2) - leg.segment_length.Coxa  # Avoid division by zero
#     #L1 = max(0.1, np.sqrt(dx**2 + dy**2) - leg.segment_length.Coxa)  # Avoid division by zero
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

#     alpha_1 = np.arccos(-L2 / L)  # -π/2 to π/2
#     # alpha_2 = np.acos(((leg.segment_length.Tibia**2 - 
#     #                              leg.segment_length.Femur**2 - 
#     #                              L**2) / 
#     #                             (-2 * leg.segment_length.Femur * L)))
#     alpha_2 = np.arccos(np.clip((leg.segment_length.Tibia**2 - 
#                                  leg.segment_length.Femur**2 - 
#                                  L**2) / 
#                                 (-2 * leg.segment_length.Femur * L), -1, 1))
    
#     # beta = np.acos((L**2 -
#     #                   leg.segment_length.Femur**2 - 
#     #                   leg.segment_length.Tibia**2 / 
#     #                   (-2 * leg.segment_length.Tibia * leg.segment_length.Femur)))

    
#     # Perform offsets for each motor, and convert to angles
#     servoAngle.Femur = np.degrees((alpha_1 + alpha_2))  # 0° = up
#     servoAngle.Tibia = np.degrees(np.arccos(np.clip(
#                               (L**2 -
#                               leg.segment_length.Femur**2 - 
#                               leg.segment_length.Tibia**2 / 
#                                 (-2 * leg.segment_length.Tibia * leg.segment_length.Femur)), -1, 1)))              # 0° = straight
#     # servoAngle.Femur = max(0, min(180, servoAngle.Femur))
#     # servoAngle.Tibia = max(0, min(180, servoAngle.Tibia))

#     if debug == True:
#         print(f"Prior to making x/y: endo_target.X: {effectorTarget.X}, endo_target.Y: {effectorTarget.Y}")
#         print("Effector Target X: ", effectorTarget.X)
#         print("Effector Target Y: ", effectorTarget.Y)
#         print(f"Coxa.X: {leg.Joints[COXA_ORIGIN_INDEX].X}, Coxa.Y: {leg.Joints[COXA_ORIGIN_INDEX].Y}")
#         print("dx: ", dx, " dy: ", dy)
#         print("Segment lengths",leg.segment_length)
#         print(f"Alpha_1: {alpha_1}, Alpha_2: {alpha_2}, Alpha: {servoAngle.Femur}, Beta: {servoAngle.Tibia}")
#         print("Servo Angle after IK: ", servoAngle)
#     return servoAngle