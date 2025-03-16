
from coord import ServoAngles, Coordinate
from leg import Leg, COXA_ORIGIN_INDEX, FEMUR_ORIGIN_INDEX
import numpy as np

def solve_effector_IK(leg:Leg, effectorTarget:Coordinate) -> ServoAngles:

    ##--------------------------
    #### Variation 2 (Remake 3-7-2025)
    ##--------------------------
    # Assumed neurtral position is at 0 degrees, else zero offset is required
    servoAngle = ServoAngles()
    # Inverse kin equation 1
    # Body frame
    x = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
    y = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y


    # Eq 1
    # Calculate coxa, convert to degrees (180.0/np.pi), and apply offset,
    servoAngle.Coxa = (180.0/np.pi)*np.atan2(y,x) + 360 - leg.coxa_angle_offset
    if servoAngle.Coxa >= 180:
        servoAngle.Coxa = servoAngle.Coxa - 360

    
    # Moving to leg Frame
    # Eq 2

    dx = effectorTarget.X - leg.Joints[COXA_ORIGIN_INDEX].X
    dy = effectorTarget.Y - leg.Joints[COXA_ORIGIN_INDEX].Y

    L1 = np.sqrt((dx**2 + dy**2) - leg.segment_length.Coxa)
    L2 = effectorTarget.Z - leg.Joints[FEMUR_ORIGIN_INDEX].Z
    L = np.sqrt(L2**2 + L1**2)
    
    # Eq 3
    arg_1 = (L2/L1)
    alpha_1 =  np.clip(np.arccos(arg_1),-1,1) # Clip result to prevent errors

    # Eq 4
    arg_2 = (leg.segment_length.Tibia**2 - leg.segment_length.Femur**2 - L**2)/(-2 * leg.segment_length.Femur * L)
    alpha_2 = np.clip(np.arccos(arg_2), -1,1) # Clip result to prevent errors

    # Eq 5
    alpha = alpha_1 + alpha_2
    
    # Eq 6
    arg_b = (L**2 - leg.segment_length.Femur**2 - leg.segment_length.Tibia**2)/(-2*leg.segment_length.Tibia*leg.segment_length.Femur)
    beta = np.clip(np.arccos(arg_b) ,-1,1)
    
    # Perform offsets for each motor, and convert to angles
    servoAngle.Femur = 90 - (180.0/np.pi)* alpha
    servoAngle.Tibia= (180.0/np.pi) * beta

    return servoAngle