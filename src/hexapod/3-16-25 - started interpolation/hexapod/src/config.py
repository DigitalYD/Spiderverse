# Constants related to leg joints and interpolation (from leg.go and pod.go)
NUM_JOINTS = 4  # number of joints in a leg (Coxa, Femur, Tibia, End Effector)
COXA_ORIGIN_INDEX = 0
FEMUR_ORIGIN_INDEX = 1
TIBIA_ORIGIN_INDEX = 2
EFFECTOR_ORIGIN_INDEX = 3

INTERPOLATION_STEPS = 21  # number of interpolation steps for leg movement (must be odd for symmetry)
REVERT_LIFT = 20.0        # height of arc for end effector when reverting to neutral

# Direction constants for leg movement (forward or reverse in gait cycle)
Forward = 1
Reverse = -1

# Set distance from base reference frame Z to end effector Z when robot is in rest/neutral
POD_Z_HEIGHT:float = 50

# height of arc for end effector during swing phase
z_lift:float = 70.0

# Define max height of arc by end effector when in neutral/rest pos
revert_lift:float = 50

# Odd number of interpolation steps
INTERPOLATION_STEPS = 21