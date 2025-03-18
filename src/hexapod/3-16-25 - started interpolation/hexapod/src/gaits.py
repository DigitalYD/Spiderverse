from typing import List
from dataclasses import dataclass
from enum import Enum

# Define GaitType constants
class GaitType(Enum):
    ''' Enumeration for the gait types used in a multi-legged robot '''
    TRIPOD = 0
    WAVE = 1
    RIPPLE = 2

# GaitPattern is a list of lists of ints. Each inner list represents one leg's pattern across the gait cycle.
GaitPattern = List[List[int]]

@dataclass
class Gait:
    '''Set gait pattern''' 
    name: str
    pattern: GaitPattern
    indices: int
    speed_factor: float

def new_Gait(gait_type:GaitType, speed_factor:float=0) -> Gait:
    if gait_type == GaitType.TRIPOD:
        if speed_factor == 0:
            speed_factor = 1
        #pattern =  [["LR", "RM", "LF"], ["RF","LM","RR"]]
        tripod_pattern = [
            [0, 1],  # Leg 0 swings in phase 1 (second step)
            [1, 0],  # Leg 1 swings in phase 0 (first step)
            [0, 1],  # Leg 2 swings in phase 1
            [1, 0],  # Leg 3 swings in phase 0
            [0, 1],  # Leg 4 swings in phase 1
            [1, 0]   # Leg 5 swings in phase 0
        ]
        return Gait(speed_factor=speed_factor, name="Tripod", pattern=tripod_pattern, indices=2)
    
    elif gait_type == GaitType.WAVE:
        if speed_factor == 0:
            speed_factor = 0.19
        wave_pattern =  [
        [1, 0, 0, 0, 0, 0],  # Leg 0 swings at step 0
        [0, 1, 0, 0, 0, 0],  # Leg 1 swings at step 1
        [0, 0, 1, 0, 0, 0],  # Leg 2 swings at step 2
        [0, 0, 0, 1, 0, 0],  # Leg 3 swings at step 3
        [0, 0, 0, 0, 1, 0],  # Leg 4 swings at step 4
        [0, 0, 0, 0, 0, 1]   # Leg 5 swings at step 5
        ]
        return Gait(speed_factor=speed_factor, name="Wave", pattern=wave_pattern, indices=6)
    else: # Gait type is ripple
        if speed_factor == 0:
            speed_factor = .4
        ripple_pattern = [
            [0, 1, 0],  # Leg 0 swings during step 1 (pattern: stance, swing, stance)
            [0, 0, 1],  # Leg 1 swings during step 2
            [1, 0, 0],  # Leg 2 swings during step 0
            [0, 1, 0],  # Leg 3 swings during step 1 (synchronized with leg 0)
            [0, 0, 1],  # Leg 4 swings during step 2 (synchronized with leg 1)
            [1, 0, 0]   # Leg 5 swings during step 0 (synchronized with leg 2)
        ]
        return Gait(speed_factor=speed_factor, name="Ripple", pattern=ripple_pattern, indices=3)