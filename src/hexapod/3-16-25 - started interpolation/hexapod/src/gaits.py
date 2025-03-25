from typing import List
from dataclasses import dataclass
from enum import Enum

# Define GaitType constants
class GaitType(Enum):
    ''' Enumeration for the gait types used in a multi-legged robot '''
    TRIPOD = 0
    WAVE = 1
    RIPPLE = 2
    TETRAPOD = 3

# GaitPattern is a list of lists of ints. Each inner list represents one leg's pattern across the gait cycle.
GaitPattern = List[List[int]]

@dataclass
class Gait:
    '''Set gait pattern''' 
    name: str
    pattern: GaitPattern
    indices: int
    speed_factor: float

def new_Gait(gait_type: GaitType, speed_factor: float = 0) -> Gait:
    
    if gait_type == GaitType.TRIPOD:
        if speed_factor == 0:
            speed_factor = 1.0
        tripod_pattern = [
            [1, 0],  # Leg 0 swings in phase 0 (e.g., Left Front)
            [0, 1],  # Leg 1 swings in phase 1 (e.g., Right Front)
            [1, 0],  # Leg 2 swings in phase 0 (e.g., Left Middle)
            [0, 1],  # Leg 3 swings in phase 1 (e.g., Right Middle)
            [1, 0],  # Leg 4 swings in phase 0 (e.g., Left Rear)
            [0, 1]   # Leg 5 swings in phase 1 (e.g., Right Rear)
        ]
        return Gait(name="Tripod", pattern=tripod_pattern, indices=2, speed_factor=speed_factor)
    
    elif gait_type == GaitType.WAVE:
        if speed_factor == 0:
            speed_factor = 0.19
        wave_pattern = [
            [1, 0, 0, 0, 0, 0],  # Leg 0 swings at step 0
            [0, 1, 0, 0, 0, 0],  # Leg 1 swings at step 1
            [0, 0, 1, 0, 0, 0],  # Leg 2 swings at step 2
            [0, 0, 0, 1, 0, 0],  # Leg 3 swings at step 3
            [0, 0, 0, 0, 1, 0],  # Leg 4 swings at step 4
            [0, 0, 0, 0, 0, 1]   # Leg 5 swings at step 5
        ]
        return Gait(name="Wave", pattern=wave_pattern, indices=6, speed_factor=speed_factor)
    
    elif gait_type == GaitType.TETRAPOD:
        if speed_factor == 0:
            speed_factor = 0.4
        tetrapod_pattern = [
            [1, 0, 0],  # Leg 0 swings at step 0
            [0, 0, 1],  # Leg 1 swings at step 2
            [0, 1, 0],  # Leg 2 swings at step 1
            [1, 0, 0],  # Leg 3 swings at step 0
            [0, 0, 1],  # Leg 4 swings at step 2
            [0, 1, 0]   # Leg 5 swings at step 1
        ]
        return Gait(name="Tetrapod", pattern=tetrapod_pattern, indices=3, speed_factor=speed_factor)
    
    elif gait_type == GaitType.RIPPLE:
        if speed_factor == 0:
            speed_factor = 0.4
        ripple_pattern = [
            [1, 0, 0, 0],  # Leg 0 swings at step 0
            [0, 1, 0, 0],  # Leg 1 swings at step 1
            [0, 0, 1, 0],  # Leg 2 swings at step 2
            [0, 0, 0, 1],  # Leg 3 swings at step 3
            [1, 0, 0, 0],  # Leg 4 swings at step 0 (with Leg 0)
            [0, 1, 0, 0]   # Leg 5 swings at step 1 (with Leg 1)
        ]
        return Gait(name="Ripple", pattern=ripple_pattern, indices=4, speed_factor=speed_factor)