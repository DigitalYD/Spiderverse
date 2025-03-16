import time
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from leg import *
from hex_body import *
from coord import *
from dataclasses import dataclass, field
from typing import List, Optional, Dict
from gaits import *


# Direction constants for leg movement (forward or reverse in gait cycle)
Forward = 1
Reverse = -1

# Set distance from base reference frame Z to end effector Z when robot is in rest/neutral
pod_z_height:float 

# Define maxheight of arc (May not be needed due to bezier curve)
z_lift:float = 70.0

# Define max height of arc by end effector when in neutral/rest pos
revert_lift:float = 50


@dataclass
class Pod:
    """Pod represents a multi-legged robot (e.g., hexapod, pentapod) and manages its legs and movement state."""
    body_def: Body                    # Robot body parameters (number of legs, geometry, gait, etc.)
    Legs: List[Leg] = field(default_factory=list)     # List of legs composing the robot
    has_stride: bool = False                          # True if a movement vector or rotation is defined (stride planning)
    isWalking: bool = False                           # True if the robot is currently walking
    targetgaitCycles: int = 0                         # Target number of gait cycles to execute (0 for infinite until stopped)
    currentgaitCycle: int = 0                         # Counter for how many gait cycles have been completed
    currentgaitIndex: int = 0                         # Current index in the gait pattern cycle
    isReverting: bool = False                         # True if the robot is reverting legs to neutral stance
    revertinglegIndex: int = 0                        # Index of the leg currently being reverted (in neutral-return process)
    revertPhase: int = 0                              # Current phase of revert process (0 = Ground phase, 1 = MoveToNeutral phase)
    direction: int = Forward                          # Current movement direction (Forward or Reverse)
    tick: int = 0                                     # Global tick counter for movement updates
    gait: Optional[Gait] = None

    def __post_init__(self):
        """Initialize hexapod legs with servos."""
        # Example transformation matrix (identity for now)
        identity_matrix = np.eye(4)

        # Predefined coxa angle offsets (modify as needed)
        coxa_angle_offsets = self.body_def.coxa_offset # Angles for leg positioning

        # Generate six legs
        self.Legs = [
            Leg(
                Index=i,
                Name=f"Leg {i}",
                # Coxa=Servo(i * 3, pca=0x40 if i * 3 < 8 else 0x41),
                # Femur=Servo(i * 3 + 1, pca=0x40 if i * 3 + 1 < 8 else 0x41),
                # Tibia=Servo(i * 3 + 2, pca=0x40 if i * 3 + 2 < 8 else 0x41),
                coxa_angle_offset=coxa_angle_offsets[i],
                offset_transformation_matrix=identity_matrix,
                segment_length=SegmentLengths(45, 110, 193),
                servo_angles=ServoAngles(),
                neutral_effector_coord=Coordinate(0, 0, 0)
            )
            for i in range(6)  # Six legs
        ]
        self.gait = new_Gait(GaitType.TRIPOD) # Default Gait Tripod

    def set_direction(self, direction:int) -> None:
        ''' Set the movement direction '''
        self.direction = direction
    
    def reverse_direction(self) -> None:
        self.direction = Reverse if self.direction == Forward else Forward

    def get_current_gait_cycle(self) -> int:
        ''' Returns the number of gait cycles completed so far '''
        return self.currentGaitCycle
    
    def set_coxa_length(self, leg_num:int, length:float) -> None:
        ''' Dynamically change length of coxa for given leg '''
        if leg_num > self.body_def.num_legs - 1:
            raise IndexError(f"Unable to modify leg {leg_num}, the body only has {self.body_def.num_legs}. ")
        self.body_def.leg_segments[leg_num].Coxa = length
        self.UpdatePodStructure()

    def set_femur_length(self, leg_num:int, length:float) -> None:
        ''' Dynamically change length of femu for given leg '''
        if leg_num > self.body_def.num_legs - 1:
            raise IndexError(f"Unable to modify leg {leg_num}, the body only has {self.body_def.num_legs}. ")
        self.body_def.leg_segments[leg_num].Femur = length
        self.UpdatePodStructure()

    def set_tibia_length(self, leg_num:int, length:float) -> None:
        ''' Dynamically change length of tibia for given leg '''
        if leg_num > self.body_def.num_legs - 1:
            raise IndexError(f"Unable to modify leg {leg_num}, the body only has {self.body_def.num_legs}. ")
        self.body_def.leg_segments[leg_num].Tibia = length
        self.UpdatePodStructure()

    def set_coxa_angle(self, leg_num:int, angle:float)->None:
        ''' Modify resting (neutral) agnle of coxa for given leg'''
        if leg_num > self.body_def.num_legs - 1:
            raise IndexError(f"Unable to modify leg {leg_num}, the body only has {self.body_def.num_legs}. ")
        self.body_def.rest_angles[leg_num].Coxa = angle
        self.UpdatePodStructure()

    def set_femur_angle(self, leg_num:int, angle:float)->None:
        ''' Modify resting (neutral) agnle of coxa for given leg'''
        if leg_num > self.body_def.num_legs - 1:
            raise IndexError(f"Unable to modify leg {leg_num}, the body only has {self.body_def.num_legs}. ")
        self.body_def.rest_angles[leg_num].Femur = angle
        self.UpdatePodStructure()

    def set_tibia_angle(self, leg_num:int, angle:float)->None:
        ''' Modify resting (neutral) agnle of coxa for given leg'''
        if leg_num > self.body_def.num_legs - 1:
            raise IndexError(f"Unable to modify leg {leg_num}, the body only has {self.body_def.num_legs}. ")
        self.body_def.rest_angles[leg_num].Tibia = angle
        self.UpdatePodStructure()
    
    def UpdatePodStructure(self) -> None:
        ''' Recalculate the offset transformation matrices for each leg and modify the leg objects '''
        pass


    def load_body_def(self, body_def: Body) -> None:
        self.body_def = body_def
        self.direction = Forward
        self.UpdatePodStructure()

    def revert_to_neutral(self) -> None:
        '''If in reverting state, move all legs backward to neutral stance'''


    def set_gait(self, gait_type: GaitType, speed_factor: float=0):
        ''' set the gait pattern and index to 0'''
        self.gait = new_Gait(gait_type, speed_factor)
        self.currentgaitIndex = 0

    
    def update(self) -> None:
        ''' Main sequence to be called in a loop, advances walking movement or reverts to neutral '''
        if not self.gait:
            return
        
        # Adjust delta for forward and backwards movement, speed factor (how fast we want to move the leg), and the direction (forward:1, back:-1)
        delta_t = 0.05 * self.gait.speed_factor * self.direction
        
        if self.isWalking:
            all_done = True
            for i, leg in enumerate(self.Legs):
                if leg.update(delta_t, self.direction, self.gait.pattern[i], self.currentgaitIndex):
                    all_done = False
            if all_done:
                self.currentgaitIndex = (self.currentgaitIndex + 1) % self.gait.indices
                if self.currentgaitIndex == 0:
                    self.currentgaitCycle += 1
                    if self.targetgaitCycles and self.currentgaitCycle >= self.targetgaitCycles:
                        self.isWalking = False
                self.tick += 1
        elif self.isReverting:
            all_done = True
            for leg in self.Legs:
                if leg.reset_to_neutral(delta_t):
                    all_done = False
            self.isReverting = not all_done

    def rotate_in_place(self, theta: float = 2*np.pi):
        ''' rotate hexapod in place over multiple steps '''
        steps = int(theta / (np.pi / 3)) # 6 steps for 360 degrees
        delta_t = 0.05 * self.gait.speed_factor

        for _ in range(steps):
            all_done = False
            while not all_done:
                all_done = True
                for leg in self.Legs:
                    if leg.update_rotation(delta_t, self.direction):
                        all_done = False
                time.sleep(0.02)


    def stop(self) -> None:
        ''' Starts moving all legs back to neutral position (if a stride has been set)'''
        self.targetgaitCycles = self.currentgaitCycle

    def Zero(self) -> None:
        ''' Set all legs to zero angles (straight out) for the entire pod'''
        for leg in self.Legs:
            leg.Zero()





def new_pod(body_def:Body) -> Pod:
    # gait, _ = new_Gait(GaitType.TRIPOD)
    pod = Pod(body_def)
    return pod

if __name__ == '__main__':
    teset_hexapod = new_pod(new_hexapod_body())
    print(teset_hexapod)