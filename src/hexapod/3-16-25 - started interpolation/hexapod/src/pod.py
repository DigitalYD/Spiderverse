import time
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from src.leg import Leg, SegmentLengths,ServoAngles, Servo
from src.hex_body import new_hexapod_body, Body
from dataclasses import dataclass, field
from typing import List, Optional, Dict
from src.gaits import new_Gait, GaitType, Gait
from src.config import FORWARD, REVERSE, POD_Z_HEIGHT, INTERPOLATION_STEPS,EFFECTOR_ORIGIN_INDEX, NUM_LEGS
from src.inversekinematics import solve_effector_IK
from src.coord import Coordinate, new_Coordinate
from src.bezier2d import BezierCurve

@dataclass
class Pod:
    """Pod represents a multi-legged robot (e.g., hexapod, pentapod) and manages its legs and movement state."""
    body_def: Body                    # Robot body parameters (number of legs, geometry, gait, etc.)
    Legs: List[Leg] = field(default_factory=list)     # List of legs composing the robot
    has_stride: bool = False                          # True if a movement vector or rotation is defined (stride planning)
    isWalking: bool = False                           # True if the robot is currently walking
    currentMode:str = "standing"                       # Current Mode. "Neutral", "Walking", "changeElevation"
    targetgaitCycles: int = 0                         # Target number of gait cycles to execute (0 for infinite until stopped)
    currentgaitCycle: int = 0                         # Counter for how many gait cycles have been completed
    currentgaitIndex: int = 0                         # Current index in the gait pattern cycle
    isReverting: bool = False                         # True if the robot is reverting legs to neutral stance
    revertinglegIndex: int = 0                        # Index of the leg currently being reverted (in neutral-return process)
    revertPhase: int = 0                              # Current phase of revert process (0 = Ground phase, 1 = MoveToNeutral phase)
    direction: int = FORWARD                          # Current movement direction (Forward or Reverse)
    tick: int = 0                                     # tick counter for movement updates
    gait: Optional[Gait] = None
    phase_shifts: list[int] = field(default_factory=lambda: [0] * NUM_LEGS)
    body_position: list[float] = field(default_factory= lambda: [0.0, 0.0, 0.0])
    leg_phase_offsets = [0] * NUM_LEGS  
    leg_phase_progress = [0.0] * NUM_LEGS
    gait_step_duration = 20  # Frames per gait phase
    current_step = 0
    phase_progress = 0.0
    pod_height: int = POD_Z_HEIGHT
    
    def __post_init__(self):
        """Initialize hexapod legs with servos."""
        offset_matrix = {}
        for i in range(self.body_def.num_legs):  # Six legs
        # Example transformation matrix (identity for now)
            offset_matrix[i] = np.array([
                [np.cos(self.body_def.coxa_offsets[i] * np.pi/180), -np.sin(self.body_def.coxa_offsets[i]*np.pi/180), 0, self.body_def.coxa_coords[i].X],  # x translation
                [np.sin(self.body_def.coxa_offsets[i]*np.pi/180), np.cos(self.body_def.coxa_offsets[i]*np.pi/180), 0, self.body_def.coxa_coords[i].Y],    # y translation
                [0, 0, 1, self.body_def.coxa_coords[i].Z],      # z translation (none)
                [0, 0, 0, 1]       # homogeneous row
                ])
            # print(f"Leg {i} Coxa Offset: {self.body_def.coxa_offsets[i]}, Coxa Cord {self.body_def.coxa_coords[i]}, offset matrix = {offset_matrix[i]}")

        # Generate six legs
        self.Legs = [
            Leg(
                Index=i,
                Name=self.body_def.leg_names[i],
                # Coxa=Servo(i * 3, pca=0x40 if i * 3 < 9 else 0x41),
                # Femur=Servo(i * 3 + 1, pca=0x40 if i * 3 + 1 < 9 else 0x41),
                # Tibia=Servo(i * 3 + 2, pca=0x40 if i * 3 + 2 < 9 else 0x41),
                coxa_position = self.body_def.coxa_coords[i],
                coxa_angle_offset=self.body_def.coxa_offsets[i],
                offset_transformation_matrix=offset_matrix[i],
                segment_length=self.body_def.leg_segments[i],
                servo_angles=self.body_def.rest_angles[i],
                neutral_effector_coord = Coordinate(0, 0, self.pod_height)
            )
            for i in range(self.body_def.num_legs)
        ]
        self.gait = new_Gait(GaitType.TRIPOD) # Default Gait Tripod

    # Property & Setter Allows the height of the hexapod to be adjusted!
    # Note the further you set this down below ~70, the futher you need to put the leg tips out.
    @property
    def height(self):
        return self.pod_height
    
    @height.setter
    def height(self, new_height):
        self.pod_height = new_height

    def set_direction(self, direction:int) -> None:
        ''' Set the movement direction '''
        self.direction = direction
    
    def reverse_direction(self) -> None:
        self.direction = REVERSE if self.direction == FORWARD else FORWARD
    
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
        ''' Update and Recalculate the offset transformation matrices for each leg and modify the leg objects '''
        for i, leg in enumerate(self.body_def.num_legs):
            # print("Update leg {i}, object {leg}")

            # create a new offset transformation matrix matrix for each leg
            # VERIFY THIS CODE, might have coxa offset mixed up w/ another var
            # copied from leg tests
            offset_matrix = np.array([
                        [np.cos(self.body_def.coxa_offset[i] * np.pi/180), -np.sin(self.body_def.coxa_offset[i] * np.pi/180), 0, self.body_def.coxa_coord.X],  # x translation
                        [np.sin(self.body_def.coxa_offset[i] * np.pi/180), np.cos(self.body_def.coxa_offset[i] * np.pi/180), 0, self.body_def.coxa_coord.Y],    # y translation
                        [0, 0, 1, self.body_def.coxa_coord.Z],      # z translation (none)
                        [0, 0, 0, 1]       # homogeneous row
                        ])

            self.Legs[i] = Leg(
                Index=i,
                Name=f"Leg {i}",
                #Setup Servo Motors ||| UNCOMMENT ON HEXAPOD |||
                Coxa=Servo(i * 3, pca=0x40 if i * 3 < 9 else 0x41),  # Assign PCA based on ID
                Femur=Servo(i * 3 + 1, pca=0x40 if i * 3 + 1 < 9 else 0x41),
                Tibia=Servo(i * 3 + 2, pca=0x40 if i * 3 + 2 < 9 else 0x41),
                coxa_angle_offset=self.body_def.coxa_offset[i],
                offset_transformation_matrix=offset_matrix, # Creates 4x4 identity matrix for an insert
                segment_length=self.body_def.leg_segments[i],
                servo_angles=self.body_def.rest_angles[i],  # Provide default servo angles
                neutral_effector_coord = Coordinate(0,0, self.pod_height) # From Leg test (May Change)
            )

    def load_body_def(self, body_def: Body) -> None:
        self.body_def = body_def
        self.direction = FORWARD
        self.UpdatePodStructure()

    def set_gait(self, gait_type: GaitType, speed_factor: float = 0, phase_shifts: List[int] = None):
        """Set the gait pattern and optional phase shifts."""
        self.gait = new_Gait(gait_type, speed_factor)
        if phase_shifts is not None:
            self.phase_shifts = phase_shifts
        self.body_def.set_gait = self.gait
        self.currentgaitIndex = 0
        self.currentgaitCycle = 0
        self.isWalking = True

        if phase_shifts is not None:
            self.phase_shifts = phase_shifts
        else:
            if gait_type == GaitType.WAVE:
                self.phase_shifts = [0, 1, 2, 3, 4, 5]
            elif gait_type == GaitType.RIPPLE:
                self.phase_shifts = [0, 1, 2, 0, 1, 2]
            elif gait_type == GaitType.TRIPOD:
                self.phase_shifts = [0, 1, 0, 1, 0, 1]
            elif gait_type == GaitType.RIPPLE:
                self.phase_shifts = [0, 1, 2, 0, 1, 2]
            elif gait_type.name.upper() == "TETRAPOD":
                self.phase_shifts = [0, 2, 1, 0, 2, 1]  # Custom shift for tetrapod
            else:
                self.phase_shifts = [0] * NUM_LEGS  # fallback
        
        for i, leg in enumerate(self.Legs):
            leg.servo_angles = solve_effector_IK(leg, new_Coordinate(leg.neutral_effector_coord.X, leg.neutral_effector_coord.Y, leg.neutral_effector_coord.Z))
            leg.recalculate_forward_kinematics(leg.servo_angles)

    def slide_hexapod_forward(self):
        ''' Propel the hexapod's position forward using "sliding" dict and "start"
            from bezier curve. Calculate the distance between the points and add it to hexapods
            position
        '''
        start_point = self.Legs[0].bezier_curve.get_named_point("start")
        sliding_point = self.Legs[0].bezier_curve.get_named_point("sliding")
        
        if start_point is not None and sliding_point is not None:
            # Calculate the sliding vector as the difference between sliding and start points
            sliding_vector = sliding_point - start_point
            # Update body position based on the sliding vector and direction
            self.body_position[0] += sliding_vector[0] * self.direction
            self.body_position[1] += sliding_vector[1] * self.direction
            self.body_position[2] += sliding_vector[2] * self.direction
            print(f"Hexapod slid to {self.body_position}")
        
        
    def perform_gait(self, num_cycles: int = 1):
        '''
            Execute gait cycle with phase shifts and forward propulsion.
        '''
        self.targetgaitCycles = num_cycles
        self.isWalking = True
        base_time = 0.75 # (time per step in seconds)
        time_per_step = base_time / self.gait.speed_factor
        
        while self.isWalking and (self.targetgaitCycles == 0 or self.currentgaitCycle < self.targetgaitCycles):
            for s in range(self.gait.indices):
                #print(f"\nStep {s}:")
                legs_moved = False
                for j, leg in enumerate(self.Legs):
                    # Apply phase shift
                    shifted_step = (s + self.phase_shifts[j]) % self.gait.indices
                    if self.gait.pattern[j][shifted_step] == 1:
                        # Move leg along Bezier curve until "Sliding"
                        if leg.t < leg.swing_fraction:
                            leg.update(time_per_step, self.direction, self.gait.pattern[j], shifted_step)
                            legs_moved = True
                        else:
                            # Reset t for next swing
                            leg.t = 0.0
                            legs_moved = True
                
                # Propel forward if legs moved
                if legs_moved:
                    self.slide_hexapod_forward()
                
                time.sleep(time_per_step)

                # Check if all legs have completed their step
                all_done = all(leg.t == 0.0 or leg.t >= leg.swing_fraction for leg in self.Legs if self.gait.pattern[self.Legs.index(leg)][(s + self.phase_shifts[self.Legs.index(leg)]) % self.gait.indices] == 1)
                if all_done:
                    self.currentgaitIndex = (self.currentgaitIndex + 1) % self.gait.indices
                    if self.currentgaitIndex == 0:
                        self.currentgaitCycle += 1
                        if self.targetgaitCycles and self.currentgaitCycle >= self.targetgaitCycles:
                            self.isWalking = False
                    self.tick += 1 

    def update(self) -> List[Coordinate]:
        ''' Advance walking movement with proper gait '''
        # if no gait or not walking
        if not self.gait or not self.isWalking:
            return [leg.effector_target for leg in self.Legs]

        foot_targets = []
        step_complete = True
        delta_idx = 1
        
        # print(f"Gait Pattern: {self.gait.pattern}")  # Debug gait pattern
        # print(f"Current Gait Index: {self.currentgaitIndex}")

        for i, leg in enumerate(self.Legs):
            phase_idx = self.currentgaitIndex % self.gait.indices
            is_swing = self.gait.pattern[i][phase_idx] == 1
            #phase_idx = self.currentgaitIndex
            
            # Add in extra code for "if starting position"
            if self.currentMode == "standing" and leg.currentlegPhase == "neutral": # if starting from standing position
                return [leg.effector_target for leg in self.Legs]
            else:
                # cp = leg.bezier_curve.control_points_dict
                if not hasattr(leg, 'step_idx'):
                    leg.step_idx = 0
                if not hasattr(leg, 'current_phase'):
                    leg.current_phase = is_swing

                # If the phase changes, reset step_idx and update the flag
                if leg.current_phase != is_swing:
                    leg.step_idx = 0
                    leg.current_phase = is_swing

                if is_swing:
                    leg.currentlegPhase = "swinging"
                    swing_curve = leg.bezier_curve.get_points_between("start", "touchdown")
                    total_points = len(swing_curve)
                    pos = swing_curve[min(leg.step_idx, total_points - 1)]
                else:
                    stance_curve = leg.bezier_curve.get_points_between("touchdown", "return")
                    total_points = len(stance_curve)
                    pos = stance_curve[min(leg.step_idx, total_points - 1)]
                    neutral = leg.bezier_curve.get_point(leg.bezier_curve.num_points)
                    if np.allclose(pos, neutral, atol=1e-3):
                        leg.currentlegPhase = "neutral"
                        print(f"leg {i} neutral")
                    else:
                        leg.currentlegPhase = "returning"
                        print(f"leg {i} returning")
            # Only advance if not at the end of the curve
            if leg.step_idx < total_points - 1:
                leg.step_idx += delta_idx
                step_complete = False
            else:
                # Hold position until next phase triggers a reset
                leg.step_idx = total_points - 1

            foot_target = new_Coordinate(pos[0], pos[1], pos[2])
            leg.effector_target = foot_target
            angles = solve_effector_IK(leg, foot_target)
            leg.recalculate_forward_kinematics(angles)
            foot_targets.append(foot_target)
            # print(f"Leg {i}: step_idx={leg.step_idx}, Swing={is_swing}, PhaseIdx={phase_idx}, Pos={pos}, CurveLen={total_points}")

        # Advance gait index only when all legs finished their motion
        if step_complete: 
            self.slide_hexapod_forward() # Updates hexapod position in code may not be needed or change how this works.
            self.currentgaitIndex = (self.currentgaitIndex + 1) % self.gait.indices
            # print(f"Advanced gait index to {self.currentgaitIndex}")
            if self.currentgaitIndex == 0:
                self.currentgaitCycle += 1
                if self.targetgaitCycles and self.currentgaitCycle >= self.targetgaitCycles:
                    self.isWalking = False

        return foot_targets
    

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

    def set_stride_vector(self, nrepeats:int, x:float, y:float):
        ''' Sets up the path of a single step and calculates the intermediate angles needed to complete it in the direction of the vector w/ stride length equal to length of vector '''
        self.targetgaitCycles = nrepeats
        for l, leg in enumerate(self.Legs):
            #print(l, leg)
        
            # get effector origin
            effector_origin = leg.Joints[EFFECTOR_ORIGIN_INDEX]
            xMax = effector_origin.X + x
            xMin = effector_origin.X - x
            yMax = effector_origin.Y + y
            yMin = effector_origin.Y - y

            # For visualization
            xstep = (xMax - xMin) / (INTERPOLATION_STEPS - 1)
            ystep = (yMax - yMin) / (INTERPOLATION_STEPS - 1)
            deltaX = 0
            deltaY = 0
            
            # Calculate coordinates
            for i in range(INTERPOLATION_STEPS):
                swing = new_Coordinate(xMin + deltaX, yMin + deltaY, POD_Z_HEIGHT)
                servoAngles = solve_effector_IK(leg, swing)
                #revise below
                leg.intermediate_angles.JointAngle[0][leg] = servoAngles.Coxa
                leg.intermediate_angles.JointAngle[0][leg] = servoAngles.Coxa
                leg.intermediate_angles.JointAngle[0][leg] = servoAngles.Coxa
                leg.intermediate_effector_coordinates[i] = new_Coordinate(deltaX + xMin, deltaY + yMin, 0)
                deltaX += xstep
                deltaY += ystep

        self.has_stride = True

    def set_rotation(self, nrepeats:int, degrees:float):
        ''' Sets up the path of a single step, instead of following a vector it will follow a curve segment '''
        self.targetgaitCycles = nrepeats

        for l, leg in enumerate(self.Legs):
            # print(l, leg)

            # get effector
            effector_origin = leg.Joints[EFFECTOR_ORIGIN_INDEX]

            # setup radius 
            radius = np.sqrt(effector_origin.X**2 + effector_origin.Y**2)
            stepRad = np.deg2rad(degrees) / (INTERPOLATION_STEPS - 1)
            angle = np.atan2(effector_origin.Y, effector_origin.X) - 0.5 * np.deg2rad(degrees)
            # using atan2 shouldn't have errors

            delta = 0.0
            for i in range(INTERPOLATION_STEPS):

                x = radius * np.cos(angle + delta)
                y = radius * np.sin(angle + delta)
                swing = new_Coordinate(x, y, POD_Z_HEIGHT)
                servoAngles = solve_effector_IK(self.leg[l], swing)
                leg.intermediate_angles.Coxa = servoAngles.Coxa
                leg.intermediate_angles.Femur = servoAngles.Femur
                leg.intermediate_angles.Tibia = servoAngles.Tibia

    def start(self):
        ''' Allows calls to update to start cycling '''
        self.isWalking = True
        self.currentMode = "walking"


    def stop(self):
        ''' stop the gait and walking '''
        self.isWalking = False
        self.targetgaitCycles = self.currentgaitCycle
        self.currentgaitCycle = 0

    def is_swinging(self, leg_id:int):
        ''' Returns true if the leg is currently in swing phase '''
        return self.body_def.Gait.pattern[leg_id][self.currentgaitIndex] == 1
    
    def update_movement(self):
        ''' Cycles through each set of legs, updates swing and stance indices, scan through each grouping, if a group is finished, move to next to complete a "cycle" '''
        end_of_swing:bool

        # iterate through legs
        for i, leg in enumerate(self.Legs):
            if (self.body_def.Gait.pattern)[i][self.currentgaitIndex] == 1:
                end_of_swing = leg.update_stance(self.direction)
            else:
                leg.update_stance(self.direction, self.body_def.gait.speed_factor)

        # if the end of swing and direction is forward gait index ++ else reverse -- moving hexapod backward/forward
        if end_of_swing and self.direction == 1:
            self.currentgaitIndex += 1
        elif end_of_swing and self.dirction == -1:
            self.currentgaitIndex -= 1

        ## Finish program here, if gait index > number of indeces in pattern, reset current gait index, increase gait cycle
        ## if current cycle > target cycles and target cycles != 0 walking is false
        # do same for reverse direction

    
    def move_leg_bezier_test(self):
        ''' Test Function to move the leg using a bezier curve'''
        # for i,leg in enumerate(self.Legs):
        #     print(leg)
        #     leg.move_leg_with_bezier()
        self.Legs[1].move_leg_with_bezier()
        
        # for i in range(6):
        #     print(f"Servo Index name {i}: {self.Legs[i].Name}")
        #     print(f"Servo Index Leg {i}: {self.Legs[i].Coxa.servo_index}, {self.Legs[i].Femur.servo_index}, {self.Legs[i].Tibia.servo_index}")
        #     print(f"PCA Index Leg {i}: {self.Legs[i].Coxa.pca_index}, {self.Legs[i].Femur.pca_index}, {self.Legs[i].Tibia.pca_index}")
        #     print(f"PCA Object Leg {i}: {self.Legs[i].Coxa.pca}, {self.Legs[i].Femur.pca}, {self.Legs[i].Tibia.pca}")


def new_pod(body_def:Body) -> Pod:
    # gait, _ = new_Gait(GaitType.TRIPOD)
    pod = Pod(body_def)
    return pod

if __name__ == '__main__':
    teset_hexapod = new_pod(new_hexapod_body())
    # print(teset_hexapod)