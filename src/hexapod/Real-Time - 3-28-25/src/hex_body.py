import json
from dataclasses import dataclass, field
from typing import List, Optional
from src.coord import Coordinate
from src.gaits import new_Gait
from src.leg import SegmentLengths, ServoAngles
from src.gaits import Gait, GaitType

@dataclass
class Body:
    '''
        gaits: tripod, wave, ripple

        LR      LM      LF
        [0]     [1]     [2]
            \\    |    /
            ----------
            ----------
            /     |    \\
        [5]     [4]      [3]
        RR      RM       RF

        hook up legs 5 & 0 at pins 0 on the pca9685 servoboards
        
        Contains all parameters necessary to define a hexapod
        Legs
        Gait
        Coxa Angles
        Coxa Offsets : (origin of coxa reference frame/anchor point of leg.)
        Segment length : (Length of each segment of the leg)
        Rest Angles : Angles in degrees for a robot in a neutral rest stance
        --
        Also this class initializes the legs
    '''
    # Initialize basic variables
    # define the number of legs
    num_legs: int
    #Gait pattern
    Gait: Optional['GaitType']
    leg_names: list[str] = field(default_factory=list)
    # Base angle around the hexapod center (0 being directly in front)
    coxa_offsets: List[float] = field(default_factory=list)
    # Origin coordinates of each leg's coxa (anchor point) in the robot's base reference frame
    coxa_coords: List['Coordinate'] = field(default_factory=list)
    # Segment lengths for each leg (list of SegmentLengths for Coxa, Femur, Tibia)
    leg_segments: List['SegmentLengths'] = field(default_factory=list)
    # Rest (neutral) angles for each leg (list of ServoAngles)
    rest_angles: List['ServoAngles'] = field(default_factory=list)


    @classmethod
    def load(cls, filename: str) -> 'Body':
        """Load a body definition from a JSON file."""
        with open(filename, 'r') as f:
            data = json.load(f)

        # Convert dictionary values into the expected list format
        coxa_offsets = list(data["coxa_angle_offset"].values())

        # Convert dictionary coordinates into a list of Coordinate objects
        coxa_coords = [Coordinate(*coords) for coords in data["Coordinate"].values()]

        # Set each segment length for all 6 legs
        leg_segments = [
            SegmentLengths(
                Coxa=data["Segments"]["Coxa"],
                Femur=data["Segments"]["Femur"],
                Tibia=data["Segments"]["Tibia"]
            ) for _ in range(data["NumLegs"])
        ]
        leg_names = data["legNames"]

        # set standing/rest angles for each leg
        rest_angles = [
            ServoAngles(
                Coxa=data["Angles"]["Coxa"],
                Femur=data["Angles"]["Femur"],
                Tibia=data["Angles"]["Tibia"]
            ) for _ in range(data["NumLegs"])
        ]

        return cls(
            num_legs=data["NumLegs"],
            Gait=data["Gait"],  # This may need to be converted from string to an actual Gait object
            leg_names=leg_names,
            coxa_offsets=coxa_offsets,
            coxa_coords=coxa_coords,
            leg_segments=leg_segments,
            rest_angles=rest_angles,
        )

    def set_gait(self, new_gait):
        self.Gait = new_gait
    
def new_hexapod_body() -> Body:
    tripod_gait = new_Gait(0, 1.0) # Set gait and speed through bezier curve
    body = Body(6, Gait=tripod_gait) # create new hexapod body 
    body = body.load("hexapod_config.json")
    return body


if __name__ == '__main__':
    hexapod_body = new_hexapod_body()

    print(type(hexapod_body))
    print(hexapod_body.num_legs)
    print(hexapod_body.Gait)
    print(hexapod_body.coxa_offsets)
    print(hexapod_body.coxa_coords)
    print(hexapod_body.leg_segments)
    print(hexapod_body.rest_angles)