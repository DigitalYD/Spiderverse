
from dataclasses import dataclass

class coord2D:
    # Top looking down, X,Y
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y
        
    def reset(self):
        self.x = 0.0
        self.y = 0.0
    
class coord3D:
    def __init__(self, x=0.0, y=0.0, z=0.0,  roty=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.roty = roty
        
    def from_array(self, array):
        self.x, self.y, self.z = array
        
    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

@dataclass
class Coordinate:
    X: float = 0.0
    Y: float = 0.0
    Z: float = 0.0

    def __str__(self) -> str:
        return f"[{self.X:.2f}, {self.Y:.2f}, {self.Z:.2f}]"
    
def new_Coordinate(x: float, y: float, z: float) -> Coordinate:
    """Factory function to create a new Coordinate (mirrors Go's NewCoordinate)."""
    return Coordinate(X=x, Y=y, Z=z)


@dataclass
class ServoAngles:
    """ ServoAngles holds the Coxa, Femur, Tibia joint angles (in degrees) for a leg """
    Coxa: float = 0.0
    Femur: float = 0.0
    Tibia: float = 0.0

def new_servo_angles(coxa: float, femur: float, tibia: float) -> ServoAngles:
    """ Factory function to create ServoAngles """
    return ServoAngles(Coxa=coxa, Femur=femur, Tibia=tibia)


@dataclass
class SegmentLengths:
    """ Defines the lengths of each segment in the robot leg (Coxa, Femur, Tibia) """
    Coxa: float = 0.0   # length of the coxa 
    Femur: float = 0.0  # length of the femur 
    Tibia: float = 0.0  # length of the tibia 