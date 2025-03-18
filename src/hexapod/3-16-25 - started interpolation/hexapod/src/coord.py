
from dataclasses import dataclass
import numpy as np


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

### -----------------------
# Rotates around a single point
# 3d Coordinate Axis rotation matrixes. Use this to multiple against a translation matrix!
# This coordinate system has Z as vertical
# NOTE: The "pitch, roll, yaw may be incorrect depending on which application I was using it for. Rever to diagram below for use case and flip axis as needed
'''

       (Yaw - Z-axis Rotation)
            ↺ Counterclockwise / ↻ Clockwise
                 +Z (Up)
                   │    
                   │    
      (Roll - X-axis Rotation)   
 ↺ Left   ────► X-axis  ◄───  ↻ Right  
                
                   │    
                   │  
                 -Z (Down)
                   
      (Pitch - Y-axis Rotation)   
   ↘ Nose Down       Y-axis       Nose Up ↖    



'''
### -----------------------

def rot_tran_3d_x(theta):
    ''' Roll angle, converts it to radians and returns '''
    theta = np.deg2rad(theta)
    return np.array([[1, 0, 0, 0],
                    [0, np.cos(theta), -np.sin(theta), 0],
                    [0, np.sin(theta), np.cos(theta), 0],
                    [0, 0, 0, 1]
                    ])

def rot_tran_3d_y(theta):
    ''' Pitch an angle, converts it to radians and returns '''
    theta = np.deg2rad(theta)
    return np.array([[np.cos(theta), 0, np.sin(theta), 0],
                    [0, 1, 0, 0],
                    [-np.sin(theta), 0, np.cos(theta), 0],
                    [0, 0, 0, 1]
                    ])

def rot_tran_3d_z(theta):
    ''' Yaw angle, converts it to radians and returns '''
    theta = np.deg2rad(theta)
    return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                    [np.sin(theta), np.cos(theta), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                    ])


### -----------------------
# Rotates around a single point
# calculate position corrections using rotation matrix
# https://en.wikipedia.org/wiki/Rotation_matrix
# NOT useful for homogeneous solutions. These get multiplied with theta then multiplied
### -----------------------

def rotation_2d(theta):
    ''' Accepts an angle, converts it to radians and returns rotation 2d'''
    theta = np.deg2rad(theta)
    return np.array([[np.cos(theta), -np.sin[theta]], [np.sin(theta), np.cos(theta)]])

def rot_mat_3d_x(theta):
    ''' Pitch angle, converts it to radians and returns '''
    theta = np.deg2rad(theta)
    return np.array([[1, 0, 0],
                        [0, np.cos(theta), -np.sin(theta)],
                        [0, np.sin(theta), np.cos(theta)]])

def rot_mat_3d_y(theta):
    ''' Roll an angle, converts it to radians and returns '''
    theta = np.deg2rad(theta)
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])

def rot_mat_3d_z(theta):
    ''' Yaw angle, converts it to radians and returns '''
    theta = np.deg2rad(theta)
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                        [np.sin(theta), np.cos(theta), 0],
                        [0, 0, 1]])