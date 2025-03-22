
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
    ''' function to create a new Coordinate (mirrors Go's NewCoordinate)'''
    return Coordinate(X=x, Y=y, Z=z)


@dataclass
class ServoAngles:
    ''' ServoAngles holds the Coxa, Femur, Tibia joint angles (in degrees) for a leg '''
    Coxa: float = 0.0
    Femur: float = 0.0
    Tibia: float = 0.0

def new_servo_angles(coxa: float, femur: float, tibia: float) -> ServoAngles:
    ''' Function to create ServoAngles '''
    return ServoAngles(Coxa=coxa, Femur=femur, Tibia=tibia)


@dataclass
class SegmentLengths:
    ''' Defines the lengths of each segment in the robot leg (Coxa, Femur, Tibia) '''
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

def homogeneous_transform_around_center(center: np.ndarray, axis: np.ndarray, theta:float)->np.ndarray:
    '''
        A 4x4 homogeneous transformation matrix to rotate a point around a central point in space
        Args: 
            center: 3D center oof rotation [cx, cy, cz]
            axis: 3D rotation axis [ax, ay, az]
            Theta: Rotatoin angle in radians
        NOTE: To rotate around x, y, z axis'. axis = [1,0,0], [0,1,0], [0,0,1]. Mix and match for diagonal rotations
        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix
    '''
    # Ensure points are np arrays
    center = np.array(center, dtype=float)
    axis = np.array(axis, dtype=float)

    # normalize axis
    axis = axis/np.linalg.norm(axis)
    ux, uy, uz = axis

    # setup vars
    c = np.cos(theta)
    s = np.sin(theta)
    t = 1-c

    # Rotation Matrix (3x3) around arbitrary axis (Rodrigues' Formula)
    R = np.array([
        [t*ux*ux + c,      t*ux*uy - uz*s, t*ux*uz + uy*s],
        [t*ux*uy + uz*s,   t*uy*uy + c,    t*uy*uz - ux*s],
        [t*ux*uz - uy*s,   t*uy*uz + ux*s, t*uz*uz + c]
    ])
    
    ## Allows rotation around any arbitrary point
    t_to_origin = np.eye(4)
    t_to_origin[:3,3] = -center # moves point to origin
    t_rotation = np.eye(4)
    t_rotation[:3,:3] = R # Applies rotation
    t_back = np.eye(4)
    t_back[:3,3] = center # moves point back
    transformation = t_back @ t_rotation @ t_to_origin

    return transformation


def get_transformation_from_matrix(transformation:np.ndarray) -> np.ndarray:
    '''
        Extract the translated component from a 4x4 matrix | Same as get_joint_origin() but global
        Args:
            transformation: 4x4 homogenous transformation matrix
        
        Returns:
            3d point (tx, ty, tz) belonging to the transformed point
    '''
    return transformation[:3,3]

def apply_transform_to_point(transformation:np.ndarray, point:np.ndarray)-> np.ndarray:
    '''
        Apply the transformation matrix to a specific 3d point
        Args:
            Transformation: 4x4 homogeneous transform matrix
            point: 3D point

        Returns:
            np.ndarray: Transformed 3d point [x', y', z']
    '''
    # convert to homogenous coordinates
    point_h = np.append(point,1)

    # apply transform
    pointt_t_h = transformation @ point_h

    # Extract 3d Coord
    return pointt_t_h[:3]

def homogeneous_transformation_matrix(projection_matrix: np.ndarray, theta: float, length: float) -> np.ndarray:
    ''' Create a homogeneous transformation matrix given a projection matrix, a rotation angle theta (rad), and a displacement length.
    Args:
        Projection_Matrix: matrix to be rotated
        theta: rotation angle
        length: displacement to be moved along x/y axis.

    Return:
        np.ndarray: returns the 4x4 homogeneous transformation matrix that's been rotated and displaced
    '''
    # Rotation matrix about Z-axis by theta
    R_z = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta),  0],
        [0,               0,                1]
    ])

    # Final rotation = R_z * projection_matrix (3x3)
    R = R_z.dot(projection_matrix)

    # Displacement vector for this segment in the rotated frame
    D = np.array([[length * np.cos(theta)], [length * np.sin(theta)], [0]])
    
    # Construct 4x4 homogeneous transformation matrix from R and D
    H = np.array([
        [R[0,0], R[0,1], R[0,2], D[0,0]],
        [R[1,0], R[1,1], R[1,2], D[1,0]],
        [R[2,0], R[2,1], R[2,2], D[2,0]],
        [0,      0,      0,      1     ]
    ])
    return H


def get_radial_direction(coxa_pos: np.ndarray) -> np.ndarray:
    ''' 
        Computes the radial direction unit vector from center [0, 0, 0] to Coxa in XY-plane
    
    '''    
    direction = np.array([coxa_pos[0], coxa_pos[1], 0])
    return direction / np.linalg.norm(direction)


def adjust_point_away_from_coxa(point: np.ndarray, direction: np.ndarray, distance: float) -> np.ndarray:
    ''' Adjust a point away from the Coxa along the radial direction '''
    translation = direction * distance
    return point + translation


def rotate_bezier_curve(curve: np.ndarray, center: np.ndarray, theta: float) -> np.ndarray:
    ''' Rotate a Bézier curve around a center point on the Z-axis '''
    axis = np.array([0, 0, 1])  # Z-axis
    T = homogeneous_transform_around_center(center, axis, theta)
    return np.array([apply_transform_to_point(T, point) for point in curve])

def translate_point_along_leg_direction(point: np.ndarray, theta_offset: float, theta: float, distance: float) -> np.ndarray:
    ''' Translate a point along a leg's direction, adjusted by a theta '''
    total_theta = np.radians(theta_offset + theta)
    direction = np.array([np.cos(total_theta), np.sin(total_theta), 0])  # XY-plane only
    translation = direction * distance
    return point + translation


def rot_tran_3d_x(theta):
    ''' Roll angle, converts it to radians and returns 
            
    Returns
        4x4 transformation matrix for theta.
    '''
    theta = np.deg2rad(theta)
    return np.array([[1, 0, 0, 0],
                    [0, np.cos(theta), -np.sin(theta), 0],
                    [0, np.sin(theta), np.cos(theta), 0],
                    [0, 0, 0, 1]
                    ])

def rot_tran_3d_y(theta):
    ''' Pitch an angle, converts it to radians and returns 
            
    Returns
        4x4 transformation matrix for theta.  
    '''
    theta = np.deg2rad(theta)
    return np.array([[np.cos(theta), 0, np.sin(theta), 0],
                    [0, 1, 0, 0],
                    [-np.sin(theta), 0, np.cos(theta), 0],
                    [0, 0, 0, 1]
                    ])

def rot_tran_3d_z(theta):
    ''' Yaw angle, converts it to radians and 
        
    Returns
        4x4 transformation matrix for theta.   
    '''
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