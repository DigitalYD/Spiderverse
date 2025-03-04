
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
    