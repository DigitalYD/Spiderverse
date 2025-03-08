import numpy as np
from scipy.special import comb

class BezierCurve:
    def __init__(self, control_points:dict, num_pts=100):
        '''
        Create a bezier curve with dictionary positions to keep track of where the leg is
        '''
        self.control_points_dict = control_points  # Store named control points
        self.num_points = num_pts  # Number of curve points
        self.control_points_list = np.array(list(control_points.values()))  # Convert to array
        self.index_map = {}  # Store named point indices
        self.points = np.empty((0, 3), dtype=np.float64)  # Computed points go here

        # Compute Bézier curve
        self.generate()

    def bernstein_poly(self, i, n, t):
        """Computes the Bernstein polynomial for Bezier curve calculation."""
        return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

    def compute_bezier_curve(self):
        """Generates the bezier curve using De Casteljau's algorithm."""
        n = len(self.control_points_list) - 1
        t_values = np.linspace(0, 1, self.num_points)
        curve = np.zeros((self.num_points, 3))  # 3D points

        for i in range(n + 1):
            curve += np.outer(self.bernstein_poly(i, n, t_values), self.control_points_list[i])

        return curve

    def generate(self):
        """Generate the full Bézier curve and create an index map."""
        self.cp = self.compute_bezier_curve()

        # Map named control points to closest curve indices
        control_names = list(self.control_points_dict.keys())
        for i, name in enumerate(control_names):
            self.index_map[name] = int((i / (len(control_names) - 1)) * (self.num_points - 1))

    def get_point(self, index):
        """Retrieve a specific point on the Bezier curve"""
        return self.cp[min(index, len(self.cp) - 1)]

    def get_named_point(self, name):
        """Retrieve a point"""
        if name in self.index_map:
            return self.get_point(self.index_map[name])
        return None

    def curve(self):
        """Return all points"""
        if len(self.cp) == 0:
            self.generate()
        return self.cp
