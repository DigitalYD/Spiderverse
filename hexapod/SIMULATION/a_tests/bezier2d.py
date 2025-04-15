import numpy as np
from scipy.special import comb
from typing import Optional

class BezierCurve:
    def __init__(self, control_points: dict, num_pts=50):
        self.control_points_dict = control_points
        self.num_points = num_pts
        self.control_points_list = np.array(list(control_points.values()))
        self.index_map = {}
        self.index = 0
        self.generate()
        self.cp:np.ndarray

    def compute_bezier_curve(self):
        """Generates the Bezier curve using De Casteljau's algorithm."""
        n = len(self.control_points_list) - 1
        t_values = np.linspace(0, 1, self.num_points)
        curve = np.zeros((self.num_points, 3))
        for i in range(n + 1):
            curve += np.outer(self.bernstein_poly(i, n, t_values), self.control_points_list[i])
        return curve

    def bernstein_poly(self, i, n, t):
        """Computes the Bernstein polynomial for Bezier curve calculation."""
        t = np.asarray(t)  # Ensure t is an array
        return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

    def evaluate(self, t: float) -> np.ndarray:
        """Evaluate the Bezier curve at a continuous t value (0 to 1)."""
        n = len(self.control_points_list) - 1
        point = np.zeros(3)
        t = np.clip(t, 0, 1)  # Ensure t stays in [0, 1]

        bernstein_vals = np.array([self.bernstein_poly(i, n, t) for i in range(n + 1)])
        point = np.sum(bernstein_vals[:, np.newaxis] * self.control_points_list, axis=0)
        
        return point

    def generate(self):
        """Generate the full Bézier curve and create an index map."""
        self.cp = self.compute_bezier_curve()
        control_names = list(self.control_points_dict.keys())
        for i, name in enumerate(control_names):
            self.index_map[name] = int((i / (len(control_names) - 1)) * (self.num_points - 1))

    def get_point(self, index: int) -> np.ndarray:
        """Retrieve a specific point on the Bezier curve."""
        return self.cp[min(index, len(self.cp) - 1)]

    def get_named_point(self, name: str) -> Optional[np.ndarray]:
        """Retrieve a point by name."""
        if name in self.index_map:
            return self.get_point(self.index_map[name])
        return None
    
    def reset_index(self):
        self.index = 0

    def curve(self) -> np.ndarray:
        """Return all points."""
        if len(self.cp) == 0:
            self.generate()
        return self.cp