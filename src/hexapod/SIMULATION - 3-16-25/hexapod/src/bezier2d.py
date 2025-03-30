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
        ''' Generates the Bezier curve using De Casteljau's algorithm '''
        n = len(self.control_points_list) - 1
        t_values = np.linspace(0, 1, self.num_points)
        curve = np.zeros((self.num_points, 3))
        for i in range(n + 1):
            curve += np.outer(self.bernstein_poly(i, n, t_values), self.control_points_list[i])
        return curve

    def bernstein_poly(self, i, n, t):
        ''' Computes the Bernstein polynomial for Bezier curve calculation '''
        t = np.asarray(t)  # Ensure t is an array
        return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

    def evaluate(self, t: float) -> np.ndarray:
        ''' Evaluate the Bezier curve at a continuous t value (0 to 1) '''
        n = len(self.control_points_list) - 1
        point = np.zeros(3)
        t = np.clip(t, 0, 1)  # Ensure t stays in [0, 1]

        bernstein_vals = np.array([self.bernstein_poly(i, n, t) for i in range(n + 1)])
        point = np.sum(bernstein_vals[:, np.newaxis] * self.control_points_list, axis=0)
        
        return point

    def generate(self):
        ''' Generate the full Bézier curve and create an index map '''
        self.cp = self.compute_bezier_curve()
        control_names = list(self.control_points_dict.keys())
        for i, name in enumerate(control_names):
            self.index_map[name] = int((i / (len(control_names) - 1)) * (self.num_points - 1))

    def get_point(self, index: int) -> np.ndarray:
        ''' Retrieve a specific point on the Bezier curve '''
        return self.cp[min(index, len(self.cp) - 1)]

    def get_named_point(self, name: str) -> Optional[np.ndarray]:
        """Retrieve a point by name."""
        if name in self.index_map:
            return self.get_point(self.index_map[name])
        return None
    
    def precomputed_points(self, control_points, num_points=100):
        ''' Precompute num_points along the Bezier curve defined by control_points '''
        return [self.bezier_curve(control_points, i / (num_points - 1)) for i in range(num_points)]

    def reset_index(self):
        self.index = 0

    def curve(self) -> np.ndarray:
        ''' Return all points '''
        if len(self.cp) == 0:
            self.generate()
        return self.cp
    
    def bezier_curve(self, points, t):
        n = len(points) - 1
        return sum(comb(n, i) * (1 - t)**(n - i) * t**i * np.array(p) for i, p in enumerate(points))
    
    def get_points_between_compute(self, start_key: str, end_key: str) -> list[np.ndarray]:
        ''' Get all points between two named control points in the dictionary '''
        if start_key not in self.index_map or end_key not in self.index_map:
            raise ValueError(f"Keys '{start_key}' or '{end_key}' not found in control points dictionary.")
        
        # Get the indices of the start and end keys in the control points list
        control_names = list(self.control_points_dict.keys())
        start_idx = control_names.index(start_key)
        end_idx = control_names.index(end_key)
        
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx  # Swap if start is after end
        
        # Extract the subset of control points between start_key and end_key (inclusive)
        subset_control_points = [self.control_points_dict[control_names[i]] 
                                 for i in range(start_idx, end_idx + 1)]
        
        num_points = len(subset_control_points)
        # Compute the Bezier curve for this subset
        return self.precomputed_points(subset_control_points, num_points)
    

    def get_points_between(self, start_key: str, end_key: str) -> list[np.ndarray]:
        ''' Get the precomputed points between two named control points in the dictionary '''
        if start_key not in self.index_map or end_key not in self.index_map:
            raise ValueError(f"Keys '{start_key}' or '{end_key}' not found in control points dictionary.")

        # Get the precomputed indices for the start and end keys
        start_idx = self.index_map[start_key]
        end_idx = self.index_map[end_key]
        
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx  # Swap if start is after end
        
        # Return the slice of precomputed points between start_idx and end_idx
        return [self.cp[i] for i in range(start_idx, end_idx + 1)]