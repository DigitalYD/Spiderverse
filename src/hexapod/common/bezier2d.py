import numpy as np

class BezierCurve:
    def __init__(self, n=100):
        '''
        Create a bezier curve. (Adding points, and generating all values is done later)
        '''
        self.points_list = []  # Control points
        self.num_points = n
        self.cp = np.empty((0, 2), dtype=np.float64)  # Computed curve points

    def add_point(self, x, y):
        """ Add a control point (x, y) to the curve. """
        self.points_list.append([x, y])

    def de_casteljau(self, t):
        """ Compute a single point on the Bezier curve using De Casteljau's algorithm. """
        points = np.array(self.points_list, dtype=np.float64)
        while len(points) > 1:
            points = (1 - t) * points[:-1] + t * points[1:]  # Linear interpolation
        return points[0]

    def generate(self):
        """ Generate the full Bezier curve. """
        self.cp = np.array([self.de_casteljau(t) for t in np.linspace(0, 1, self.num_points)])

    def curve(self):
        """ Return the generated Bezier curve points. """
        if len(self.cp) == 0:
            self.generate()
        return self.cp

    def control_points(self):
        """ Return the control points. """
        return np.array(self.points_list)
    

if __name__ == "__main__":
    pass