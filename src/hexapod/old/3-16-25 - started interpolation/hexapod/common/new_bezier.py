import numpy as np

def de_casteljau(points, t):
    """ Compute a single point on the Bezier curve using De Casteljau's algorithm. """
    points = np.array(points, dtype=np.float64)
    while len(points) > 1:
        points = (1 - t) * points[:-1] + t * points[1:]  # Linear interpolation
    return points[0]

def bezier_curve(points, num_points=100):
    """ Generate the full Bezier curve using De Casteljau's algorithm. """
    return np.array([de_casteljau(points, t) for t in np.linspace(0, 1, num_points)])


if __name__ in "__main__":
    pass
    control_points = [
        np.array([0.2, 0, 0]),    # Start (grounded, back position)
        np.array([0.2, 0.1, 0.5]), # Lift up
        np.array([0.4, 0.2, 0.7]), # Peak height
        np.array([0.6, 0.1, 0.5]), # Descend
        np.array([0.8, 0, 0]),    # End (grounded, forward position)
        np.array([0.2, 0, 0])     # Transition back to P0 (ensuring a continuous loop)
    ]

    # Generate Bezier curve
    curve_points = bezier_curve(control_points, num_points=100)

