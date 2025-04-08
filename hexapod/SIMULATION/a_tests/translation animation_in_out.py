import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.special import comb
from matplotlib.animation import FuncAnimation

class BezierCurve:
    def __init__(self, control_points, num_pts=100):
        self.control_points = list(control_points.values())
        self.num_pts = num_pts

    def curve(self):
        t = np.linspace(0, 1, self.num_pts)
        n = len(self.control_points) - 1
        curve_points = np.zeros((self.num_pts, 3))
        for i in range(self.num_pts):
            point = np.zeros(3)
            for j, cp in enumerate(self.control_points):
                b = comb(n, j) * (t[i]**j) * ((1 - t[i])**(n - j))
                point += b * cp
            curve_points[i] = point
        return curve_points

def translate_point_along_leg_direction(point: np.ndarray, theta_offset: float, theta: float, distance: float) -> np.ndarray:
    total_theta = np.radians(theta_offset + theta)
    direction = np.array([np.cos(total_theta), np.sin(total_theta), 0])
    translation = direction * distance
    return point + translation

def get_radial_theta(coxa_pos: np.ndarray) -> float:
    """Compute the radial angle from [0, 0, 0] to Coxa in degrees."""
    return np.degrees(np.arctan2(coxa_pos[1], coxa_pos[0]))

leg_data = {
    "RR": {"coxa_coord": np.array([60.5, -89, 0]), "theta_offset": 147.172},
    "LR": {"coxa_coord": np.array([-60.5, -89, 0]), "theta_offset": 214.309},
    "RF": {"coxa_coord": np.array([60.5, 89, 0]), "theta_offset": 34.0685},
    "LF": {"coxa_coord": np.array([-60.5, 89, 0]), "theta_offset": 325.931},
    "LM": {"coxa_coord": np.array([-97, 0, 0]), "theta_offset": 270},
    "RM": {"coxa_coord": np.array([97, 0, 0]), "theta_offset": 90}
}

neutral_effector_offset = np.array([0, 0, -160])

def get_bezier_control_points(start_pos: np.ndarray) -> dict:
    return {
        "start": start_pos,
        "lift": start_pos + np.array([0, 10, 25]),
        "peak": start_pos + np.array([0, 20, 75]),
        "lower": start_pos + np.array([0, 25, 25]),
        "touchdown": start_pos + np.array([0, 25, 25]),
        "grounded": start_pos + np.array([0, 25, 0]),
        "sliding": start_pos + np.array([0, 35, 0]),
        "return": start_pos,
    }

theta = 0.0
distance = 20.0

original_curves = {}
translated_curves = {}
for leg_name, data in leg_data.items():
    coxa_pos = data["coxa_coord"]
    start_pos = coxa_pos + neutral_effector_offset
    control_points = get_bezier_control_points(start_pos)
    bezier = BezierCurve(control_points, num_pts=100)
    original_curves[leg_name] = bezier.curve()
    radial_theta = get_radial_theta(coxa_pos)
    translated_curve = np.array([translate_point_along_leg_direction(point, radial_theta, theta, distance)
                                 for point in original_curves[leg_name]])
    translated_curves[leg_name] = translated_curve

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
center = np.array([0, 0, 0])
ax.scatter(*center, color='black', s=100, label="Hexapod Center")
colors = {'RR': 'b', 'LR': 'm', 'RF': 'g', 'LF': 'r', 'LM': 'c', 'RM': 'y'}
plots = {leg: {'orig': ax.plot([], [], [], '-', color=colors[leg], alpha=0.5)[0],
               'trans': ax.plot([], [], [], '--', color=colors[leg])[0]} 
         for leg in leg_data}
for leg, data in leg_data.items():
    ax.scatter(data["coxa_coord"][0], data["coxa_coord"][1], data["coxa_coord"][2], color=colors[leg], label=leg)
ax.set_xlim(-150, 150)
ax.set_ylim(-150, 150)
ax.set_zlim(-170, 80)
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")
ax.set_title("3D Translation of Bézier Curves (Radial In/Out)")
ax.legend()

def update(frame):
    t = frame / 49
    dist = distance * np.sin(2 * np.pi * t)
    for leg_name, data in leg_data.items():
        coxa_pos = data["coxa_coord"]
        orig_curve = original_curves[leg_name]
        radial_theta = get_radial_theta(coxa_pos)
        trans_curve = np.array([translate_point_along_leg_direction(point, radial_theta, theta, dist)
                                for point in orig_curve])
        plots[leg_name]['orig'].set_data(orig_curve[:, 0], orig_curve[:, 1])
        plots[leg_name]['orig'].set_3d_properties(orig_curve[:, 2])
        plots[leg_name]['trans'].set_data(trans_curve[:, 0], trans_curve[:, 1])
        plots[leg_name]['trans'].set_3d_properties(trans_curve[:, 2])
    return [p for leg in plots.values() for p in leg.values()]

ani = FuncAnimation(fig, update, frames=50, interval=50, blit=True)
plt.show()