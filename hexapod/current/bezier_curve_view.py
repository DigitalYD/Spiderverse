import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pod import Pod
from hex_body import new_hexapod_body

# Initialize pod with hexapod body config
hexapod = Pod(body_def=new_hexapod_body())

# Setup a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Loop through each leg and plot its Bézier curve
for i, leg in enumerate(hexapod.Legs):
    curve = leg.bezier_curve.curve()  # shape: (num_points, 3)
    xs, ys, zs = curve[:, 0], curve[:, 1], curve[:, 2]
    ax.plot(xs, ys, zs, label=f'Leg {i}')

# Label and show
ax.set_title('Bezier Curves of Hexapod Legs')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()
