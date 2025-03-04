import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class HexapodAnimator:
    def __init__(self, leg_lengths=None, mount_angle=None, rotation_radius=100):
        self.leg_lengths = leg_lengths
        self.leg_names = ["LR", "LM", "LF", "RF", "RM", "RR"]

        # Initialize Matplotlib figure
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.view_init(elev=20, azim=30)

        # Initialize plot elements for all legs
        self.leg_lines = {leg: self.ax.plot([], [], [], 'o-', markersize=6)[0] for leg in self.leg_names}

        # Define fixed mount position
        self.mount_radius = rotation_radius  # Fixed radius of the circle
        self.mount_angle = mount_angle  # Fixed mount angle per leg
        self.rotation_angle = 0  # Rotation of the coxa in place

        # Compute initial mount positions
        self.mountx = {}
        self.mounty = {}
        self.mountz = {}

        for leg in self.leg_names:
            self.mountx[leg] = self.mount_radius * np.cos(self.mount_angle[leg])
            self.mounty[leg] = self.mount_radius * np.sin(self.mount_angle[leg])
            self.mountz[leg] = 193  # Fixed height

        # Configure 3D space limits
        self.ax.set_xlim([-500, 500])
        self.ax.set_ylim([-500, 500])
        self.ax.set_zlim([0, 500])
        self.ax.set_xlabel("X-axis")
        self.ax.set_ylabel("Y-axis")
        self.ax.set_zlabel("Z-axis")
        self.ax.set_title("Hexapod Leg - Fixed Mount with Proper Rotation")

        # ✅ Draw the Mount Circle
        self.draw_mount_circle()

    def draw_mount_circle(self):
        """ Draws the circle where the hexapod legs are mounted. """
        theta = np.linspace(0, 2 * np.pi, 100)  # 100 points around the circle
        x_circle = self.mount_radius * np.cos(theta)
        y_circle = self.mount_radius * np.sin(theta)
        z_circle = np.full_like(theta, 193)  # Keep the circle at mount height

        # ✅ Plot the circle
        self.ax.plot(x_circle, y_circle, z_circle, 'r-', linewidth=2, label="Mount Circle")
        self.ax.legend()

    def compute_all_legs(self, angles):
        """ Computes joint positions for all legs, rotating at their fixed mount. """
        leg_positions = {}

        for leg_name in angles.keys():
            theta1, theta2, theta3 = angles[leg_name]
            theta1 = np.deg2rad(int(np.rad2deg(theta1)))
            theta2 = np.deg2rad(int(np.rad2deg(theta2)))
            theta3 = np.deg2rad(int(np.rad2deg(theta3)))

            # Rotate leg around hexapod
            theta1 = theta1 + self.mount_angle[leg_name]
            coxa, femur, tibia = self.leg_lengths["coxa"], self.leg_lengths["femur"], self.leg_lengths["tibia"]

            # Compute Coxa Joint Position
            Xa = self.mountx[leg_name] + coxa * np.cos(theta1)
            Ya = self.mounty[leg_name] + coxa * np.sin(theta1)

            # Compute Femur Joint Position
            G2 = np.sin(theta2) * femur
            P1 = np.cos(theta2) * femur
            Xc = np.cos(theta1) * P1
            Yc = np.sin(theta1) * P1

            # Compute Tibia Joint Position
            H = np.sqrt(tibia**2 + femur**2 - 2*tibia*femur*np.cos(np.radians(180) - theta3))
            phi1 = np.arccos((femur**2 + H**2 - tibia**2) / (2*femur*H))
            phi3 = phi1 - theta2
            Pp = np.cos(phi3) * H
            Yb = np.sin(theta1) * Pp
            Xb = np.cos(theta1) * Pp
            G1 = - np.sin(phi3) * H

            # Store the joint locations
            leg_positions[leg_name] = np.array([
                [self.mountx[leg_name], self.mounty[leg_name], self.mountz[leg_name]],  # Mount
                [Xa, Ya, self.mountz[leg_name]],  # Coxa Joint
                [Xa + Xc, Ya + Yc, G2 + self.mountz[leg_name]],  # Femur Joint
                [Xa + Xb, Ya + Yb, G1 + self.mountz[leg_name]]  # Tibia (Toe)
            ])
        return leg_positions

    def update(self, thetas):
        """ Updates the leg positions while keeping the mount circle visible. """
        leg_positions = self.compute_all_legs(thetas)

        for leg_name in leg_positions.keys():
            joint_positions = leg_positions[leg_name]
            x_data = joint_positions[:, 0]
            y_data = joint_positions[:, 1]
            z_data = joint_positions[:, 2]

            self.leg_lines[leg_name].set_data(x_data, y_data)
            self.leg_lines[leg_name].set_3d_properties(z_data)

        plt.draw()
        plt.pause(0.001)  # Small pause to refresh the figure
