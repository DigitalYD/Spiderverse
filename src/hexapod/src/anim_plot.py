import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class HexapodAnimator:
    def __init__(self, leg_lengths=None, mount_positions=None, mount_angle=None):
        self.leg_lengths = leg_lengths
        self.leg_names = ["LR", "LM", "LF", "RF", "RM", "RR"]
        self.leg_colors = {
            "LR": "red",
            "LM": "blue",
            "LF": "green",
            "RF": "orange",
            "RM": "purple",
            "RR": "royalblue"
        }
        
        # Initialize Matplotlib figure
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.view_init(elev=20, azim=30)

        # Initialize plot elements for all legs
        self.leg_lines = {
            leg: self.ax.plot([], [], [], 'o-', markersize=6, color=self.leg_colors[leg])[0]
            for leg in self.leg_names
        }
        
        # Define fixed mount positions from pentagon layout
        self.mount_positions = mount_positions
        self.mount_angle = mount_angle  # Fixed mount angle per leg

        # Convert to dictionaries for easier access
        self.mountx = {leg: mount_positions[leg][0] for leg in self.leg_names}
        self.mounty = {leg: mount_positions[leg][1] for leg in self.leg_names}
        self.mountz = {leg: 193 for leg in self.leg_names}  # Fixed height

        # Configure 3D space limits
        self.ax.set_xlim([150, -150]) 
        self.ax.set_ylim([-150, 150])
        self.ax.set_zlim([0, 250]) 
        self.ax.set_xlabel("X-axis")
        self.ax.set_ylabel("Y-axis")
        self.ax.set_zlabel("Z-axis")
        self.ax.set_title("Hexapod Leg - Mounted on a Pentagon")

        # Draw the Pentagon Mount
        self.draw_pentagon_mount()

    def draw_pentagon_mount(self):
        """ Draws the pentagon structure where the hexapod legs are mounted and labels each leg. """
        pentagon_points = np.array([
            self.mount_positions["LR"],
            self.mount_positions["LM"],
            self.mount_positions["LF"],
            self.mount_positions["RF"],
            self.mount_positions["RM"],
            self.mount_positions["RR"],
            self.mount_positions["LR"] 
        ])

        x_penta, y_penta = pentagon_points[:, 0], pentagon_points[:, 1]
        z_penta = np.full_like(x_penta, 193)  # Keep the mount height

        # Plot the pentagon
        self.ax.plot(x_penta, y_penta, z_penta, 'r-', linewidth=2, label="Pentagon Mount")

        # Add text labels with color information
        for leg, pos in self.mount_positions.items():
            x, y = pos
            z = 210  # Place text slightly above the mount height
            color = self.leg_colors.get(leg, "black")  # Default to black if no color is assigned
            self.ax.text(x, y, z, f"{leg}\n({color})", color=color, fontsize=10, ha='center', va='bottom', fontweight='bold')

        self.ax.legend()

    def compute_all_legs(self, angles):
        """ Foward Kinematics for simulation """
        leg_positions = {}
        
        for leg_name in angles.keys():
            theta1, theta2, theta3 = angles[leg_name]
            theta1 = np.deg2rad(int(np.rad2deg(theta1)))
            theta2 = np.deg2rad(int(np.rad2deg(theta2)))
            theta3 = np.deg2rad(int(np.rad2deg(theta3)))

            # Rotate leg around hexapod (adjusted for pentagon angles)
            theta1 = theta1 + np.deg2rad(self.mount_angle[leg_name])
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
        """ Update the leg positions """
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
