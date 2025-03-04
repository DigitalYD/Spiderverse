import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

class HexapodAnimator:
    def __init__(self, leg_lengths=None, mount_angle= None, rotation_radius=100):
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
        self.mount_angle = mount_angle  # Fixed mount angle for 
        self.rotation_angle = 0  # Rotation of the coxa in place

        # Compute initial mount position
        self.mountx = {}
        self.mounty = {}
        self.mountz = {}

        for idx, leg in enumerate(self.leg_names):
            self.mountx[leg] = self.mount_radius * np.cos(self.mount_angle[leg])
            self.mounty[leg] = self.mount_radius * np.sin(self.mount_angle[leg])
            self.mountz[leg] = 193  # Fixed height

        # Configure 3D space limits
        self.ax.set_xlim([-500, 500])
        self.ax.set_ylim([-500, 500])
        self.ax.set_zlim([-500, 500])
        self.ax.set_xlabel("X-axis")
        self.ax.set_ylabel("Y-axis")
        self.ax.set_zlabel("Z-axis")
        self.ax.set_title("Hexapod Leg - Fixed Mount with Proper Rotation")

    def compute_all_legs(self, angles):
        """ Computes joint positions for the LR leg, rotating at its fixed mount. """
        leg_positions = {}

        if "LR" in angles:
            theta1, theta2, theta3 = angles["LR"]

            # rotate lega aroundhexapod
            theta1 = theta1 + self.mount_angle["LR"]
            coxa, femur, tibia = self.leg_lengths["coxa"], self.leg_lengths["femur"], self.leg_lengths["tibia"]

            # Apply additional rotation to the leg at its mount point
            Xa = self.mountx["LR"] + coxa * np.cos(theta1)
            Ya = self.mounty["LR"] + coxa * np.sin(theta1)
            
            # Calculate vertical component of femur length
            G2 = np.sin(theta2) * femur
            
            # Calculate horizontal component of femur length
            P1 = np.cos(theta2) * femur
            
            # Calculate x and y coordinates of femur-tibia joint
            Xc = np.cos(theta1) * P1
            Yc = np.sin(theta1) * P1
            
            # Calculate H, phi1, phi2, phi3, Pp, P2, Yb, Xb, G1
            # to get the coordinates of the tibia-tip joint
            H = np.sqrt(tibia**2 + femur**2 - 2*tibia*femur*np.cos(np.radians(180) - theta3))
            phi1 = np.acos((femur**2 + H**2 - tibia**2) / (2*femur*H))
            phi2 = np.pi - theta3 - phi1
            phi3 = phi1 - theta2
            Pp = np.cos(phi3) * H
            P2 = Pp - P1
            Yb = np.sin(theta1) * Pp
            Xb = np.cos(theta1) * Pp
            G1 = - np.sin(phi2) * H

            # Create a list of joint locations
            leg_positions["LR"] = np.array([
                [self.mountx["LR"], self.mounty["LR"], self.mountz["LR"]], # start joint
                [Xa, Ya, self.mountz["LR"]],  # coxa-femur joint
                [Xa + Xc, Ya + Yc, G2+ self.mountz["LR"]],  # femur-tibia joint
                [Xa + Xb, Ya + Yb, G1+ self.mountz["LR"]]  # tip of the leg
            ])
            return leg_positions


    def update(self, thetas):
        """ Updates the LR leg position while rotating at its fixed mount position. """
        #self.rotation_angle =   # Increment rotation relative to mount according to what leg we are on
        leg_positions = self.compute_all_legs(thetas)

        if "LR" in leg_positions:
            joint_positions = leg_positions["LR"]
            x_data = joint_positions[:, 0]
            y_data = joint_positions[:, 1]
            z_data = joint_positions[:, 2]

            self.leg_lines["LR"].set_data(x_data, y_data)
            self.leg_lines["LR"].set_3d_properties(z_data)

        plt.draw()
        plt.pause(0.01)  # Small pause to refresh the figure