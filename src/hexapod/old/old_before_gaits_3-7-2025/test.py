import numpy as np
import matplotlib.pyplot as plt

coxa_positions = np.array([
    [-60.5, -89, 0],
    [-97, 0, 0],
    [-60.5, 89, 0],
    [60.5, 89, 0],
    [97, 0, 0],
    [60.5, -89, 0]
])

# Given angles in degrees
angles_deg = np.array([214.309, 270, 325.931, 34.0685, 90, 146.172])

# Convert angles to radians
angles_rad = np.radians(angles_deg)

# Radius of the circle (use an average of the coxa positions)
radius = np.mean(np.linalg.norm(coxa_positions[:, :2], axis=1))

# Compute circle points from the given angles
circle_points = np.array([
    [radius * np.cos(angle), radius * np.sin(angle), 0] for angle in angles_rad
])

# Create a 2D plot to compare the given points and hexagonal structure
fig, ax = plt.subplots(figsize=(8, 8))

# Plot coxa positions (hexagonal structure)
ax.scatter(coxa_positions[:, 0], coxa_positions[:, 1], color='g', s=100, label='Hexagon Points')

# Plot calculated circle points from given angles
ax.scatter(circle_points[:, 0], circle_points[:, 1], color='b', s=100, label='Circle Points')

# Connect hexagon points with a dashed line
for i in range(len(coxa_positions)):
    ax.plot([coxa_positions[i-1, 0], coxa_positions[i, 0]], 
            [coxa_positions[i-1, 1], coxa_positions[i, 1]], 'g--')

# Connect circle points with a dashed line
for i in range(len(circle_points)):
    ax.plot([circle_points[i-1, 0], circle_points[i, 0]], 
            [circle_points[i-1, 1], circle_points[i, 1]], 'b--')

# Set labels and title
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_title("Comparison of Hexagon Points and Circle Points from Given Angles")

# Equal axis scaling for proper visualization
ax.set_aspect('equal', adjustable='datalim')

# Show legend
ax.legend()

# Display the plot
plt.show()


import numpy as np
import matplotlib.pyplot as plt

coxa_positions = np.array([
    [-60.5, -89, 0],
    [-97, 0, 0],
    [-60.5, 89, 0],
    [60.5, 89, 0],
    [97, 0, 0],
    [60.5, -89, 0]
])

# Given angles in degrees
angles_deg = np.array([214.309, 270, 325.931, 34.0685, 90, 146.172])
import numpy as np
import matplotlib.pyplot as plt

# Define the circle radius (same as coxa distance from center)
radius = np.linalg.norm(coxa_positions[0][:2])  # Approximate radius from first coxa position

# Define the given angles in degrees
angles_deg = [214.309, 270, 325.931, 34.0685, 90, 146.172]
angles_rad = np.radians(angles_deg)  # Convert to radians

# Compute circle points from angles
circle_points = np.array([[radius * np.sin(angle), radius * np.cos(angle), 0] for angle in angles_rad])

# Create a 2D figure to compare
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')

# Plot hexagon (Coxa Positions)
ax.scatter(coxa_positions[:, 0], coxa_positions[:, 1], color='g', s=100, label="Hexagon Points")

# Plot circle points
ax.scatter(circle_points[:, 0], circle_points[:, 1], color='b', s=100, label="Circle Points")

# Connect hexagon points
for i in range(len(coxa_positions)):
    ax.plot([coxa_positions[i-1, 0], coxa_positions[i, 0]], 
            [coxa_positions[i-1, 1], coxa_positions[i, 1]], 'g--')

# Connect circle points
for i in range(len(circle_points)):
    ax.plot([circle_points[i-1, 0], circle_points[i, 0]], 
            [circle_points[i-1, 1], circle_points[i, 1]], 'b--')

# Labels and title
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_title("Hexagon vs Circle Points")

ax.legend()
plt.grid()
plt.show()
