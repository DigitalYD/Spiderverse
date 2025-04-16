#!/usr/bin/env python3

import os
import sys
import argparse
import time
import json
import socket
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
from matplotlib.animation import FuncAnimation

# Default anchor positions (x, y, z) in centimeters - will be overridden based on launch files
BILATERATION_ANCHORS = [
    [0, 0, 90],      # Anchor 1
    [310, 0, 90]     # Anchor 2
]

TRILATERATION_ANCHORS = [
    [0, 0, 90],      # Anchor 1
    [310, 0, 90],    # Anchor 2
    [250, 600, 90]   # Anchor 3
]

# Default reference_y and prefer_positive_y for bilateration
REFERENCE_Y = 300.0
PREFER_POSITIVE_Y = True

# Server configuration for UDP listener
SERVER_IP = "0.0.0.0"  # Listen on all available interfaces
SERVER_PORT = 50000    # Port number

# Define paths
LIDAR_WS_PATH = os.path.expanduser("~/Documents/Spiderverse/lidar_ws")
BILATERATION_LAUNCH_PATH = os.path.join(LIDAR_WS_PATH, "src/lidar_udp_receiver/launch/bilateration_slam_launch.py")
TRILATERATION_LAUNCH_PATH = os.path.join(LIDAR_WS_PATH, "src/lidar_udp_receiver/launch/trilateration_slam_launch.py")

class UWBVisualizer:
    def __init__(self, mode='bi', simulation=False, reference_y=None, prefer_positive_y=None):
        """
        Initialize the UWB visualizer
        
        Args:
            mode: 'bi' for bilateration or 'tri' for trilateration
            simulation: If True, generate simulated data instead of using real UDP data
            reference_y: Override the reference_y value for bilateration
            prefer_positive_y: Override the prefer_positive_y value for bilateration
        """
        self.mode = mode
        self.simulation = simulation
        
        # Set up anchor positions based on mode
        self.anchors = self.load_anchor_positions()
        
        # For bilateration, set reference_y and prefer_positive_y
        self.reference_y = reference_y if reference_y is not None else REFERENCE_Y
        self.prefer_positive_y = prefer_positive_y if prefer_positive_y is not None else PREFER_POSITIVE_Y
        
        # Storage for tag position and distances
        self.tag_position = None  # (x, y, z) in cm
        self.distances = [None] * len(self.anchors)
        
        # Storage for position history (for trail)
        self.position_history = []
        self.max_history = 20  # Store last 20 positions
        
        # Set up socket if not in simulation mode
        if not simulation:
            self.setup_socket()
        
        # Set up the plot
        self.setup_plot()
    
    def load_anchor_positions(self):
        """Load anchor positions from launch files"""
        if self.mode == 'bi':
            # Get bilateration anchor positions
            try:
                anchors, extras = self.read_launch_file(BILATERATION_LAUNCH_PATH, 2)
                if anchors and len(anchors) == 2:
                    global REFERENCE_Y, PREFER_POSITIVE_Y
                    REFERENCE_Y = extras.get('reference_y', REFERENCE_Y)
                    PREFER_POSITIVE_Y = extras.get('prefer_positive_y', PREFER_POSITIVE_Y)
                    return anchors
                return BILATERATION_ANCHORS
            except Exception as e:
                print(f"Error loading bilateration anchor positions: {e}")
                return BILATERATION_ANCHORS
        else:
            # Get trilateration anchor positions
            try:
                anchors, _ = self.read_launch_file(TRILATERATION_LAUNCH_PATH, 3)
                if anchors and len(anchors) == 3:
                    return anchors
                return TRILATERATION_ANCHORS
            except Exception as e:
                print(f"Error loading trilateration anchor positions: {e}")
                return TRILATERATION_ANCHORS
    
    def read_launch_file(self, file_path, num_anchors):
        """Read anchor positions from a launch file"""
        positions = []
        extras = {}
        
        try:
            with open(file_path, 'r') as f:
                content = f.read()
                
            for i in range(1, num_anchors + 1):
                pattern = r"'anchor{}_pos':\s*\[\s*([\d.-]+)\s*,\s*([\d.-]+)\s*,\s*([\d.-]+)\s*\]".format(i)
                import re
                match = re.search(pattern, content)
                if match:
                    x, y, z = map(float, match.groups())
                    positions.append([x, y, z])
            
            # For bilateration, also read reference_y and prefer_positive_y
            if num_anchors == 2:
                pattern = r"'reference_y':\s*([\d.-]+)"
                match = re.search(pattern, content)
                if match:
                    extras['reference_y'] = float(match.group(1))
                    
                pattern = r"'prefer_positive_y':\s*(True|False)"
                match = re.search(pattern, content)
                if match:
                    extras['prefer_positive_y'] = match.group(1) == "True"
            
            return positions, extras
        except Exception as e:
            print(f"Error reading positions from {file_path}: {e}")
            return [], {}
    
    def setup_socket(self):
        """Set up UDP socket for receiving UWB data"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((SERVER_IP, SERVER_PORT))
            self.socket.settimeout(0.1)  # Short timeout for animation
            print(f"Listening for UWB data on {SERVER_IP}:{SERVER_PORT}")
        except Exception as e:
            print(f"Error setting up socket: {e}")
            print("Falling back to simulation mode")
            self.simulation = True
    
    def setup_plot(self):
        """Set up matplotlib visualization"""
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.canvas.manager.set_window_title(
            f"UWB {'Bilateration' if self.mode == 'bi' else 'Trilateration'} Visualization"
        )
        
        # Set up plot properties
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.grid(True)
        
        # Set up artists for dynamic elements
        self.artists = {
            'anchors': [],       # Anchor points
            'distances': [],     # Distance circles
            'tag': None,         # Tag position marker
            'trail': None,       # Position history trail
            'possible': None,    # Possible positions (for bilateration)
            'text_info': None    # Text information
        }
        
        # Calculate plot limits based on anchor positions
        anchor_xs = [a[0] for a in self.anchors]
        anchor_ys = [a[1] for a in self.anchors]
        min_x, max_x = min(anchor_xs) - 100, max(anchor_xs) + 100
        min_y, max_y = min(anchor_ys) - 100, max(anchor_ys) + 100
        
        # Add some margin for the distance circles
        max_dim = max(max_x - min_x, max_y - min_y)
        margin = max_dim * 0.2
        
        # Set axis limits
        self.ax.set_xlim(min_x - margin, max_x + margin)
        self.ax.set_ylim(min_y - margin, max_y + margin)
        
        # Plot anchors
        for i, anchor in enumerate(self.anchors):
            anchor_artist = self.ax.plot(anchor[0], anchor[1], 'ro', markersize=8)[0]
            self.artists['anchors'].append(anchor_artist)
            
            # Add anchor label
            self.ax.text(anchor[0] + 10, anchor[1] + 10, f"A{i+1}", fontsize=12)
            
            # Add distance circle (will be updated with real distances)
            distance_circle = Circle((anchor[0], anchor[1]), 0, fill=False, linestyle='--', alpha=0.5)
            self.artists['distances'].append(self.ax.add_patch(distance_circle))
        
        # Add tag position marker (starts at origin)
        self.artists['tag'] = self.ax.plot(0, 0, 'bo', markersize=10, label='Tag Position')[0]
        
        # Add trail (position history)
        self.artists['trail'] = self.ax.plot([], [], 'b-', alpha=0.5, linewidth=2)[0]
        
        # For bilateration, add markers for possible positions
        if self.mode == 'bi':
            self.artists['possible'] = self.ax.plot([], [], 'go', markersize=6, alpha=0.5, 
                                                   label='Possible Positions')[0]
        
        # Add reference_y line for bilateration
        if self.mode == 'bi':
            self.ax.axhline(y=self.reference_y, color='g', linestyle='--', alpha=0.5, 
                            label=f'Reference Y={self.reference_y}')
        
        # Add text information
        self.artists['text_info'] = self.ax.text(0.02, 0.98, "", transform=self.ax.transAxes, 
                                               va='top', fontsize=10, bbox=dict(boxstyle='round', 
                                               facecolor='white', alpha=0.7))
        
        # Add legend
        self.ax.legend(loc='upper right')
        
        # Add title
        self.ax.set_title(f"{'Bilateration' if self.mode == 'bi' else 'Trilateration'} "
                         f"Visualization - {'Simulation' if self.simulation else 'Live Data'}")
        
        # Enable tight layout
        plt.tight_layout()
    
    def update(self, frame):
        """Update function for animation"""
        if self.simulation:
            self.generate_simulated_data(frame)
        else:
            self.receive_uwb_data()
        
        # Calculate tag position if distances are available
        all_distances_available = all(d is not None for d in self.distances)
        
        if all_distances_available:
            if self.mode == 'bi':
                self.calculate_bilateration()
            else:
                self.calculate_trilateration()
        
        # Update visualization
        self.update_visualization()
        
        # Return all artists that need to be redrawn
        artists_to_return = []
        for key, artist in self.artists.items():
            if isinstance(artist, list):
                artists_to_return.extend(artist)
            elif artist is not None:
                artists_to_return.append(artist)
        
        return artists_to_return
    
    def generate_simulated_data(self, frame):
        """Generate simulated UWB distances for testing"""
        # Simulate a circular motion of the tag
        radius = 200  # cm
        angular_speed = 0.05  # radians per frame
        center_x = (self.anchors[0][0] + self.anchors[1][0]) / 2
        center_y = self.reference_y if self.mode == 'bi' else 300
        
        # Calculate tag position
        angle = frame * angular_speed
        tag_x = center_x + radius * math.cos(angle)
        tag_y = center_y + radius * math.sin(angle)
        
        # Simulate tag position
        self.tag_position = (tag_x, tag_y, self.anchors[0][2])
        
        # Calculate distances to anchors (with some noise)
        for i, anchor in enumerate(self.anchors):
            dx = tag_x - anchor[0]
            dy = tag_y - anchor[1]
            dz = self.anchors[0][2] - anchor[2]  # Z difference
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # Add some noise (±5 cm)
            noise = np.random.normal(0, 5)
            self.distances[i] = distance + noise
    
    def receive_uwb_data(self):
        """Receive UWB data via UDP socket"""
        try:
            # Try to receive data with timeout
            data, addr = self.socket.recvfrom(1024)
            
            try:
                # Decode and parse the JSON data
                json_data = json.loads(data.decode('utf-8'))
                
                # Process the data
                if all(k in json_data for k in ("device_address", "distance")):
                    device_address = json_data.get("device_address")
                    distance_str = json_data.get("distance")
                    
                    # Extract distance value (in cm)
                    if isinstance(distance_str, str) and "cm" in distance_str:
                        distance_value = float(distance_str.replace(" cm", "").strip())
                        
                        # Update respective distances based on device address
                        if device_address == "10":
                            self.distances[0] = distance_value
                        elif device_address == "11":
                            self.distances[1] = distance_value
                        elif device_address == "12" and self.mode == 'tri':
                            self.distances[2] = distance_value
            
            except json.JSONDecodeError:
                pass
            except Exception as e:
                print(f"Error processing JSON data: {e}")
                
        except socket.timeout:
            # This is normal, just continue
            pass
        except Exception as e:
            print(f"Error receiving UDP data: {e}")
    
    def calculate_bilateration(self):
        """Calculate tag position using bilateration"""
        # Extract anchor coordinates
        x1, y1, _ = self.anchors[0]
        x2, y2, _ = self.anchors[1]
        z = self.anchors[0][2]  # Z coordinate (assumed same as anchors)
        
        # Get distances
        d1 = self.distances[0]
        d2 = self.distances[1]
        
        # Distance between anchors
        d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        # Check if the position can be determined
        if d == 0:
            print("Anchors are at the same position, cannot determine location")
            return None
        
        # Check if the distances make sense
        if d1 + d2 < d:
            print(f"Invalid distances: sum ({d1 + d2}) less than anchor separation ({d})")
            return None
        
        # Calculate position using circle intersection formula
        # First, determine the x-coordinate in the simplified coordinate system
        x = (d1**2 - d2**2 + d**2) / (2 * d)
        
        # Check if the circles actually intersect
        dist_squared = d1**2 - x**2
        if dist_squared < 0:
            print("No valid intersection found - measurements may be inaccurate")
            # Adjust to find nearest valid solution
            dist_squared = 0
        
        # Calculate y (two possible solutions: y and -y)
        y = math.sqrt(dist_squared)
        
        # Now transform back to the original coordinate system
        # First, find the direction vector from anchor1 to anchor2
        dx = (x2 - x1) / d
        dy = (y2 - y1) / d
        
        # Perpendicular direction vector (-dy, dx)
        perpx = -dy
        perpy = dx
        
        # Calculate the two possible solutions
        solution1_x = x1 + dx * x + perpx * y
        solution1_y = y1 + dy * x + perpy * y
        
        solution2_x = x1 + dx * x - perpx * y
        solution2_y = y1 + dy * x - perpy * y
        
        # Store both solutions for visualization
        self.possible_positions = [(solution1_x, solution1_y), (solution2_x, solution2_y)]
        
        # Decide which solution to use based on preference
        if self.tag_position is not None:
            # Use the solution closest to the previous position
            prev_x, prev_y, _ = self.tag_position
            
            dist1 = math.sqrt((solution1_x - prev_x)**2 + (solution1_y - prev_y)**2)
            dist2 = math.sqrt((solution2_x - prev_x)**2 + (solution2_y - prev_y)**2)
            
            if dist1 <= dist2:
                self.tag_position = (solution1_x, solution1_y, z)
            else:
                self.tag_position = (solution2_x, solution2_y, z)
        else:
            # No previous position to compare with
            # Choose based on prefer_positive_y setting or reference_y
            if self.prefer_positive_y:
                if solution1_y >= 0 and solution2_y < 0:
                    self.tag_position = (solution1_x, solution1_y, z)
                elif solution2_y >= 0 and solution1_y < 0:
                    self.tag_position = (solution2_x, solution2_y, z)
                else:
                    # If both solutions have same sign for y, choose the one closest to reference_y
                    dist1_to_ref = abs(solution1_y - self.reference_y)
                    dist2_to_ref = abs(solution2_y - self.reference_y)
                    
                    if dist1_to_ref <= dist2_to_ref:
                        self.tag_position = (solution1_x, solution1_y, z)
                    else:
                        self.tag_position = (solution2_x, solution2_y, z)
            else:
                # Choose the one closest to reference_y
                dist1_to_ref = abs(solution1_y - self.reference_y)
                dist2_to_ref = abs(solution2_y - self.reference_y)
                
                if dist1_to_ref <= dist2_to_ref:
                    self.tag_position = (solution1_x, solution1_y, z)
                else:
                    self.tag_position = (solution2_x, solution2_y, z)
        
        # Update position history
        self.update_position_history()
    
    def calculate_trilateration(self):
        """Calculate tag position using trilateration"""
        # Extract anchor coordinates
        x1, y1, _ = self.anchors[0]
        x2, y2, _ = self.anchors[1]
        x3, y3, _ = self.anchors[2]
        z = self.anchors[0][2]  # Z coordinate (assumed same as anchors)
        
        # Get distances
        d1 = self.distances[0]
        d2 = self.distances[1]
        d3 = self.distances[2]
        
        # Trilateration algorithm
        A = 2*x2 - 2*x1
        B = 2*y2 - 2*y1
        C = d1**2 - d2**2 - x1**2 + x2**2 - y1**2 + y2**2
        D = 2*x3 - 2*x2
        E = 2*y3 - 2*y2
        F = d2**2 - d3**2 - x2**2 + x3**2 - y2**2 + y3**2
        
        # Check for potential division by zero or other numerical issues
        denominator = E*A - B*D
        if abs(denominator) < 1e-6:
            print("Trilateration math error: denominator near zero")
            return None
        
        # Calculate tag position
        x = (C*E - F*B) / denominator
        y = (C*D - A*F) / (B*D - A*E)
        
        # Update tag position
        self.tag_position = (x, y, z)
        
        # Update position history
        self.update_position_history()
    
    def update_position_history(self):
        """Update the position history trail"""
        if self.tag_position:
            self.position_history.append((self.tag_position[0], self.tag_position[1]))
            if len(self.position_history) > self.max_history:
                self.position_history.pop(0)
    
    def update_visualization(self):
        """Update visualization elements"""
        # Update distance circles
        for i, (distance, artist) in enumerate(zip(self.distances, self.artists['distances'])):
            if distance is not None:
                artist.set_radius(distance)
                artist.set_visible(True)
            else:
                artist.set_visible(False)
        
        # Update tag position
        if self.tag_position:
            self.artists['tag'].set_data(self.tag_position[0], self.tag_position[1])
            self.artists['tag'].set_visible(True)
        else:
            self.artists['tag'].set_visible(False)
        
        # Update trail
        if self.position_history:
            x_vals = [pos[0] for pos in self.position_history]
            y_vals = [pos[1] for pos in self.position_history]
            self.artists['trail'].set_data(x_vals, y_vals)
            self.artists['trail'].set_visible(True)
        else:
            self.artists['trail'].set_visible(False)
        
        # Update possible positions for bilateration
        if self.mode == 'bi' and hasattr(self, 'possible_positions'):
            x_vals = [pos[0] for pos in self.possible_positions]
            y_vals = [pos[1] for pos in self.possible_positions]
            self.artists['possible'].set_data(x_vals, y_vals)
            self.artists['possible'].set_visible(True)
        
        # Update text information
        text = "Tag Position:\n"
        if self.tag_position:
            text += f"X: {self.tag_position[0]:.2f} cm\n"
            text += f"Y: {self.tag_position[1]:.2f} cm\n"
            text += f"Z: {self.tag_position[2]:.2f} cm\n\n"
        else:
            text += "Unknown\n\n"
            
        text += "Distances:\n"
        for i, distance in enumerate(self.distances):
            if distance is not None:
                text += f"A{i+1}: {distance:.2f} cm\n"
            else:
                text += f"A{i+1}: Unknown\n"
                
        # Add bilateration specific info
        if self.mode == 'bi':
            text += f"\nReference Y: {self.reference_y} cm\n"
            text += f"Prefer positive Y: {self.prefer_positive_y}\n"
        
        self.artists['text_info'].set_text(text)
    
    def run(self):
        """Run the animation"""
        ani = FuncAnimation(self.fig, self.update, interval=100, blit=True)
        plt.show()

def parse_arguments():
    parser = argparse.ArgumentParser(description='Visualize UWB anchor and tag positions.')
    
    parser.add_argument('--mode', choices=['bi', 'tri'], default='bi',
                        help='Mode: bilateration (bi) or trilateration (tri)')
    parser.add_argument('--simulation', action='store_true',
                        help='Use simulated data instead of real UDP data')
    parser.add_argument('--reference_y', type=float, default=None,
                        help='Override reference Y value for bilateration')
    parser.add_argument('--positive_y', action='store_true', default=None,
                        help='Override prefer positive Y setting for bilateration')
    
    return parser.parse_args()

def main():
    # Parse command-line arguments
    args = parse_arguments()
    
    # Create and run visualizer
    visualizer = UWBVisualizer(
        mode=args.mode,
        simulation=args.simulation,
        reference_y=args.reference_y,
        prefer_positive_y=args.positive_y
    )
    
    # Run the visualizer
    visualizer.run()

if __name__ == "__main__":
    main()