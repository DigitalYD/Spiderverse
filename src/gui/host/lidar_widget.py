#!/usr/bin/env python3

import math
import threading
import time
import random
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QGroupBox
from PyQt5.QtGui import QPixmap, QPainter, QColor, QPen, QFont, QBrush, QPolygonF
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF, pyqtSignal, QObject, QProcess

# Use subprocess to get real LiDAR data without threading conflicts
import json
import subprocess
import os
import sys
import tempfile

# Path to data file for IPC
LIDAR_DATA_FILE = os.path.join(tempfile.gettempdir(), "lidar_data.json")

# Flag to track if we're using real data
REAL_DATA_AVAILABLE = False

# If available, import from config, otherwise use default
try:
    from config import ROS_LIDAR_TOPIC
except ImportError:
    ROS_LIDAR_TOPIC = "/scan"


class LidarScanProcessor(QObject):
    """Receives LaserScan data from ROS and provides it to the widget"""
    scan_ready = pyqtSignal(object)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.latest_scan = None
        self.running = True
        
        # Start ROS in a separate thread
        self.ros_thread = threading.Thread(target=self.run_ros)
        self.ros_thread.daemon = True
        self.ros_thread.start()
    
    def run_ros(self):
        """Run ROS node in a separate thread"""
        if not ROS_AVAILABLE:
            return
        
        # Initialize ROS if not already done
        if not rclpy.ok():
            try:
                rclpy.init()
            except Exception as e:
                print(f"Error initializing ROS: {e}")
                return
        
        # Create node and subscription
        try:
            node = Node('lidar_widget_processor')
            
            # Create QoS profile
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            # Subscribe to LaserScan topic
            subscription = node.create_subscription(
                LaserScan,
                ROS_LIDAR_TOPIC,
                self._scan_callback,
                qos_profile
            )
            
            node.get_logger().info(f'LiDAR data processor started, listening on {ROS_LIDAR_TOPIC}')
            
            # Spin node in this thread
            while rclpy.ok() and self.running:
                rclpy.spin_once(node, timeout_sec=0.1)
                
            # Clean up
            node.destroy_node()
        except Exception as e:
            print(f"Error in ROS thread: {e}")
    
    def _scan_callback(self, scan_msg):
        """Process LaserScan message and emit signal"""
        self.latest_scan = scan_msg
        self.scan_ready.emit(scan_msg)
    
    def stop(self):
        """Stop the thread"""
        self.running = False
        if self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0)


class LidarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        global REAL_DATA_AVAILABLE
        
        # Setup UI components
        self.setup_ui()
        
        # Configure visualization parameters
        self.zoom_factor = 50.0   # Default zoom (pixels per meter)
        self.max_range = 5.0      # Default max range to display (meters)
        self.show_grid = True     # Show distance grid
        self.show_angles = True   # Show angle markers
        self.filter_outliers = True  # Filter out readings with invalid ranges
        
        # Status flag
        self.waiting_for_data = True
        self.latest_scan = None
        self.lidar_process = None
        
        # Try to start the LiDAR receiver process
        try:
            # Path to the receiver script
            script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lidar_receiver.py")
            
            if os.path.exists(script_path):
                # Remove any existing data file
                if os.path.exists(LIDAR_DATA_FILE):
                    try:
                        os.unlink(LIDAR_DATA_FILE)
                    except:
                        pass
                        
                # Launch the process
                cmd = ["python3", script_path, LIDAR_DATA_FILE, ROS_LIDAR_TOPIC]
                self.lidar_process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    universal_newlines=True
                )
                
                REAL_DATA_AVAILABLE = True
                self.status_label.setText("Starting LiDAR data receiver...")
                self.status_label.setStyleSheet("color: yellow;")
                print(f"Started LiDAR receiver process, output file: {LIDAR_DATA_FILE}")
            else:
                print(f"LiDAR receiver script not found at {script_path}")
        except Exception as e:
            print(f"Error starting LiDAR receiver: {e}")
            REAL_DATA_AVAILABLE = False
            
        # Timer to check for data file updates
        self.data_timer = QTimer(self)
        self.data_timer.timeout.connect(self.check_lidar_data)
        self.data_timer.start(200)  # Reduced frequency: check every 200ms instead of 100ms
        
        # Timer for visualization updates
        self.viz_timer = QTimer(self)
        self.viz_timer.timeout.connect(self.update_visualization)
        self.viz_timer.start(200)  # Reduced frequency: update visualization every 200ms
        
        # Cache for visualization objects to avoid recreating them
        self.viz_cache = {
            'grid_pen': QPen(QColor(40, 40, 40)),
            'angle_pen': QPen(QColor(70, 70, 70)),
            'poly_brush': QBrush(QColor(0, 100, 150, 40)),
            'line_pen': QPen(QColor(0, 150, 200, 100), 1),
            'point_brush': QBrush(QColor(0, 180, 255, 180))
        }
        
        # Simulated scan data for demo mode
        self.demo_angle = 0
        self.last_update_time = time.time()
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Group box
        group_box = QGroupBox("LiDAR Data")
        group_box.setStyleSheet("QGroupBox { font-weight: bold; }")
        group_layout = QVBoxLayout()
        
        # LiDAR visualization
        self.lidar_label = QLabel()
        self.lidar_label.setAlignment(Qt.AlignCenter)
        
        # Set minimum size to ensure enough space for visualization
        self.lidar_label.setMinimumSize(400, 400)
        
        # Create pixmap for drawing
        self.pixmap = QPixmap(600, 600)
        self.pixmap.fill(Qt.black)
        self.lidar_label.setPixmap(self.pixmap)
        
        # Create a button to save the map
        from PyQt5.QtWidgets import QPushButton, QHBoxLayout
        
        from PyQt5.QtWidgets import QGridLayout
        
        # Button style (common properties)
        button_style = """
            QPushButton {
                border-radius: 6px;
                padding: 8px 12px;
                font-weight: bold;
                font-size: 12px;
                border: 1px solid #555555;
            }
            QPushButton:hover {
                background-color: #aaccff;
            }
            QPushButton:pressed {
                background-color: #88aadd;
            }
            QPushButton:disabled {
                background-color: #888888;
                color: #dddddd;
            }
        """
        
        # Use grid layout for multiple buttons
        map_button_layout = QGridLayout()
        map_button_layout.setSpacing(10)  # Add spacing between buttons
        
        # SLAM buttons in first row - all with grey background
        self.slam_button = QPushButton("Launch SLAM")
        self.slam_button.setToolTip("Launch regular SLAM (slam.sh)")
        self.slam_button.clicked.connect(lambda: self.launch_slam("slam.sh"))
        self.slam_button.setStyleSheet(button_style + """
            background-color: #dddddd;
            color: black;
        """)
        map_button_layout.addWidget(self.slam_button, 0, 0)
        
        self.bi_slam_button = QPushButton("Launch Bi-SLAM")
        self.bi_slam_button.setToolTip("Launch bilateration SLAM (bi_slam.sh)")
        self.bi_slam_button.clicked.connect(lambda: self.launch_slam("bi_slam.sh"))
        self.bi_slam_button.setStyleSheet(button_style + """
            background-color: #dddddd;
            color: black;
        """)
        map_button_layout.addWidget(self.bi_slam_button, 0, 1)
        
        self.tri_slam_button = QPushButton("Launch Tri-SLAM")
        self.tri_slam_button.setToolTip("Launch trilateration SLAM (tri_slam.sh)")
        self.tri_slam_button.clicked.connect(lambda: self.launch_slam("tri_slam.sh"))
        self.tri_slam_button.setStyleSheet(button_style + """
            background-color: #dddddd;
            color: black;
        """)
        map_button_layout.addWidget(self.tri_slam_button, 0, 2)
        
        # Add IMU-based SLAM button
        self.imu_slam_button = QPushButton("IMU SLAM")
        self.imu_slam_button.setToolTip("Launch SLAM with IMU odometry")
        self.imu_slam_button.clicked.connect(self.launch_imu_slam)
        self.imu_slam_button.setStyleSheet(button_style + """
            background-color: #dddddd;
            color: black;
        """)
        map_button_layout.addWidget(self.imu_slam_button, 0, 3)
        
        # Save map button in second row, spanning all columns - green background
        self.save_map_button = QPushButton("SAVE MAP")
        self.save_map_button.setToolTip("Save the current SLAM map to a file")
        self.save_map_button.clicked.connect(self.save_map)
        self.save_map_button.setStyleSheet(button_style + """
            background-color: #2ecc71;
            color: white;
            font-size: 14px;
            padding: 10px;
        """)
        map_button_layout.addWidget(self.save_map_button, 1, 0, 1, 4)  # row, col, rowspan, colspan (updated span for new button)
        
        # Current SLAM process tracking
        self.slam_process = None
        
        # Status label
        self.status_label = QLabel("Waiting for LiDAR data...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setFont(QFont("Monospace", 10))
        self.status_label.setStyleSheet("color: yellow;")
        
        group_layout.addWidget(self.lidar_label)
        group_layout.addLayout(map_button_layout)
        group_layout.addWidget(self.status_label)
        group_box.setLayout(group_layout)
        layout.addWidget(group_box)
    
    def check_lidar_data(self):
        """Check for updates in the LiDAR data file"""
        global REAL_DATA_AVAILABLE
        
        # Check if process is still running
        if self.lidar_process and self.lidar_process.poll() is not None:
            # Process has terminated
            rc = self.lidar_process.returncode
            stdout, stderr = self.lidar_process.communicate()
            print(f"LiDAR process terminated with code {rc}")
            print(f"STDOUT: {stdout}")
            print(f"STDERR: {stderr}")
            
            # Set to None to avoid checking again
            self.lidar_process = None
            REAL_DATA_AVAILABLE = False
            
            self.status_label.setText("LiDAR data connection lost. Running in demo mode.")
            self.status_label.setStyleSheet("color: orange;")
            return
            
        # Check if data file exists and has data
        if REAL_DATA_AVAILABLE and os.path.exists(LIDAR_DATA_FILE):
            try:
                modified_time = os.path.getmtime(LIDAR_DATA_FILE)
                # Only read if file was modified in the last second
                if time.time() - modified_time < 1.0:
                    with open(LIDAR_DATA_FILE, 'r') as f:
                        data = f.read().strip()
                        if data:
                            self.process_scan_data(data)
            except Exception as e:
                print(f"Error reading LiDAR data file: {e}")
    
    def process_scan_data(self, data_str):
        """Process scan data from the data file"""
        try:
            # Parse the JSON data
            data = json.loads(data_str)
            
            # Create a LaserScan-like object
            class ScanData:
                pass
                
            scan = ScanData()
            scan.angle_min = data['angle_min']
            scan.angle_max = data['angle_max']
            scan.angle_increment = data['angle_increment']
            scan.range_min = data['range_min']
            scan.range_max = data['range_max']
            scan.ranges = data['ranges']
            scan.angles = data['angles']  # This is our custom field
            
            # Store scan data
            self.latest_scan = scan
            self.waiting_for_data = False
            
            # Update status
            valid_count = len(scan.ranges)
            avg_range = sum(scan.ranges) / valid_count if valid_count > 0 else 0
            status = f"Live data | Points: {valid_count} | Avg: {avg_range:.2f}m"
            self.status_label.setText(status)
            self.status_label.setStyleSheet("color: lime;")
        except Exception as e:
            print(f"Error processing scan data: {e}")
    
    def update_visualization(self):
        """Update the visualization pixmap"""
        # Clear pixmap
        self.pixmap.fill(Qt.black)
        
        # Create painter
        painter = QPainter(self.pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Set up coordinates
        width = self.pixmap.width()
        height = self.pixmap.height()
        center_x = width // 2
        center_y = height // 2
        
        # Draw grid if enabled
        if self.show_grid:
            self.draw_grid(painter, center_x, center_y, width, height)
        
        # Draw angle markers if enabled
        if self.show_angles:
            self.draw_angles(painter, center_x, center_y, width, height)
        
        # Check if we have real data or should generate demo data
        if self.latest_scan is not None:
            # Use real data from file
            self.draw_scan(painter, center_x, center_y, self.latest_scan)
            self.waiting_for_data = False
        elif not REAL_DATA_AVAILABLE:
            # Generate demo data when real data isn't available
            self.draw_demo_scan(painter, center_x, center_y, width, height)
            self.waiting_for_data = False
        else:
            # Draw "waiting for data" indicator
            self.draw_waiting_message(painter, center_x, center_y)
            self.waiting_for_data = True
        
        # Draw current position marker
        self.draw_position_marker(painter, center_x, center_y)
        
        # Finish painting
        painter.end()
        
        # Update label
        self.lidar_label.setPixmap(self.pixmap)
    
    def draw_grid(self, painter, center_x, center_y, width, height):
        """Draw coordinate grid with distance markers"""
        # Use cached grid pen
        painter.setPen(self.viz_cache['grid_pen'])
        
        # Calculate max radius based on window size
        max_radius = min(center_x, center_y) - 20
        
        # Draw concentric circles
        for distance in range(1, int(self.max_range) + 1):
            radius = distance * self.zoom_factor
            if radius <= max_radius:
                # Convert to integers once to reduce conversions
                x = int(center_x - radius)
                y = int(center_y - radius)
                diam = int(radius * 2)
                painter.drawEllipse(x, y, diam, diam)
                
                # Only draw labels for even distances to reduce text rendering
                if distance % 2 == 0 or distance == 1:
                    # Draw distance label
                    font = QFont()
                    font.setPointSize(8)
                    painter.setFont(font)
                    painter.setPen(QColor(60, 60, 60))
                    painter.drawText(
                        QPointF(center_x + 5, center_y - radius + 15),
                        f"{distance}m"
                    )
        
        # Draw axis lines
        painter.drawLine(center_x, 0, center_x, height)
        painter.drawLine(0, center_y, width, center_y)
    
    def draw_angles(self, painter, center_x, center_y, width, height):
        """Draw angle markers for orientation"""
        max_radius = min(center_x, center_y) - 20
        
        # Draw angle markers (every 30 degrees)
        painter.setPen(QPen(QColor(70, 70, 70)))
        
        # Using standard 0-360 degree format
        for angle_deg in range(0, 361, 30):
            if angle_deg == 360:
                continue
                
            angle_rad = math.radians(angle_deg)
            
            # Calculate end points
            end_x = center_x + math.cos(angle_rad) * max_radius
            end_y = center_y - math.sin(angle_rad) * max_radius
            
            # Draw lines
            painter.drawLine(int(center_x), int(center_y), int(end_x), int(end_y))
            
            # Draw cardinal directions
            if angle_deg == 0:
                painter.drawText(center_x + 5, 15, "0°")           # Top (0°)
            elif angle_deg == 90:
                painter.drawText(width - 25, center_y - 5, "90°")  # Right (90°)
            elif angle_deg == 180:
                painter.drawText(center_x + 5, height - 5, "180°") # Bottom (180°)
            elif angle_deg == 270:
                painter.drawText(10, center_y - 5, "270°")         # Left (270°)
    
    def draw_waiting_message(self, painter, center_x, center_y):
        """Draw waiting message when no scan data is available"""
        message = "Waiting for LiDAR data..."
        
        font = QFont()
        font.setPointSize(14)
        painter.setFont(font)
        painter.setPen(QColor(255, 255, 0))
        painter.drawText(QRectF(center_x - 150, center_y, 300, 40), Qt.AlignCenter, message)
        
        # Draw spinning wait indicator
        angle = (math.pi * (time.time() % 2)) / 2
        spin_radius = self.max_range * self.zoom_factor / 3
        end_x = center_x + int(spin_radius * math.cos(angle))
        end_y = center_y + int(spin_radius * math.sin(angle))
        
        painter.setPen(QPen(QColor(255, 255, 0), 3))
        painter.drawLine(center_x, center_y, end_x, end_y)
    
    def draw_scan(self, painter, center_x, center_y, scan):
        """Draw the LiDAR scan data"""
        if scan is None:
            return
        
        # Check if we have our custom format with pre-calculated angles
        has_custom_angles = hasattr(scan, 'angles') and len(scan.angles) == len(scan.ranges)
        
        # Get scan parameters
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment
        range_min = scan.range_min
        range_max = scan.range_max
        
        # Create a polygon for connecting points
        scan_polygon = QPolygonF()
        scan_points = []  # Store valid points for drawing
        valid_point_count = 0
        
        # Sort the points to ensure they're drawn in correct order
        point_data = []
        
        # Calculate how many points to skip for optimization
        # Skip factor increases with higher point counts
        if hasattr(scan, 'ranges'):
            total_points = len(scan.ranges)
            # Only subsample if we have lots of points
            if total_points > 360:  # More than 1 point per degree
                skip_factor = 2  # Skip every other point
            else:
                skip_factor = 1  # Don't skip points
        else:
            skip_factor = 1
        
        # Process each scan point (with skipping for optimization)
        for i, range_value in enumerate(scan.ranges):
            # Skip points based on skip_factor
            if skip_factor > 1 and i % skip_factor != 0:
                continue
                
            # Skip invalid readings
            if math.isinf(range_value) or math.isnan(range_value):
                continue
                
            # Skip readings outside the range limits if filtering is enabled
            if self.filter_outliers and (range_value < range_min or range_value > range_max):
                continue
            
            # Skip points beyond max display range
            if range_value > self.max_range:
                continue
            
            # Get the angle for this point (either from pre-calculated or calculate it)
            if has_custom_angles:
                angle = scan.angles[i]
            else:
                angle = angle_min + (i * angle_increment)
            
            # Store point data as (angle, range) for sorting
            point_data.append((angle, range_value))
            valid_point_count += 1
        
        # Sort points by angle for consistent display
        point_data.sort(key=lambda p: p[0])
        
        # Convert sorted points to screen coordinates
        for angle, range_value in point_data:
            # Convert polar to cartesian coordinates
            # Correct coordinate system: 0° is front, 90° is left
            # Need to mirror horizontally to match visualize_lidar.py
            x = -range_value * math.cos(angle)  # Negate x to mirror horizontally
            y = range_value * math.sin(angle)
            
            # Scale and translate to screen coordinates
            screen_x = center_x + x * self.zoom_factor
            screen_y = center_y - y * self.zoom_factor
            
            point = QPointF(screen_x, screen_y)
            scan_polygon.append(point)
            scan_points.append((screen_x, screen_y))
        
        # Draw scan points
        if scan_points:
            # Draw scan polygon (fill)
            painter.setPen(Qt.NoPen)
            painter.setBrush(self.viz_cache['poly_brush'])
            if len(scan_points) > 2:
                # Add center point to close the polygon
                centered_polygon = QPolygonF(scan_polygon)
                centered_polygon.append(QPointF(center_x, center_y))
                painter.drawPolygon(centered_polygon)
            
            # Draw point-to-point connections (outline)
            painter.setPen(self.viz_cache['line_pen'])
            if len(scan_points) > 1:
                # Draw lines more efficiently using a polygon instead of individual lines
                outline_polygon = QPolygonF()
                for x, y in scan_points:
                    outline_polygon.append(QPointF(x, y))
                # Draw the polygon outline
                painter.drawPolyline(outline_polygon)
            
            # Always draw points, but adjust size based on point count for performance
            painter.setPen(Qt.NoPen)
            painter.setBrush(self.viz_cache['point_brush'])
            
            # Adjust point size based on number of points
            point_size = 3
            if len(scan_points) > 500:
                point_size = 2
            if len(scan_points) > 1000:
                point_size = 1
                
            for x, y in scan_points:
                painter.drawEllipse(QPointF(x, y), point_size, point_size)
                
            # Update status with point count info
            self.status_label.setText(f"LiDAR points: {valid_point_count}")
            self.status_label.setStyleSheet("color: lime;")
    
    def draw_demo_scan(self, painter, center_x, center_y, width, height):
        """Draw simulated LiDAR data for demo mode"""
        # Calculate time increment for animation
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        
        # Animate rotation
        self.demo_angle += 100 * dt  # Degrees per second
        if self.demo_angle >= 360:
            self.demo_angle = 0
        
        # Create a polygon for connecting points
        scan_polygon = QPolygonF()
        scan_points = []
        
        # Generate points around a circle with simulated objects
        for angle_deg in range(0, 360, 2):  # Every 2 degrees
            
            # Add some "objects" to the scan at fixed angles
            if 45 <= angle_deg <= 70:
                # "Wall" on one side
                distance = self.max_range * 0.3
            elif 180 <= angle_deg <= 200:
                # Another "object"
                distance = self.max_range * 0.6
            elif 300 <= angle_deg <= 320:
                # A third "object"
                distance = self.max_range * 0.45
            else:
                # Random distance for other points
                base_distance = self.max_range * 0.8
                noise = random.uniform(-0.1, 0.1) * self.max_range
                distance = max(0.2, min(self.max_range, base_distance + noise))
            
            # Add noise to make it look more natural (simulate some missing points)
            if random.random() < 0.05:  # 5% chance for a "missing" point
                continue
                
            # Convert to radians
            angle_rad = math.radians(angle_deg + self.demo_angle)  # Add rotation
            
            # Calculate x, y
            x = distance * math.cos(angle_rad)
            y = -distance * math.sin(angle_rad)  # Negate for PyQt coordinate system
            
            # Scale and translate to screen coordinates
            screen_x = center_x + x * self.zoom_factor
            screen_y = center_y + y * self.zoom_factor
            
            point = QPointF(screen_x, screen_y)
            scan_polygon.append(point)
            scan_points.append((screen_x, screen_y))
        
        # Draw scan points
        if scan_points:
            # Draw scan polygon (fill)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(0, 100, 150, 40)))
            if len(scan_points) > 2:
                # Add center point to close the polygon
                centered_polygon = QPolygonF(scan_polygon)
                centered_polygon.append(QPointF(center_x, center_y))
                painter.drawPolygon(centered_polygon)
            
            # Draw point-to-point connections (outline)
            painter.setPen(QPen(QColor(0, 150, 200, 100), 1))
            if len(scan_points) > 1:
                for i in range(len(scan_points) - 1):
                    x1, y1 = scan_points[i]
                    x2, y2 = scan_points[i + 1]
                    painter.drawLine(int(x1), int(y1), int(x2), int(y2))
            
            # Connect last point to first to complete the loop if we have enough points
            if len(scan_points) > 2:
                x1, y1 = scan_points[-1]
                x2, y2 = scan_points[0]
                painter.drawLine(int(x1), int(y1), int(x2), int(y2))
            
            # Draw individual points
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(0, 180, 255, 180)))
            for x, y in scan_points:
                painter.drawEllipse(QPointF(x, y), 3, 3)
            
            # Update status in demo mode with a moving indicator to show it's live
            indicator = "◐◓◑◒"[int(time.time() * 2) % 4]
            self.status_label.setText(f"Demo mode {indicator} | {len(scan_points)} points")
            self.status_label.setStyleSheet("color: cyan;")
    
    def draw_position_marker(self, painter, center_x, center_y):
        """Draw the current position marker (robot position)"""
        # Draw marker at current position
        painter.setPen(QPen(QColor(255, 100, 100), 2))
        marker_size = 12
        painter.drawLine(int(center_x - marker_size/2), int(center_y), int(center_x + marker_size/2), int(center_y))
        painter.drawLine(int(center_x), int(center_y - marker_size/2), int(center_x), int(center_y + marker_size/2))
        
        # Draw circle
        painter.setPen(QPen(QColor(255, 80, 80), 2))
        painter.setBrush(QBrush(QColor(255, 50, 50, 150)))
        painter.drawEllipse(int(center_x - 5), int(center_y - 5), 10, 10)
        
        # Draw forward direction indicator (0 degrees, mirrored to left)
        painter.setPen(QPen(QColor(255, 200, 100), 2))
        forward_x = center_x - marker_size * 1.5  # Mirrored to point left
        forward_y = center_y
        painter.drawLine(int(center_x), int(center_y), int(forward_x), int(forward_y))
        
        # Draw legend for angle reference
        if self.latest_scan:
            font = QFont()
            font.setPointSize(8)
            painter.setFont(font)
            painter.setPen(QColor(200, 200, 200))
            painter.drawText(QRectF(10, 10, 400, 20), "0° is left ←, -90° is down ↓, +90° is up ↑ (mirrored)")
    
    def launch_slam(self, script_name):
        """Launch one of the SLAM scripts"""
        from PyQt5.QtWidgets import QMessageBox
        
        # Check if a SLAM process is already running
        if self.slam_process is not None:
            if self.slam_process.poll() is None:  # Still running
                response = QMessageBox.question(
                    self,
                    "SLAM Already Running",
                    f"A SLAM process is already running. Do you want to stop it and start {script_name}?",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No
                )
                
                if response == QMessageBox.No:
                    return
                    
                # Kill the existing process
                try:
                    self.slam_process.terminate()
                    self.slam_process.wait(timeout=3)
                    if self.slam_process.poll() is None:
                        self.slam_process.kill()
                except Exception as e:
                    print(f"Error terminating existing SLAM process: {e}")
        
        # Path to the SLAM script
        script_path = os.path.expanduser(f"~/Documents/Spiderverse/lidar_ws/{script_name}")
        
        if not os.path.exists(script_path):
            QMessageBox.critical(self, "Error", f"SLAM script not found at {script_path}")
            return
            
        # Update button states
        self.slam_button.setEnabled(False)
        self.bi_slam_button.setEnabled(False)
        self.tri_slam_button.setEnabled(False)
        
        # Update status
        self.status_label.setText(f"Starting {script_name}...")
        self.status_label.setStyleSheet("color: orange;")
        
        # Force update UI
        from PyQt5.QtCore import QCoreApplication
        QCoreApplication.processEvents()
        
        try:
            # Create a new terminal window to run the SLAM script
            # This allows users to see the SLAM output and interact with it if needed
            if sys.platform == 'linux':
                # For Linux, use xterm, gnome-terminal, or konsole
                for terminal in ['xterm', 'gnome-terminal', 'konsole']:
                    if subprocess.call(['which', terminal], stdout=subprocess.PIPE) == 0:
                        if terminal == 'gnome-terminal':
                            cmd = [terminal, '--', 'bash', '-c', f"cd ~/Documents/Spiderverse/lidar_ws && bash {script_name}; exec bash"]
                        elif terminal == 'konsole':
                            cmd = [terminal, '-e', f"cd ~/Documents/Spiderverse/lidar_ws && bash {script_name}; exec bash"]
                        else:  # xterm
                            cmd = [terminal, '-e', f"cd ~/Documents/Spiderverse/lidar_ws && bash {script_name}; exec bash"]
                        
                        self.slam_process = subprocess.Popen(cmd)
                        self.status_label.setText(f"Running {script_name} in separate terminal")
                        self.status_label.setStyleSheet("color: lime;")
                        break
                else:
                    # No terminal found, run script directly
                    cmd = ["bash", "-c", f"cd ~/Documents/Spiderverse/lidar_ws && ./imu_slam.sh"]
                    self.slam_process = subprocess.Popen(
                        cmd,
                        cwd=os.path.expanduser("~/Documents/Spiderverse/lidar_ws")
                    )
                    self.status_label.setText(f"Running {script_name} (no terminal available)")
                    self.status_label.setStyleSheet("color: lime;")
            else:
                # For other platforms
                cmd = ["bash", "-c", f"cd ~/Documents/Spiderverse/lidar_ws && ./imu_slam.sh"]
                self.slam_process = subprocess.Popen(
                    cmd,
                    cwd=os.path.expanduser("~/Documents/Spiderverse/lidar_ws")
                )
                self.status_label.setText(f"Running {script_name}")
                self.status_label.setStyleSheet("color: lime;")
            
            # Start a timer to check process status
            self.process_timer = QTimer()
            self.process_timer.timeout.connect(self.check_slam_process)
            self.process_timer.start(1000)  # Check every second
            
        except Exception as e:
            self.status_label.setText(f"Error launching {script_name}: {e}")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(self, "Error", f"Failed to launch SLAM script:\n{str(e)}")
            
            # Re-enable buttons
            self.slam_button.setEnabled(True)
            self.bi_slam_button.setEnabled(True)
            self.tri_slam_button.setEnabled(True)
    
    def check_slam_process(self):
        """Check if the SLAM process is still running"""
        if self.slam_process and self.slam_process.poll() is not None:
            # Process has terminated
            return_code = self.slam_process.returncode
            
            if return_code == 0:
                self.status_label.setText("SLAM process completed successfully")
            else:
                self.status_label.setText(f"SLAM process terminated with code {return_code}")
                self.status_label.setStyleSheet("color: orange;")
            
            # Re-enable buttons
            self.slam_button.setEnabled(True)
            self.bi_slam_button.setEnabled(True)
            self.tri_slam_button.setEnabled(True)
            self.imu_slam_button.setEnabled(True)
            
            # Stop the timer
            self.process_timer.stop()
    
    def launch_imu_slam(self):
        """Launch SLAM with IMU-based odometry"""
        from PyQt5.QtWidgets import QMessageBox
        
        # Check if a SLAM process is already running
        if self.slam_process is not None:
            if self.slam_process.poll() is None:  # Still running
                response = QMessageBox.question(
                    self,
                    "SLAM Already Running",
                    "A SLAM process is already running. Do you want to stop it and start IMU-based SLAM?",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No
                )
                
                if response == QMessageBox.No:
                    return
                    
                # Kill the existing process
                try:
                    self.slam_process.terminate()
                    self.slam_process.wait(timeout=3)
                    if self.slam_process.poll() is None:
                        self.slam_process.kill()
                except Exception as e:
                    print(f"Error terminating existing SLAM process: {e}")
        
        # Path to the IMU SLAM shell script in lidar_ws
        script_path = os.path.expanduser("~/Documents/Spiderverse/lidar_ws/imu_slam.sh")
        
        if not os.path.exists(script_path):
            QMessageBox.critical(self, "Error", f"IMU SLAM script not found at {script_path}")
            return
            
        # Update button states
        self.slam_button.setEnabled(False)
        self.bi_slam_button.setEnabled(False)
        self.tri_slam_button.setEnabled(False)
        self.imu_slam_button.setEnabled(False)
        
        # Update status
        self.status_label.setText("Starting IMU-based SLAM...")
        self.status_label.setStyleSheet("color: orange;")
        
        # Force update UI
        from PyQt5.QtCore import QCoreApplication
        QCoreApplication.processEvents()
        
        try:
            # Create a new terminal window to run the SLAM script
            if sys.platform == 'linux':
                # For Linux, use xterm, gnome-terminal, or konsole
                for terminal in ['xterm', 'gnome-terminal', 'konsole']:
                    if subprocess.call(['which', terminal], stdout=subprocess.PIPE) == 0:
                        if terminal == 'gnome-terminal':
                            cmd = [terminal, '--', 'bash', '-c', f"cd ~/Documents/Spiderverse/lidar_ws && ./imu_slam.sh; exec bash"]
                        elif terminal == 'konsole':
                            cmd = [terminal, '-e', f"cd ~/Documents/Spiderverse/lidar_ws && ./imu_slam.sh; exec bash"]
                        else:  # xterm
                            cmd = [terminal, '-e', f"cd ~/Documents/Spiderverse/lidar_ws && ./imu_slam.sh; exec bash"]
                        
                        self.slam_process = subprocess.Popen(cmd)
                        self.status_label.setText("Running IMU-based SLAM in separate terminal")
                        self.status_label.setStyleSheet("color: lime;")
                        break
                else:
                    # No terminal found, run script directly
                    cmd = ["python3", script_path]
                    self.slam_process = subprocess.Popen(
                        cmd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        universal_newlines=True
                    )
                    self.status_label.setText("Running IMU-based SLAM (no terminal available)")
                    self.status_label.setStyleSheet("color: lime;")
            else:
                # For other platforms
                cmd = ["python3", script_path]
                self.slam_process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    universal_newlines=True
                )
                self.status_label.setText("Running IMU-based SLAM")
                self.status_label.setStyleSheet("color: lime;")
            
            # Start a timer to check process status
            self.process_timer = QTimer()
            self.process_timer.timeout.connect(self.check_slam_process)
            self.process_timer.start(1000)  # Check every second
            
        except Exception as e:
            self.status_label.setText(f"Error launching IMU-based SLAM: {e}")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(self, "Error", f"Failed to launch IMU-based SLAM:\n{str(e)}")
            
            # Re-enable buttons
            self.slam_button.setEnabled(True)
            self.bi_slam_button.setEnabled(True)
            self.tri_slam_button.setEnabled(True)
            self.imu_slam_button.setEnabled(True)
    
    def save_map(self):
        """Run the save_map.sh script to save the current SLAM map using QProcess"""
        from PyQt5.QtWidgets import QMessageBox
        
        # Path to the save_map.sh script
        script_path = os.path.expanduser("~/Documents/Spiderverse/lidar_ws/save_map.sh")
        
        if not os.path.exists(script_path):
            QMessageBox.critical(self, "Error", f"Save map script not found at {script_path}")
            return
            
        # Disable the button while running
        self.save_map_button.setEnabled(False)
        self.save_map_button.setText("Saving...")
        self.status_label.setText("Saving map... Please wait...")
        self.status_label.setStyleSheet("color: orange;")

        # Store output data
        self.map_stdout = ""
        self.map_stderr = ""
        
        # Create a QProcess
        self.map_process = QProcess(self)
        self.map_process.setWorkingDirectory(os.path.expanduser("~/Documents/Spiderverse/lidar_ws"))
        
        # Connect signals
        self.map_process.finished.connect(self.on_map_save_finished)
        self.map_process.readyReadStandardOutput.connect(self.read_map_stdout)
        self.map_process.readyReadStandardError.connect(self.read_map_stderr)
        
        # Setup timeout timer
        self.map_timeout_timer = QTimer(self)
        self.map_timeout_timer.timeout.connect(self.on_map_save_timeout)
        self.map_timeout_timer.setSingleShot(True)
        self.map_timeout_timer.start(120000)  # 2 minute timeout
        
        # Start the process
        self.map_process.start("bash", [script_path])
        
    def read_map_stdout(self):
        """Read standard output from map save process"""
        data = self.map_process.readAllStandardOutput().data().decode('utf-8')
        self.map_stdout += data
        
    def read_map_stderr(self):
        """Read standard error from map save process"""
        data = self.map_process.readAllStandardError().data().decode('utf-8')
        self.map_stderr += data
        
    def on_map_save_timeout(self):
        """Handle timeout when saving map takes too long"""
        from PyQt5.QtWidgets import QMessageBox
        
        if hasattr(self, 'map_process') and self.map_process.state() != QProcess.NotRunning:
            self.map_process.kill()
            self.status_label.setText("Map save timed out")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(self, "Timeout", "Save map operation timed out after 2 minutes")
            
            # Re-enable the button
            self.save_map_button.setEnabled(True)
            self.save_map_button.setText("Save Map")
            
    def on_map_save_finished(self, exit_code, exit_status):
        """Handle completion of the map save process"""
        from PyQt5.QtWidgets import QMessageBox
        import re
        
        # Stop the timeout timer
        if hasattr(self, 'map_timeout_timer') and self.map_timeout_timer.isActive():
            self.map_timeout_timer.stop()
        
        # Check if the process was successful
        if exit_code == 0 and "SUCCESS" in self.map_stdout:
            # Extract the map file path from the output
            map_path_match = re.search(r"Map file: (.*\.pgm)", self.map_stdout)
            map_path = map_path_match.group(1) if map_path_match else "Unknown location"
            
            # Show success message
            self.status_label.setText(f"Map saved successfully to {map_path}")
            self.status_label.setStyleSheet("color: lime;")
            
            QMessageBox.information(
                self, 
                "Map Saved", 
                f"Map successfully saved to:\n{map_path}\n\nOutput:\n{self.map_stdout.strip()}"
            )
        else:
            error_msg = self.map_stderr if self.map_stderr else self.map_stdout
            self.status_label.setText(f"Error saving map")
            self.status_label.setStyleSheet("color: red;")
            
            QMessageBox.warning(
                self, 
                "Map Save Failed", 
                f"Failed to save map. Is Cartographer running?\n\nError:\n{error_msg}"
            )
            
        # Re-enable the button
        self.save_map_button.setEnabled(True)
        self.save_map_button.setText("Save Map")
    
    def closeEvent(self, event):
        """Handle widget close event"""
        # Stop all timers
        for timer_attr in ['viz_timer', 'data_timer', 'process_timer', 'map_timeout_timer']:
            if hasattr(self, timer_attr):
                timer = getattr(self, timer_attr)
                if timer and timer.isActive():
                    timer.stop()
            
        # Terminate the LiDAR process if it's running
        if self.lidar_process is not None:
            print("Terminating LiDAR data receiver process...")
            try:
                self.lidar_process.terminate()
                self.lidar_process.wait(timeout=2)
                
                # Force kill if it didn't terminate properly
                if self.lidar_process.poll() is None:
                    self.lidar_process.kill()
            except Exception as e:
                print(f"Error terminating LiDAR process: {e}")
                
        # Terminate SLAM process if it's running
        if hasattr(self, 'slam_process') and self.slam_process is not None:
            print("Terminating SLAM process...")
            try:
                self.slam_process.terminate()
                self.slam_process.wait(timeout=2)
                
                # Force kill if it didn't terminate properly
                if self.slam_process.poll() is None:
                    self.slam_process.kill()
            except Exception as e:
                print(f"Error terminating SLAM process: {e}")
        
        # Terminate map save process if it's running
        if hasattr(self, 'map_process') and self.map_process is not None:
            print("Terminating map save process...")
            try:
                if self.map_process.state() != QProcess.NotRunning:
                    self.map_process.kill()
            except Exception as e:
                print(f"Error terminating map save process: {e}")
                
        # Clean up data file
        if os.path.exists(LIDAR_DATA_FILE):
            try:
                os.unlink(LIDAR_DATA_FILE)
            except Exception as e:
                print(f"Error removing data file: {e}")
                
        event.accept()