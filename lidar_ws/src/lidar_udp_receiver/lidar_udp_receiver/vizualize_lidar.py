#!/usr/bin/env python3

import sys
import math
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QSlider, QCheckBox, QPushButton)
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen, QPolygonF, QFont, QPainterPath
from PyQt5.QtCore import Qt, QPointF, QRectF, QTimer, pyqtSignal, QObject


class LidarVisualizerNode(Node, QObject):
    """
    Node to receive and visualize LaserScan messages in -180 to 180 degree range
    """
    scan_updated = pyqtSignal(object)  # Signal to notify GUI when new scan data is available
    
    def __init__(self):
        Node.__init__(self, 'lidar_visualizer')
        QObject.__init__(self)
        
        # Create QoS profile for reliable scan data
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to the scan topic
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile
        )
        
        self.latest_scan = None
        self.get_logger().info('LiDAR visualizer started, listening on /scan topic')
    
    def scan_callback(self, scan_msg):
        """Process incoming LaserScan message"""
        # Print first message received to confirm subscription is working
        if self.latest_scan is None:
            self.get_logger().info(f"First scan received! Points: {len(scan_msg.ranges)}")
            self.get_logger().info(f"Angle range: {scan_msg.angle_min:.2f} to {scan_msg.angle_max:.2f} rad")
            self.get_logger().info(f"Angle increment: {scan_msg.angle_increment:.4f} rad")
            self.get_logger().info(f"Range limits: {scan_msg.range_min:.2f} to {scan_msg.range_max:.2f} m")
            
            # Count NaN values
            nan_count = sum(1 for r in scan_msg.ranges if math.isnan(r))
            self.get_logger().info(f"NaN count: {nan_count}/{len(scan_msg.ranges)} points")
        
        self.latest_scan = scan_msg
        self.scan_updated.emit(scan_msg)  # Emit signal with the scan data


class LidarVisualizerWindow(QMainWindow):
    """
    Main window for LiDAR visualization
    """
    def __init__(self, lidar_node):
        super().__init__()
        self.lidar_node = lidar_node
        self.latest_scan = None
        
        # Connect to the scan_updated signal
        self.lidar_node.scan_updated.connect(self.update_scan_data)
        
        # Setup visualization parameters
        self.zoom_factor = 50.0   # Default zoom (pixels per meter)
        self.max_range = 10.0     # Default max range to display (meters)
        self.show_grid = True     # Show distance grid
        self.show_angles = True   # Show angle markers
        self.filter_outliers = True  # Filter out readings with invalid ranges
        
        self.initUI()
        
        # Setup timer for regular updates
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.visualizer.update)
        self.update_timer.start(100)  # Update every 100ms
    
    def initUI(self):
        """Initialize the user interface"""
        self.setWindowTitle('LiDAR Visualizer (Horizontally Mirrored)')
        self.setGeometry(100, 100, 800, 800)
        
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Create visualizer widget
        self.visualizer = LidarVisualizerWidget(self)
        main_layout.addWidget(self.visualizer)
        
        # Create controls layout
        controls_layout = QHBoxLayout()
        main_layout.addLayout(controls_layout)
        
        # Zoom control
        zoom_layout = QVBoxLayout()
        zoom_label = QLabel("Zoom:")
        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setMinimum(10)
        self.zoom_slider.setMaximum(200)
        self.zoom_slider.setValue(int(self.zoom_factor))
        self.zoom_slider.valueChanged.connect(self.update_zoom)
        zoom_layout.addWidget(zoom_label)
        zoom_layout.addWidget(self.zoom_slider)
        controls_layout.addLayout(zoom_layout)
        
        # Range control
        range_layout = QVBoxLayout()
        range_label = QLabel("Max Range (m):")
        self.range_slider = QSlider(Qt.Horizontal)
        self.range_slider.setMinimum(1)
        self.range_slider.setMaximum(40)
        self.range_slider.setValue(int(self.max_range))
        self.range_slider.valueChanged.connect(self.update_range)
        range_layout.addWidget(range_label)
        range_layout.addWidget(self.range_slider)
        controls_layout.addLayout(range_layout)
        
        # Add checkboxes for display options
        options_layout = QHBoxLayout()
        main_layout.addLayout(options_layout)
        
        # Grid checkbox
        self.grid_checkbox = QCheckBox("Show Grid")
        self.grid_checkbox.setChecked(self.show_grid)
        self.grid_checkbox.stateChanged.connect(self.toggle_grid)
        options_layout.addWidget(self.grid_checkbox)
        
        # Angles checkbox
        self.angles_checkbox = QCheckBox("Show Angles")
        self.angles_checkbox.setChecked(self.show_angles)
        self.angles_checkbox.stateChanged.connect(self.toggle_angles)
        options_layout.addWidget(self.angles_checkbox)
        
        # Filter outliers checkbox
        self.filter_checkbox = QCheckBox("Filter Outliers")
        self.filter_checkbox.setChecked(self.filter_outliers)
        self.filter_checkbox.stateChanged.connect(self.toggle_filter)
        options_layout.addWidget(self.filter_checkbox)
        
        # Reset view button
        self.reset_button = QPushButton("Reset View")
        self.reset_button.clicked.connect(self.reset_view)
        options_layout.addWidget(self.reset_button)
        
        # Status bar for displaying stats
        self.statusBar().showMessage('Waiting for LiDAR data...')
    
    def update_scan_data(self, scan_msg):
        """Update the latest scan data and statistics"""
        self.latest_scan = scan_msg
        
        # Update status bar with scan info
        valid_ranges = [r for r in scan_msg.ranges if not math.isnan(r) and not math.isinf(r)]
        nan_count = sum(1 for r in scan_msg.ranges if math.isnan(r))
        inf_count = sum(1 for r in scan_msg.ranges if math.isinf(r))
        total_count = len(scan_msg.ranges)
        valid_count = len(valid_ranges)
        
        if valid_count > 0:
            avg_range = sum(valid_ranges) / valid_count
            min_range = min(valid_ranges)
            max_range = max(valid_ranges)
            status = f"Points: {total_count} | Valid: {valid_count} | NaN: {nan_count} | Inf: {inf_count} | Avg: {avg_range:.2f}m"
            self.statusBar().showMessage(status)
    
    def update_zoom(self, value):
        """Update zoom factor from slider"""
        self.zoom_factor = float(value)
        self.visualizer.update()
    
    def update_range(self, value):
        """Update max range from slider"""
        self.max_range = float(value)
        self.visualizer.update()
    
    def toggle_grid(self, state):
        """Toggle grid display"""
        self.show_grid = (state == Qt.Checked)
        self.visualizer.update()
    
    def toggle_angles(self, state):
        """Toggle angle markers display"""
        self.show_angles = (state == Qt.Checked)
        self.visualizer.update()
    
    def toggle_filter(self, state):
        """Toggle outlier filtering"""
        self.filter_outliers = (state == Qt.Checked)
        self.visualizer.update()
    
    def reset_view(self):
        """Reset view to default settings"""
        self.zoom_factor = 50.0
        self.zoom_slider.setValue(int(self.zoom_factor))
        
        self.max_range = 10.0
        self.range_slider.setValue(int(self.max_range))
        
        self.show_grid = True
        self.grid_checkbox.setChecked(self.show_grid)
        
        self.show_angles = True
        self.angles_checkbox.setChecked(self.show_angles)
        
        self.filter_outliers = True
        self.filter_checkbox.setChecked(self.filter_outliers)
        
        self.visualizer.update()
    
    def closeEvent(self, event):
        """Handle window close event"""
        self.update_timer.stop()
        event.accept()


class LidarVisualizerWidget(QWidget):
    """
    Widget for drawing the LiDAR visualization
    """
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.setMinimumSize(400, 400)
        
        # Set background color
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(self.backgroundRole(), QColor(20, 20, 30))
        self.setPalette(palette)
    
    def paintEvent(self, event):
        """Paint the visualization"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Get the center of the widget
        center_x = self.width() / 2
        center_y = self.height() / 2
        
        # Draw coordinate grid if enabled
        if self.parent.show_grid:
            self.draw_grid(painter, center_x, center_y)
        
        # Draw angle markers if enabled
        if self.parent.show_angles:
            self.draw_angles(painter, center_x, center_y)
        
        # Draw LiDAR data if available
        if self.parent.latest_scan is not None:
            self.draw_scan(painter, center_x, center_y)
        else:
            # Draw waiting message
            self.draw_waiting_message(painter, center_x, center_y)
        
        # Draw current position marker
        self.draw_position_marker(painter, center_x, center_y)
    
    def draw_waiting_message(self, painter, center_x, center_y):
        """Draw waiting message when no scan data is available"""
        message = "Waiting for LiDAR data..."
        
        font = QFont()
        font.setPointSize(14)
        painter.setFont(font)
        painter.setPen(QColor(255, 200, 0))
        painter.drawText(QRectF(center_x - 150, center_y - 20, 300, 40), Qt.AlignCenter, message)
    
    def draw_grid(self, painter, center_x, center_y):
        """Draw coordinate grid with distance markers"""
        # Set up grid pen
        grid_pen = QPen(QColor(60, 60, 80))
        grid_pen.setWidth(1)
        painter.setPen(grid_pen)
        
        # Draw circular distance markers
        max_range_pixels = self.parent.max_range * self.parent.zoom_factor
        
        # Draw distance circles
        for distance in range(1, int(self.parent.max_range) + 1, 1):
            radius = distance * self.parent.zoom_factor
            if radius <= max_range_pixels:
                painter.drawEllipse(
                    int(center_x - radius),
                    int(center_y - radius),
                    int(radius * 2),
                    int(radius * 2)
                )
                
                # Draw distance label
                font = QFont()
                font.setPointSize(8)
                painter.setFont(font)
                painter.setPen(QColor(120, 120, 140))
                painter.drawText(
                    QPointF(center_x + 5, center_y - radius + 15),
                    f"{distance}m"
                )
        
        # Draw X and Y axes
        painter.setPen(QPen(QColor(70, 70, 90), 1))
        painter.drawLine(int(center_x - max_range_pixels), int(center_y), int(center_x + max_range_pixels), int(center_y))
        painter.drawLine(int(center_x), int(center_y - max_range_pixels), int(center_x), int(center_y + max_range_pixels))
    
    def draw_angles(self, painter, center_x, center_y):
        """Draw angle markers for orientation (mirrored)"""
        radius = min(self.width(), self.height()) * 0.45  # Use 90% of the window size
        
        # Draw angle markers (every 30 degrees)
        painter.setPen(QPen(QColor(80, 80, 100), 1))
        
        # Draw angle lines and labels
        font = QFont()
        font.setPointSize(9)
        painter.setFont(font)
        
        # Using -180 to 180 degree format with mirroring
        for angle_deg in range(-180, 181, 30):
            # Skip 180 and -180 as they're the same line
            if angle_deg == 180:
                continue
                
            # For mirroring, invert the angle
            mirrored_angle_deg = -angle_deg
            mirrored_angle_rad = math.radians(mirrored_angle_deg)
            
            line_length = radius * 1.05
            
            # Calculate end points
            end_x = center_x + math.cos(mirrored_angle_rad) * line_length
            end_y = center_y - math.sin(mirrored_angle_rad) * line_length
            
            # Draw lines
            painter.drawLine(int(center_x), int(center_y), int(end_x), int(end_y))
            
            # Add angle label
            label_x = center_x + math.cos(mirrored_angle_rad) * (line_length + 15)
            label_y = center_y - math.sin(mirrored_angle_rad) * (line_length + 15)
            
            # Adjust label alignment based on position
            alignment = Qt.AlignCenter
            if abs(mirrored_angle_deg - 90) < 10 or abs(mirrored_angle_deg + 90) < 10:
                alignment = Qt.AlignHCenter | Qt.AlignTop
            elif abs(mirrored_angle_deg) < 10 or abs(mirrored_angle_deg + 180) < 10:
                alignment = Qt.AlignVCenter | Qt.AlignLeft
            
            painter.setPen(QColor(150, 150, 180))
            painter.drawText(
                QRectF(label_x - 20, label_y - 10, 40, 20),
                alignment,
                f"{angle_deg}°"  # Keep original angle for labels
            )
    
    def draw_scan(self, painter, center_x, center_y):
        """Draw the LiDAR scan data with horizontal mirroring"""
        scan = self.parent.latest_scan
        if scan is None:
            return
        
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
        
        # Process each scan point
        for i, range_value in enumerate(scan.ranges):
            # Skip invalid readings (inf, nan, etc.)
            if math.isinf(range_value) or math.isnan(range_value):
                continue
                
            # Skip invalid readings if filtering is enabled
            if self.parent.filter_outliers and (range_value < range_min or range_value > range_max):
                continue
            
            # Skip points beyond max display range
            if range_value > self.parent.max_range:
                continue
            
            # Calculate angle for this point (in radians)
            angle = angle_min + (i * angle_increment)
            
            # Convert polar to cartesian coordinates
            # Note: In PyQt, y increases downward, so we negate y
            # Add horizontal mirroring by negating the x coordinate
            x = -range_value * math.cos(angle)  # Negate x to mirror horizontally
            y = range_value * math.sin(angle)
            
            # Scale and translate to screen coordinates
            screen_x = center_x + x * self.parent.zoom_factor
            screen_y = center_y - y * self.parent.zoom_factor
            
            point = QPointF(screen_x, screen_y)
            scan_polygon.append(point)
            scan_points.append((screen_x, screen_y))
            valid_point_count += 1
        
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
                
            # Add info about point count
            painter.setPen(QColor(160, 160, 160))
            painter.drawText(QRectF(10, 70, 300, 20), f"Valid points: {valid_point_count}")
        else:
            # No valid points to display
            message = "No valid LiDAR points (all NaN/Inf)"
            font = QFont()
            font.setPointSize(14)
            painter.setFont(font)
            painter.setPen(QColor(255, 100, 100))
            painter.drawText(QRectF(center_x - 150, center_y - 20, 300, 40), Qt.AlignCenter, message)
    
    def draw_position_marker(self, painter, center_x, center_y):
        """Draw the current position marker (mirrored)"""
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
        
        # Draw legend
        font = QFont()
        font.setPointSize(10)
        painter.setFont(font)
        painter.setPen(QColor(200, 200, 200))
        
        # Draw angle reference (updated for mirrored display)
        painter.drawText(QRectF(10, 90, 400, 20), "0° is left ←, -90° is down ↓, +90° is up ↑ (display mirrored)")
        
        # Draw zoom info
        zoom_text = f"Zoom: {self.parent.zoom_factor:.1f} px/m"
        painter.drawText(QRectF(10, 10, 200, 20), zoom_text)
        
        range_text = f"Range: {self.parent.max_range:.1f} m"
        painter.drawText(QRectF(10, 30, 200, 20), range_text)
        
        # Get scan parameters if available
        if self.parent.latest_scan:
            scan = self.parent.latest_scan
            angle_text = f"Angle range: {math.degrees(scan.angle_min):.1f}° - {math.degrees(scan.angle_max):.1f}°"
            painter.drawText(QRectF(10, 50, 300, 20), angle_text)
        
        # Draw compass labels (for -180 to 180 format, mirrored)
        painter.drawText(QRectF(center_x - 10, 5, 20, 20), "+90°")
        painter.drawText(QRectF(center_x - 15, self.height() - 25, 30, 20), "-90°")
        painter.drawText(QRectF(self.width() - 45, center_y - 10, 40, 20), "±180°")
        painter.drawText(QRectF(5, center_y - 10, 30, 20), "0°")


def main(args=None):
    """Main function to run the LiDAR visualizer"""
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    
    # Create LiDAR node
    lidar_node = LidarVisualizerNode()
    
    # Create main window with reference to the node
    window = LidarVisualizerWindow(lidar_node)
    window.show()
    
    # Setup threading for ROS node
    ros_spin_thread = threading.Thread(target=lambda: rclpy.spin(lidar_node))
    ros_spin_thread.daemon = True
    ros_spin_thread.start()
    
    # Start the application
    try:
        sys.exit(app.exec_())
    finally:
        # Clean up ROS resources
        rclpy.shutdown()


if __name__ == '__main__':
    main()