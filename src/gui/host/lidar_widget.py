#!/usr/bin/env python3

import random
import math
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QGroupBox
from PyQt5.QtGui import QPixmap, QPainter, QColor, QPen, QFont, QBrush
from PyQt5.QtCore import Qt, QTimer

class LidarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
        
        # Data storage for LiDAR points
        self.lidar_data = None
        
        # Status flag
        self.waiting_for_data = True
        
        # Start timer for demo updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_visualization)
        self.timer.start(100)  # Update every 100ms
    
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
        
        # Status label
        self.status_label = QLabel("Waiting for LiDAR data...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setFont(QFont("Monospace", 10))
        self.status_label.setStyleSheet("color: yellow;")
        
        group_layout.addWidget(self.lidar_label)
        group_layout.addWidget(self.status_label)
        group_box.setLayout(group_layout)
        layout.addWidget(group_box)
    
    def update_visualization(self):
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
        
        # Draw circular grid
        painter.setPen(QPen(QColor(40, 40, 40)))
        max_radius = min(center_x, center_y) - 20
        
        # Draw concentric circles
        for r in range(0, max_radius, max_radius // 5):
            painter.drawEllipse(center_x - r, center_y - r, r * 2, r * 2)
            
            # Draw distance labels
            if r > 0:
                # Calculate distance in meters (assuming max radius is 10m)
                distance = r * 10 / max_radius
                distance_text = f"{distance:.1f}m"
                
                # Draw text on the bottom-right of the circle
                text_x = center_x + r * 0.7
                text_y = center_y + 15
                painter.setPen(QPen(QColor(60, 60, 60)))
                painter.drawText(text_x, text_y, distance_text)
                
                # Reset pen
                painter.setPen(QPen(QColor(40, 40, 40)))
        
        # Draw axis lines
        painter.drawLine(center_x, 0, center_x, height)
        painter.drawLine(0, center_y, width, center_y)
        
        # Draw cardinal directions
        painter.setPen(QPen(QColor(70, 70, 70)))
        painter.drawText(center_x + 5, 15, "0°")            # Top (0°)
        painter.drawText(width - 25, center_y - 5, "90°")   # Right (90°)
        painter.drawText(center_x + 5, height - 5, "180°")  # Bottom (180°)
        painter.drawText(10, center_y - 5, "270°")          # Left (270°)
        
        if self.waiting_for_data:
            # Draw "waiting for data" indicator
            painter.setPen(QPen(QColor(255, 255, 0), 2))
            painter.drawText(center_x - 100, center_y, "Waiting for LiDAR data...")
            
            # Draw spinning wait indicator
            angle = (math.pi * (time.time() % 2)) / 2
            spin_radius = max_radius // 3
            end_x = center_x + int(spin_radius * math.cos(angle))
            end_y = center_y + int(spin_radius * math.sin(angle))
            
            painter.setPen(QPen(QColor(255, 255, 0), 3))
            painter.drawLine(center_x, center_y, end_x, end_y)
        else:
            # Draw simulated LiDAR points
            painter.setPen(QPen(QColor(0, 255, 0), 2))
            
            # Generate random points for demo
            num_points = 360  # One point per degree
            for i in range(num_points):
                angle = math.radians(i)
                
                # Randomize distance for demo
                distance = max_radius * (0.3 + 0.7 * random.random())
                
                # Add noise to make it look more natural
                if random.random() < 0.1:  # 10% chance for a "missing" point
                    continue
                    
                # Calculate x, y (polar to cartesian)
                x = center_x + distance * math.cos(angle)
                y = center_y + distance * math.sin(angle)
                
                # Draw point
                painter.drawPoint(int(x), int(y))
        
        # Finish painting
        painter.end()
        
        # Update label
        self.lidar_label.setPixmap(self.pixmap)
        
    def set_lidar_data(self, data):
        """Set LiDAR data from ROS topic"""
        self.lidar_data = data
        self.waiting_for_data = False
        self.status_label.setText("LiDAR data received")
        self.status_label.setStyleSheet("color: lime;")

import time  # Add time import for the wait indicator animation