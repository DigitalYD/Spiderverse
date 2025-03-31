#!/usr/bin/env python3

import random
import math
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QGroupBox
from PyQt5.QtGui import QPixmap, QPainter, QColor, QPen
from PyQt5.QtCore import Qt, QTimer

class LidarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
        
        # Start timer for demo updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_visualization)
        self.timer.start(100)  # Update every 100ms
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Group box
        group_box = QGroupBox("LiDAR Data")
        group_layout = QVBoxLayout()
        
        # LiDAR visualization
        self.lidar_label = QLabel()
        self.lidar_label.setMinimumSize(300, 300)
        self.lidar_label.setAlignment(Qt.AlignCenter)
        
        # Create pixmap for drawing
        self.pixmap = QPixmap(300, 300)
        self.pixmap.fill(Qt.black)
        self.lidar_label.setPixmap(self.pixmap)
        
        group_layout.addWidget(self.lidar_label)
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
        max_radius = min(center_x, center_y) - 10
        
        # Draw concentric circles
        for r in range(0, max_radius, max_radius // 4):
            painter.drawEllipse(center_x - r, center_y - r, r * 2, r * 2)
        
        # Draw axis lines
        painter.drawLine(center_x, 0, center_x, height)
        painter.drawLine(0, center_y, width, center_y)
        
        # Draw simulated LiDAR points
        painter.setPen(QColor(0, 255, 0))
        
        # Generate random points for demo
        num_points = 360
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
