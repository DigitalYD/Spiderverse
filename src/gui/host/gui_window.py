#!/usr/bin/env python3

from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QSplitter
from PyQt5.QtCore import Qt

from gst_widget import GStreamerWidget
from imu_widget import IMUWidget
from lidar_widget import LidarWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pi5 Dashboard")
        self.resize(1200, 800)
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Create main vertical layout
        main_layout = QVBoxLayout(central_widget)
        
        # Top panel - IMU data (full width)
        self.imu_widget = IMUWidget()
        main_layout.addWidget(self.imu_widget)
        
        # Bottom panel - Splitter for video and LiDAR
        bottom_splitter = QSplitter(Qt.Horizontal)
        
        # Left side - Video stream
        self.video_widget = GStreamerWidget()
        bottom_splitter.addWidget(self.video_widget)
        
        # Right side - LiDAR data
        self.lidar_widget = LidarWidget()
        bottom_splitter.addWidget(self.lidar_widget)
        
        # Set initial sizes for the bottom splitter (60% video, 40% LiDAR)
        bottom_splitter.setSizes([600, 400])
        
        # Add bottom splitter to main layout
        main_layout.addWidget(bottom_splitter)
        
        # Set the stretch factor to make bottom panel larger than top panel
        main_layout.setStretchFactor(bottom_splitter, 4)  # Bottom gets 4x space of top
        main_layout.setStretchFactor(self.imu_widget, 1)  # Top gets 1 unit of space
        
        # Status bar
        self.statusBar().showMessage("Ready")
    
    def closeEvent(self, event):
        # Stop video stream
        self.video_widget.stop_stream()
        event.accept()