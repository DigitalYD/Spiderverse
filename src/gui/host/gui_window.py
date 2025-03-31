#!/usr/bin/env python3

from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QSplitter
from PyQt5.QtCore import Qt

from gst_widget import GStreamerWidget
from mqtt_widget import PiStatsWidget, IMUWidget
from lidar_widget import LidarWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pi5 Dashboard")
        self.resize(1200, 800)
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Create main layout with splitter
        main_layout = QHBoxLayout(central_widget)
        splitter = QSplitter(Qt.Horizontal)
        
        # Left panel - Video stream
        self.video_widget = GStreamerWidget()
        splitter.addWidget(self.video_widget)
        
        # Middle panel - Pi stats and IMU data
        middle_panel = QWidget()
        middle_layout = QVBoxLayout(middle_panel)
        
        # Pi stats widget
        self.pi_stats_widget = PiStatsWidget()
        middle_layout.addWidget(self.pi_stats_widget)
        
        # IMU widget
        self.imu_widget = IMUWidget()
        middle_layout.addWidget(self.imu_widget)
        
        splitter.addWidget(middle_panel)
        
        # Right panel - LiDAR data
        self.lidar_widget = LidarWidget()
        splitter.addWidget(self.lidar_widget)
        
        # Set initial sizes
        splitter.setSizes([500, 350, 350])
        
        main_layout.addWidget(splitter)
        
        # Status bar
        self.statusBar().showMessage("Ready")
    
    def closeEvent(self, event):
        # Stop video stream
        self.video_widget.stop_stream()
        event.accept()
