#!/usr/bin/env python3

import json
import threading
import time
import random
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QGroupBox, QGridLayout
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer

class MQTTWidget(QWidget):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.title = title
        self.setup_ui()
        
        # Start a timer to update with demo data
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_with_demo_data)
        self.timer.start(1000)  # Update every second
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Group box
        self.group_box = QGroupBox(self.title)
        self.group_layout = QGridLayout()
        self.group_box.setLayout(self.group_layout)
        
        layout.addWidget(self.group_box)
    
    def update_with_demo_data(self):
        # To be implemented by subclasses
        pass

class PiStatsWidget(MQTTWidget):
    def __init__(self, parent=None):
        super().__init__("Raspberry Pi 5 Stats", parent)
        
        # Set up labels
        self.cpu_label = QLabel("CPU: 0.0%")
        self.ram_label = QLabel("RAM: 0.0%")
        self.temp_label = QLabel("Temperature: 0.0°C")
        self.storage_label = QLabel("Storage: 0.0%")
        self.net_rx_label = QLabel("Network RX: 0.0 KB/s")
        self.net_tx_label = QLabel("Network TX: 0.0 KB/s")
        
        # Add labels to grid
        self.group_layout.addWidget(self.cpu_label, 0, 0)
        self.group_layout.addWidget(self.ram_label, 0, 1)
        self.group_layout.addWidget(self.temp_label, 1, 0)
        self.group_layout.addWidget(self.storage_label, 1, 1)
        self.group_layout.addWidget(self.net_rx_label, 2, 0)
        self.group_layout.addWidget(self.net_tx_label, 2, 1)
    
    def update_with_demo_data(self):
        # Generate random stats for demonstration
        cpu = random.uniform(0, 100)
        ram = random.uniform(0, 100)
        temp = random.uniform(40, 70)
        storage = random.uniform(0, 100)
        net_rx = random.uniform(0, 1000)
        net_tx = random.uniform(0, 500)
        
        # Update UI
        self.cpu_label.setText(f"CPU: {cpu:.1f}%")
        self.ram_label.setText(f"RAM: {ram:.1f}%")
        self.temp_label.setText(f"Temperature: {temp:.1f}°C")
        self.storage_label.setText(f"Storage: {storage:.1f}%")
        self.net_rx_label.setText(f"Network RX: {net_rx:.1f} KB/s")
        self.net_tx_label.setText(f"Network TX: {net_tx:.1f} KB/s")

class IMUWidget(MQTTWidget):
    def __init__(self, parent=None):
        super().__init__("IMU Data", parent)
        
        # Set up labels
        self.orientation_label = QLabel("Orientation (x, y, z, w):\n0.00, 0.00, 0.00, 0.00")
        self.orientation_label.setFont(QFont("Monospace", 10))
        
        self.angular_vel_label = QLabel("Angular Velocity (x, y, z):\n0.00, 0.00, 0.00")
        self.angular_vel_label.setFont(QFont("Monospace", 10))
        
        self.linear_accel_label = QLabel("Linear Acceleration (x, y, z):\n0.00, 0.00, 0.00")
        self.linear_accel_label.setFont(QFont("Monospace", 10))
        
        # Add labels to grid
        self.group_layout.addWidget(self.orientation_label, 0, 0)
        self.group_layout.addWidget(self.angular_vel_label, 1, 0)
        self.group_layout.addWidget(self.linear_accel_label, 2, 0)
    
    def update_with_demo_data(self):
        # Generate random IMU data
        import math
        
        # Create simple rotating orientation
        angle = (time.time() % 6.28)  # Full circle every ~6 seconds
        
        # Orientation (quaternion)
        qx = math.sin(angle) * 0.1
        qy = math.cos(angle) * 0.1
        qz = 0.0
        qw = math.sqrt(1 - 0.02)  # Ensure quaternion has magnitude 1
        
        # Angular velocity
        wx = math.sin(angle * 2) * 0.5
        wy = math.cos(angle * 2) * 0.5
        wz = math.sin(angle * 3) * 0.5
        
        # Linear acceleration
        ax = random.gauss(0, 0.1)
        ay = random.gauss(0, 0.1)
        az = 9.8 + random.gauss(0, 0.1)  # ~9.8 m/s² with noise
        
        # Update UI
        self.orientation_label.setText(f"Orientation (x, y, z, w):\n{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}")
        self.angular_vel_label.setText(f"Angular Velocity (x, y, z):\n{wx:.4f}, {wy:.4f}, {wz:.4f}")
        self.linear_accel_label.setText(f"Linear Acceleration (x, y, z):\n{ax:.4f}, {ay:.4f}, {az:.4f}")
