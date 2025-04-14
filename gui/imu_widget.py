#!/usr/bin/env python3

import json
import threading
import time
import math
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QGroupBox, QHBoxLayout, QPushButton
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject

# Try to import paho.mqtt if available
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False

class IMUSignals(QObject):
    """Class to handle signals from IMU thread to Qt UI thread"""
    data_received = pyqtSignal(dict)

class IMUWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Set up signals for thread-safe updates
        self.signals = IMUSignals()
        self.signals.data_received.connect(self.update_with_imu_data)
        
        # MQTT client and connection parameters
        self.mqtt_client = None
        self.mqtt_connected = False
        self.broker_address = "192.168.0.49"  # Default Pi address
        self.port = 1883
        self.topic = "pi5/imu"
        
        # Setup UI
        self.setup_ui()
        
        # Connect to MQTT if available
        if MQTT_AVAILABLE:
            self.setup_mqtt()
        else:
            self.status_label.setText("MQTT library not available")
            self.status_label.setStyleSheet("color: red;")
    
    def setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Group box
        self.group_box = QGroupBox("IMU Data")
        self.group_box.setStyleSheet("QGroupBox { font-weight: bold; }")
        
        # Use horizontal layout for the bar
        group_layout = QHBoxLayout()
        group_layout.setSpacing(10)
        self.group_box.setLayout(group_layout)
        
        # Create sections for different IMU data types
        # Orientation section
        orientation_widget = QWidget()
        orientation_layout = QVBoxLayout(orientation_widget)
        orientation_layout.setContentsMargins(5, 5, 5, 5)
        
        orientation_title = QLabel("Orientation (Quaternion)")
        orientation_title.setFont(QFont("Monospace", 9, QFont.Bold))
        orientation_title.setAlignment(Qt.AlignCenter)
        
        self.orientation_label = QLabel("x: -.----, y: -.----, z: -.----, w: -.----")
        self.orientation_label.setFont(QFont("Monospace", 9))
        self.orientation_label.setAlignment(Qt.AlignCenter)
        
        orientation_layout.addWidget(orientation_title)
        orientation_layout.addWidget(self.orientation_label)
        
        # Euler angles section
        euler_widget = QWidget()
        euler_layout = QVBoxLayout(euler_widget)
        euler_layout.setContentsMargins(5, 5, 5, 5)
        
        euler_title = QLabel("Euler Angles (deg)")
        euler_title.setFont(QFont("Monospace", 9, QFont.Bold))
        euler_title.setAlignment(Qt.AlignCenter)
        
        self.euler_label = QLabel("roll: --.--°, pitch: --.--°, yaw: --.--°")
        self.euler_label.setFont(QFont("Monospace", 9))
        self.euler_label.setAlignment(Qt.AlignCenter)
        
        euler_layout.addWidget(euler_title)
        euler_layout.addWidget(self.euler_label)
        
        # Angular velocity section
        angular_widget = QWidget()
        angular_layout = QVBoxLayout(angular_widget)
        angular_layout.setContentsMargins(5, 5, 5, 5)
        
        angular_title = QLabel("Angular Velocity (deg/s)")
        angular_title.setFont(QFont("Monospace", 9, QFont.Bold))
        angular_title.setAlignment(Qt.AlignCenter)
        
        self.angular_vel_label = QLabel("x: -.----, y: -.----, z: -.----")
        self.angular_vel_label.setFont(QFont("Monospace", 9))
        self.angular_vel_label.setAlignment(Qt.AlignCenter)
        
        angular_layout.addWidget(angular_title)
        angular_layout.addWidget(self.angular_vel_label)
        
        # Linear acceleration section
        accel_widget = QWidget()
        accel_layout = QVBoxLayout(accel_widget)
        accel_layout.setContentsMargins(5, 5, 5, 5)
        
        accel_title = QLabel("Linear Acceleration (m/s²)")
        accel_title.setFont(QFont("Monospace", 9, QFont.Bold))
        accel_title.setAlignment(Qt.AlignCenter)
        
        self.linear_accel_label = QLabel("x: -.----, y: -.----, z: -.----")
        self.linear_accel_label.setFont(QFont("Monospace", 9))
        self.linear_accel_label.setAlignment(Qt.AlignCenter)
        
        accel_layout.addWidget(accel_title)
        accel_layout.addWidget(self.linear_accel_label)
        
        # Status section
        status_widget = QWidget()
        status_layout = QVBoxLayout(status_widget)
        status_layout.setContentsMargins(5, 5, 5, 5)
        
        status_title = QLabel("IMU Status")
        status_title.setFont(QFont("Monospace", 9, QFont.Bold))
        status_title.setAlignment(Qt.AlignCenter)
        
        self.status_label = QLabel("Disconnected")
        self.status_label.setFont(QFont("Monospace", 9))
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: red;")
        
        # Calibration status
        self.calib_label = QLabel("Cal: -/-/-/-")
        self.calib_label.setFont(QFont("Monospace", 9))
        self.calib_label.setAlignment(Qt.AlignCenter)
        self.calib_label.setStyleSheet("color: gray;")
        
        # Connect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        
        status_layout.addWidget(status_title)
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.calib_label)
        status_layout.addWidget(self.connect_button)
        
        # Add all sections to the group layout
        group_layout.addWidget(orientation_widget)
        group_layout.addWidget(euler_widget)
        group_layout.addWidget(angular_widget)
        group_layout.addWidget(accel_widget)
        group_layout.addWidget(status_widget)
        
        # Add the group box to the main layout
        layout.addWidget(self.group_box)
    
    def setup_mqtt(self):
        """Set up MQTT client"""
        try:
            # Create MQTT client
            self.mqtt_client = mqtt.Client()
            
            # Set callbacks
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            # Set status
            self.status_label.setText("Ready to connect")
            self.status_label.setStyleSheet("color: orange;")
            
        except Exception as e:
            print(f"Error setting up MQTT client: {e}")
            self.status_label.setText(f"MQTT Error")
            self.status_label.setStyleSheet("color: red;")
    
    def toggle_connection(self):
        """Toggle MQTT connection"""
        if not MQTT_AVAILABLE:
            self.status_label.setText("MQTT library not available")
            self.status_label.setStyleSheet("color: red;")
            return
        
        if self.mqtt_connected:
            # Disconnect
            try:
                self.mqtt_client.disconnect()
                self.mqtt_client.loop_stop()
                self.mqtt_connected = False
                self.connect_button.setText("Connect")
                self.status_label.setText("Disconnected")
                self.status_label.setStyleSheet("color: red;")
                
                # Reset data displays
                self.orientation_label.setText("x: -.----, y: -.----, z: -.----, w: -.----")
                self.euler_label.setText("roll: --.--°, pitch: --.--°, yaw: --.--°")
                self.angular_vel_label.setText("x: -.----, y: -.----, z: -.----")
                self.linear_accel_label.setText("x: -.----, y: -.----, z: -.----")
                self.calib_label.setText("Cal: -/-/-/-")
                self.calib_label.setStyleSheet("color: gray;")
                
            except Exception as e:
                print(f"Error disconnecting: {e}")
        else:
            # Connect
            try:
                self.status_label.setText("Connecting...")
                self.status_label.setStyleSheet("color: yellow;")
                
                # Try to connect
                self.mqtt_client.connect_async(self.broker_address, self.port, 60)
                self.mqtt_client.loop_start()
                
                # Update button
                self.connect_button.setText("Disconnect")
                
            except Exception as e:
                print(f"Error connecting to MQTT: {e}")
                self.status_label.setText(f"Connection Error")
                self.status_label.setStyleSheet("color: red;")
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback for MQTT connection"""
        if rc == 0:
            # Successfully connected
            self.mqtt_connected = True
            self.status_label.setText("Connected to MQTT")
            self.status_label.setStyleSheet("color: green;")
            
            # Subscribe to IMU topic
            client.subscribe(self.topic)
            print(f"Subscribed to {self.topic}")
        else:
            # Connection failed
            self.mqtt_connected = False
            self.status_label.setText(f"Connection Failed ({rc})")
            self.status_label.setStyleSheet("color: red;")
            self.connect_button.setText("Connect")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback for MQTT disconnection"""
        self.mqtt_connected = False
        
        if rc == 0:
            # Clean disconnect
            self.status_label.setText("Disconnected")
        else:
            # Unexpected disconnect
            self.status_label.setText(f"Connection Lost ({rc})")
        
        self.status_label.setStyleSheet("color: red;")
        self.connect_button.setText("Connect")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Callback for MQTT message received"""
        try:
            # Parse JSON payload
            payload = json.loads(msg.payload.decode('utf-8'))
            
            # Add status key
            payload["status"] = "data"
            
            # Send to UI thread
            self.signals.data_received.emit(payload)
            
        except Exception as e:
            print(f"Error processing message: {e}")
    
    def update_with_imu_data(self, data):
        """Update UI with real IMU data (called from the UI thread via signal)"""
        status = data.get("status")
        
        if status == "data":
            # We received actual data, update the UI
            
            # Update quaternion
            quat = data.get("quaternion", {})
            self.orientation_label.setText(
                f"x: {quat.get('x', 0):.4f}, y: {quat.get('y', 0):.4f}, "
                f"z: {quat.get('z', 0):.4f}, w: {quat.get('w', 0):.4f}"
            )
            
            # Update euler angles
            euler = data.get("euler", {})
            self.euler_label.setText(
                f"roll: {euler.get('roll', 0):.2f}°, pitch: {euler.get('pitch', 0):.2f}°, "
                f"yaw: {euler.get('yaw', 0):.2f}°"
            )
            
            # Update angular velocity
            gyro = data.get("angular_velocity", {})
            self.angular_vel_label.setText(
                f"x: {gyro.get('x', 0):.4f}, y: {gyro.get('y', 0):.4f}, z: {gyro.get('z', 0):.4f}"
            )
            
            # Update linear acceleration
            accel = data.get("linear_acceleration", {})
            self.linear_accel_label.setText(
                f"x: {accel.get('x', 0):.4f}, y: {accel.get('y', 0):.4f}, z: {accel.get('z', 0):.4f}"
            )
            
            # Update calibration status
            calib = data.get("calibration", {})
            self.calib_label.setText(
                f"Cal: {calib.get('system', 0)}/{calib.get('gyro', 0)}/"
                f"{calib.get('accel', 0)}/{calib.get('mag', 0)}"
            )
            
            # Set calibration status color
            sys_cal = calib.get('system', 0)
            if sys_cal == 3:
                self.calib_label.setStyleSheet("color: lime;")
            elif sys_cal >= 2:
                self.calib_label.setStyleSheet("color: yellow;")
            else:
                self.calib_label.setStyleSheet("color: orange;")