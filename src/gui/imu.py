#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json
import time
import threading
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, QGroupBox, QHBoxLayout
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QFont

# MQTT Configuration
MQTT_BROKER = "localhost"  # Use localhost if MQTT broker is running on your laptop
MQTT_PORT = 1883
MQTT_TOPIC = "pi5/imu"
MQTT_CLIENT_ID = "laptop_imu_receiver"

class IMUDataSignals(QObject):
    """Class to handle signals from MQTT thread to Qt UI thread"""
    data_received = pyqtSignal(dict)

class IMUDataWidget(QWidget):
    """Widget to display IMU data received from MQTT"""
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Create signals for thread-safe updates
        self.signals = IMUDataSignals()
        self.signals.data_received.connect(self.update_display)
        
        # Set up the UI
        self.setup_ui()
        
        # Start MQTT client in a separate thread
        self.mqtt_thread = threading.Thread(target=self.run_mqtt_client)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()
    
    def setup_ui(self):
        """Set up the UI to display IMU data"""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Group box
        self.group_box = QGroupBox("IMU Data (MQTT)")
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
        
        self.orientation_label = QLabel("x: 0.000, y: 0.000, z: 0.000, w: 0.000")
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
        
        self.euler_label = QLabel("roll: 0.00, pitch: 0.00, yaw: 0.00")
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
        
        self.angular_vel_label = QLabel("x: 0.000, y: 0.000, z: 0.000")
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
        
        self.linear_accel_label = QLabel("x: 0.000, y: 0.000, z: 0.000")
        self.linear_accel_label.setFont(QFont("Monospace", 9))
        self.linear_accel_label.setAlignment(Qt.AlignCenter)
        
        accel_layout.addWidget(accel_title)
        accel_layout.addWidget(self.linear_accel_label)
        
        # Status section
        status_widget = QWidget()
        status_layout = QVBoxLayout(status_widget)
        status_layout.setContentsMargins(5, 5, 5, 5)
        
        status_title = QLabel("MQTT Status")
        status_title.setFont(QFont("Monospace", 9, QFont.Bold))
        status_title.setAlignment(Qt.AlignCenter)
        
        self.status_label = QLabel("Connecting to MQTT broker...")
        self.status_label.setFont(QFont("Monospace", 9))
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: yellow;")
        
        # Calibration status
        self.calib_label = QLabel("Cal: 0/0/0/0")
        self.calib_label.setFont(QFont("Monospace", 9))
        self.calib_label.setAlignment(Qt.AlignCenter)
        
        # Last update time
        self.time_label = QLabel("Last update: Never")
        self.time_label.setFont(QFont("Monospace", 9))
        self.time_label.setAlignment(Qt.AlignCenter)
        
        status_layout.addWidget(status_title)
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(self.calib_label)
        status_layout.addWidget(self.time_label)
        
        # Add all sections to the group layout
        group_layout.addWidget(orientation_widget)
        group_layout.addWidget(euler_widget)
        group_layout.addWidget(angular_widget)
        group_layout.addWidget(accel_widget)
        group_layout.addWidget(status_widget)
        
        # Add the group box to the main layout
        layout.addWidget(self.group_box)
    
    def run_mqtt_client(self):
        """Run MQTT client in a separate thread"""
        # Create MQTT client
        client = mqtt.Client(client_id=MQTT_CLIENT_ID)
        
        # Set up callbacks
        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.on_disconnect = self.on_disconnect
        
        # Try to connect to MQTT broker
        try:
            client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
            
            # Main MQTT loop
            client.loop_forever()
        except Exception as e:
            print(f"Error connecting to MQTT broker: {e}")
            self.signals.data_received.emit({"status": "error", "message": str(e)})
    
    def on_connect(self, client, userdata, flags, rc):
        """Callback for when the client connects to the MQTT broker"""
        if rc == 0:
            print(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
            self.signals.data_received.emit({"status": "connected"})
            
            # Subscribe to IMU topic
            client.subscribe(MQTT_TOPIC)
            print(f"Subscribed to topic: {MQTT_TOPIC}")
        else:
            print(f"Failed to connect to MQTT broker, return code: {rc}")
            self.signals.data_received.emit({"status": "connection_failed", "code": rc})
    
    def on_message(self, client, userdata, msg):
        """Callback for when a message is received from the MQTT broker"""
        try:
            # Parse JSON payload
            payload = json.loads(msg.payload.decode('utf-8'))
            
            # Add status to indicate this is data
            payload["status"] = "data"
            
            # Add timestamp for internal tracking
            payload["received_time"] = time.time()
            
            # Emit signal with data
            self.signals.data_received.emit(payload)
        except Exception as e:
            print(f"Error processing MQTT message: {e}")
            print(f"Raw message: {msg.payload}")
    
    def on_disconnect(self, client, userdata, rc):
        """Callback for when the client disconnects from the MQTT broker"""
        if rc != 0:
            print(f"Unexpected disconnection from MQTT broker, code: {rc}")
            self.signals.data_received.emit({"status": "disconnected", "code": rc})
    
    def update_display(self, data):
        """Update the UI with IMU data (called from UI thread via signal)"""
        status = data.get("status")
        
        if status == "connected":
            self.status_label.setText("Connected to MQTT broker")
            self.status_label.setStyleSheet("color: lime;")
        elif status == "connection_failed":
            code = data.get("code", "unknown")
            self.status_label.setText(f"Connection failed (code: {code})")
            self.status_label.setStyleSheet("color: red;")
        elif status == "disconnected":
            code = data.get("code", "unknown")
            self.status_label.setText(f"Disconnected (code: {code})")
            self.status_label.setStyleSheet("color: orange;")
        elif status == "error":
            message = data.get("message", "Unknown error")
            self.status_label.setText(f"Error: {message[:20]}")
            self.status_label.setStyleSheet("color: red;")
        elif status == "data":
            # Update status
            self.status_label.setText("Receiving data")
            self.status_label.setStyleSheet("color: lime;")
            
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
            
            # Update time label
            current_time = time.strftime("%H:%M:%S", time.localtime())
            self.time_label.setText(f"Last update: {current_time}")

class MainWindow(QMainWindow):
    """Main application window"""
    def __init__(self):
        super().__init__()
        
        # Set up the window
        self.setWindowTitle("IMU MQTT Receiver")
        self.setGeometry(100, 100, 800, 200)
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Create layout
        layout = QVBoxLayout(central_widget)
        
        # Create IMU data widget
        self.imu_widget = IMUDataWidget()
        layout.addWidget(self.imu_widget)

def main():
    """Main function"""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()