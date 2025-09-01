#!/usr/bin/env python3

import time
import json
import sys
import os
import paho.mqtt.client as mqtt
import threading
from datetime import datetime

# Fix for importing bno055.py from the imu directory
current_dir = os.path.dirname(os.path.abspath(__file__))
imu_dir = os.path.join(current_dir, 'imu')
sys.path.append(imu_dir)

try:
    from bno055 import BNO055
    print("BNO055 module imported successfully")
except ImportError as e:
    print(f"Error importing BNO055: {e}")
    print(f"Looking for bno055.py in: {imu_dir}")
    print(f"Current directory: {current_dir}")
    sys.exit(1)

# MQTT Configuration - Updated options for better connection
MQTT_BROKER = "0.0.0.0"  # Use localhost for broker on Pi5 itself
MQTT_PORT = 1883
MQTT_TOPIC = "pi5/imu"
MQTT_CLIENT_ID = "pi5_imu_sensor"
MQTT_QOS = 1

# IMU publishing rate (Hz)
IMU_RATE = 10
IMU_PERIOD = 1.0 / IMU_RATE

class IMUPublisher:
    def __init__(self, broker_address, port, topic, client_id):
        """Initialize IMU publisher with MQTT settings"""
        self.broker = broker_address
        self.port = port
        self.topic = topic
        self.client_id = client_id
        
        # Initialize IMU sensor
        self.imu = BNO055()
        self.imu_connected = False
        
        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_connected = False
        
        # Threading control
        self.running = False
        self.publish_thread = None
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback for when the MQTT client connects to the broker"""
        if rc == 0:
            print(f"Connected to MQTT broker at {self.broker}:{self.port}")
            self.mqtt_connected = True
        else:
            error_messages = {
                1: "Connection refused - incorrect protocol version",
                2: "Connection refused - invalid client identifier",
                3: "Connection refused - server unavailable",
                4: "Connection refused - bad username or password",
                5: "Connection refused - not authorized"
            }
            error_msg = error_messages.get(rc, f"Unknown error code: {rc}")
            print(f"Failed to connect to MQTT broker: {error_msg}")
            self.mqtt_connected = False
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback for when the MQTT client disconnects from the broker"""
        if rc == 0:
            print("Disconnected from MQTT broker cleanly")
        else:
            print(f"Unexpected disconnection from MQTT broker with code: {rc}")
        self.mqtt_connected = False
        
        # Try to reconnect if not shutting down
        if self.running:
            print("Attempting to reconnect...")
            try:
                self.mqtt_client.reconnect()
            except:
                print("Reconnection failed. Will retry later.")
    
    def connect_imu(self):
        """Connect to the BNO055 IMU sensor"""
        try:
            print("Connecting to BNO055 IMU sensor...")
            if self.imu.begin():
                self.imu_connected = True
                print("Connected to BNO055 IMU sensor")
                return True
            else:
                print("Failed to initialize BNO055 IMU sensor")
                return False
        except Exception as e:
            print(f"Error connecting to IMU: {e}")
            return False
    
    def connect_mqtt(self):
        """Connect to the MQTT broker"""
        try:
            print(f"Connecting to MQTT broker at {self.broker}:{self.port}...")
            
            # First check if the broker is reachable
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2)
            try:
                print(f"Testing connection to {self.broker}:{self.port}...")
                s.connect((self.broker, self.port))
                s.close()
                print("MQTT broker is reachable")
            except Exception as e:
                print(f"Cannot reach MQTT broker: {e}")
                print("Is the broker running? Is the IP address correct?")
                print("Is the port 1883 open and not blocked by firewall?")
                return False
            
            # Set connection timeout
            self.mqtt_client.connect_timeout_secs = 10
            
            # For local connections, use a very short keepalive
            self.mqtt_client.connect(self.broker, self.port, keepalive=15)
            
            # Start the MQTT loop in the background
            self.mqtt_client.loop_start()
            
            # Wait for the connection to be established
            timeout = time.time() + 5  # 5-second timeout
            while not self.mqtt_connected and time.time() < timeout:
                time.sleep(0.1)
                
            return self.mqtt_connected
        except Exception as e:
            print(f"Error connecting to MQTT broker: {e}")
            return False
    
    def publish_imu_data(self):
        """Continuously read IMU data and publish to MQTT"""
        last_status_print = 0
        
        while self.running:
            current_time = time.time()
            
            # Check connections
            if not self.imu_connected:
                print("IMU not connected, attempting to reconnect...")
                self.imu_connected = self.connect_imu()
                time.sleep(1)
                continue
                
            if not self.mqtt_connected:
                print("MQTT not connected, attempting to reconnect...")
                try:
                    self.mqtt_client.reconnect()
                    time.sleep(1)
                except Exception as e:
                    print(f"Reconnection failed: {e}")
                    time.sleep(5)  # Wait longer between reconnect attempts
                continue
            
            try:
                # Get current timestamp
                timestamp = datetime.now().isoformat()
                
                # Read sensor data
                quaternion = self.imu.get_quaternion()
                gyro = self.imu.get_gyroscope()
                accel = self.imu.get_linear_acceleration()
                euler = self.imu.get_euler()
                calib = self.imu.get_calibration_status()
                
                # Create JSON payload
                payload = {
                    "timestamp": timestamp,
                    "quaternion": {
                        "w": quaternion[0],
                        "x": quaternion[1],
                        "y": quaternion[2],
                        "z": quaternion[3]
                    },
                    "euler": {
                        "roll": euler[0],
                        "pitch": euler[1],
                        "yaw": euler[2]
                    },
                    "angular_velocity": {
                        "x": gyro[0],
                        "y": gyro[1],
                        "z": gyro[2]
                    },
                    "linear_acceleration": {
                        "x": accel[0],
                        "y": accel[1],
                        "z": accel[2]
                    },
                    "calibration": {
                        "system": calib[0],
                        "gyro": calib[1],
                        "accel": calib[2],
                        "mag": calib[3]
                    }
                }
                
                # Convert to JSON string
                json_payload = json.dumps(payload)
                
                # Publish to MQTT topic
                result = self.mqtt_client.publish(self.topic, json_payload, qos=MQTT_QOS)
                
                # Check if publish was successful
                if result.rc != mqtt.MQTT_ERR_SUCCESS:
                    print(f"Failed to publish: {mqtt.error_string(result.rc)}")
                elif current_time - last_status_print >= 5:  # Every 5 seconds
                    print(f"Publishing data to {self.topic} (message #{result.mid})")
                    last_status_print = current_time
                
                # Print calibration status occasionally
                if current_time - last_status_print >= 5:  # Every 5 seconds
                    print(f"Calibration: sys={calib[0]}/3, gyro={calib[1]}/3, accel={calib[2]}/3, mag={calib[3]}/3")
                    last_status_print = current_time
                
                # Sleep to maintain desired update rate
                time.sleep(IMU_PERIOD)
                
            except Exception as e:
                print(f"Error reading or publishing IMU data: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(1)  # Wait before retrying
    
    def start(self):
        """Start the IMU publisher"""
        if self.running:
            print("IMU publisher is already running")
            return False
        
        # Connect to IMU
        if not self.connect_imu():
            print("Failed to connect to IMU. Cannot start.")
            return False
        
        # Connect to MQTT broker
        if not self.connect_mqtt():
            print("Failed to connect to MQTT broker. Cannot start.")
            print("However, we'll continue with IMU readings for local use.")
            # Continue anyway - we'll try to reconnect in the loop
            
        # Set running flag and start thread
        self.running = True
        self.publish_thread = threading.Thread(target=self.publish_imu_data)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
        print(f"IMU publisher started. Publishing to {self.topic} at {IMU_RATE}Hz")
        return True
    
    def stop(self):
        """Stop the IMU publisher"""
        if not self.running:
            print("IMU publisher is not running")
            return
        
        # Set flag to stop thread
        self.running = False
        
        # Wait for thread to finish
        if self.publish_thread:
            self.publish_thread.join(timeout=2)
        
        # Stop MQTT client
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        print("IMU publisher stopped")

def main():
    """Main function"""
    print("BNO055 IMU MQTT Publisher for Raspberry Pi 5")
    print("===========================================")
    
    # Get broker address from command line if provided
    broker = MQTT_BROKER
    if len(sys.argv) > 1:
        broker = sys.argv[1]
        print(f"Using broker address: {broker}")
    
    # Create IMU publisher
    imu_publisher = IMUPublisher(
        broker_address=broker,
        port=MQTT_PORT,
        topic=MQTT_TOPIC,
        client_id=MQTT_CLIENT_ID
    )
    
    # Start IMU publisher
    if not imu_publisher.start():
        print("Failed to start IMU publisher. Exiting.")
        sys.exit(1)
    
    try:
        # Keep main thread alive
        print("Press Ctrl+C to exit")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping IMU publisher...")
        imu_publisher.stop()
        print("Exiting")
        sys.exit(0)

if __name__ == "__main__":
    main()