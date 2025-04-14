#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import threading
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

# Default MQTT configuration
DEFAULT_BROKER_ADDRESS = "192.168.0.115"
DEFAULT_PORT = 1883
DEFAULT_TOPIC = "pi5/imu"

class IMUReceiver(Node):
    def __init__(self):
        super().__init__('imu_receiver')
        
        # Declare parameters
        self.declare_parameter('broker_address', DEFAULT_BROKER_ADDRESS)
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('topic', DEFAULT_TOPIC)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_topic', 'imu')
        
        # Get parameters
        self.broker_address = self.get_parameter('broker_address').value
        self.port = self.get_parameter('port').value
        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.imu_topic = self.get_parameter('imu_topic').value
        
        # Create QoS profile for IMU messages
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publisher
        self.imu_publisher = self.create_publisher(
            Imu,
            self.imu_topic,
            qos_profile
        )
        
        # MQTT client and connection status
        self.mqtt_client = None
        self.mqtt_connected = False
        
        # Setup MQTT client
        self.setup_mqtt()
        
        # Start connection
        self.connect_mqtt()
        
        self.get_logger().info(f"IMU receiver initialized, connecting to {self.broker_address}:{self.port}")
        self.get_logger().info(f"Subscribing to MQTT topic: {self.topic}")
        self.get_logger().info(f"Publishing IMU messages on topic: {self.imu_topic}")
    
    def setup_mqtt(self):
        """Set up MQTT client"""
        try:
            # Create MQTT client
            self.mqtt_client = mqtt.Client()
            
            # Set callbacks
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
        except Exception as e:
            self.get_logger().error(f"Error setting up MQTT client: {e}")
    
    def connect_mqtt(self):
        """Connect to MQTT broker"""
        try:
            self.get_logger().info("Connecting to MQTT broker...")
            
            # Connect to broker
            self.mqtt_client.connect_async(self.broker_address, self.port, 60)
            
            # Start MQTT loop in a separate thread
            self.mqtt_client.loop_start()
            
        except Exception as e:
            self.get_logger().error(f"Error connecting to MQTT broker: {e}")
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback for MQTT connection"""
        if rc == 0:
            # Successfully connected
            self.mqtt_connected = True
            self.get_logger().info("Connected to MQTT broker")
            
            # Subscribe to IMU topic
            client.subscribe(self.topic)
            self.get_logger().info(f"Subscribed to topic: {self.topic}")
        else:
            # Connection failed
            self.mqtt_connected = False
            self.get_logger().error(f"Failed to connect to MQTT broker, return code: {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback for MQTT disconnection"""
        self.mqtt_connected = False
        
        if rc == 0:
            # Clean disconnect
            self.get_logger().info("Disconnected from MQTT broker")
        else:
            # Unexpected disconnect
            self.get_logger().warning(f"Unexpectedly disconnected from MQTT broker, return code: {rc}")
            # Try to reconnect
            self.get_logger().info("Attempting to reconnect...")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Callback for MQTT message received"""
        try:
            # Parse JSON payload
            payload = json.loads(msg.payload.decode('utf-8'))
            
            # Process IMU data and publish to ROS
            self.process_imu_data(payload)
            
        except json.JSONDecodeError:
            self.get_logger().warning("Received invalid JSON from MQTT")
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")
    
    def process_imu_data(self, data):
        """Process IMU data and publish to ROS topic"""
        # Create IMU message
        imu_msg = Imu()
        
        # Set header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        
        # Get quaternion orientation
        quat = data.get("quaternion", {})
        imu_msg.orientation = Quaternion(
            x=float(quat.get("x", 0.0)),
            y=float(quat.get("y", 0.0)),
            z=float(quat.get("z", 0.0)),
            w=float(quat.get("w", 1.0))  # Default to identity quaternion if missing
        )
        
        # Get angular velocity (convert from deg/s to rad/s if needed)
        gyro = data.get("angular_velocity", {})
        imu_msg.angular_velocity = Vector3(
            x=float(gyro.get("x", 0.0)),
            y=float(gyro.get("y", 0.0)),
            z=float(gyro.get("z", 0.0))
        )
        
        # Get linear acceleration (already in m/s^2)
        accel = data.get("linear_acceleration", {})
        imu_msg.linear_acceleration = Vector3(
            x=float(accel.get("x", 0.0)),
            y=float(accel.get("y", 0.0)),
            z=float(accel.get("z", 0.0))
        )
        
        # Set covariance matrices based on calibration
        calib = data.get("calibration", {})
        sys_cal = int(calib.get("system", 0))
        gyro_cal = int(calib.get("gyro", 0))
        accel_cal = int(calib.get("accel", 0))
        
        # Set orientation covariance based on system calibration (0-3)
        # 0 = uncalibrated, 3 = fully calibrated
        if sys_cal == 3:  # Fully calibrated
            orientation_variance = 0.01  # Low variance
        elif sys_cal == 2:
            orientation_variance = 0.05
        elif sys_cal == 1:
            orientation_variance = 0.1
        else:  # Uncalibrated
            orientation_variance = 0.5  # High variance
        
        # Set main diagonal elements (x, y, z)
        for i in range(3):
            imu_msg.orientation_covariance[i*4] = orientation_variance
        
        # Set angular velocity covariance based on gyro calibration
        if gyro_cal == 3:
            angular_velocity_variance = 0.01
        elif gyro_cal == 2:
            angular_velocity_variance = 0.05
        elif gyro_cal == 1:
            angular_velocity_variance = 0.1
        else:
            angular_velocity_variance = 0.5
        
        for i in range(3):
            imu_msg.angular_velocity_covariance[i*4] = angular_velocity_variance
        
        # Set linear acceleration covariance based on accel calibration
        if accel_cal == 3:
            linear_acceleration_variance = 0.1
        elif accel_cal == 2:
            linear_acceleration_variance = 0.2
        elif accel_cal == 1:
            linear_acceleration_variance = 0.5
        else:
            linear_acceleration_variance = 1.0
        
        for i in range(3):
            imu_msg.linear_acceleration_covariance[i*4] = linear_acceleration_variance
        
        # Publish the IMU message
        self.imu_publisher.publish(imu_msg)
        self.get_logger().debug("Published IMU message")
    
    def destroy_node(self):
        """Clean up resources when node is shut down"""
        self.get_logger().info("Shutting down IMU receiver...")
        
        if self.mqtt_client:
            # Disconnect from MQTT
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except Exception as e:
                self.get_logger().error(f"Error disconnecting from MQTT: {e}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IMUReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()