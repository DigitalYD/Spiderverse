#!/usr/bin/env python3

import os
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import paho.mqtt.client as mqtt

class IMUMQTTBridge(Node):
    def __init__(self):
        super().__init__('imu_mqtt_bridge')
        
        # Get parameters from environment variables
        self.broker_address = os.environ.get('BROKER_ADDRESS', '192.168.0.49')
        self.broker_port = int(os.environ.get('BROKER_PORT', '1883'))
        self.mqtt_topic = os.environ.get('MQTT_TOPIC', 'pi5/imu')
        self.imu_frame = os.environ.get('IMU_FRAME', 'imu_link')
        
        # Log the parameters
        self.get_logger().info(f"Broker address: {self.broker_address}")
        self.get_logger().info(f"Broker port: {self.broker_port}")
        self.get_logger().info(f"MQTT topic: {self.mqtt_topic}")
        self.get_logger().info(f"IMU frame: {self.imu_frame}")
        
        # Create a QoS profile that is compatible with subscribers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publisher with improved QoS
        self.imu_publisher = self.create_publisher(Imu, '/imu', qos)
        
        # Rate limiting to avoid overwhelming the system
        self.publish_rate = 20.0  # Hz (adjust as needed)
        self.last_publish_time = 0.0
        
        # MQTT setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # Connect to MQTT broker
        self.get_logger().info(f'Connecting to MQTT broker at {self.broker_address}:{self.broker_port}')
        try:
            self.mqtt_client.connect(self.broker_address, self.broker_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')
        
        self.get_logger().info('IMU MQTT Bridge started')
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f'Connected to MQTT broker {self.broker_address}')
            client.subscribe(self.mqtt_topic)
            self.get_logger().info(f'Subscribed to topic {self.mqtt_topic}')
        else:
            self.get_logger().error(f'Failed to connect to MQTT broker: {rc}')
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        if rc == 0:
            self.get_logger().info('Disconnected from MQTT broker')
        else:
            self.get_logger().warning(f'Unexpected disconnect from MQTT broker: {rc}')
    
    def on_mqtt_message(self, client, userdata, msg):
        try:
            # Parse JSON payload
            payload = json.loads(msg.payload.decode('utf-8'))
            
            # Create IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame
            
            # Fill orientation from quaternion
            quat = payload.get('quaternion', {})
            imu_msg.orientation.x = float(quat.get('x', 0.0))
            imu_msg.orientation.y = float(quat.get('y', 0.0))
            imu_msg.orientation.z = float(quat.get('z', 0.0))
            imu_msg.orientation.w = float(quat.get('w', 1.0))
            
            # Orientation covariance - must be exactly 9 float values
            calibration = payload.get('calibration', {})
            sys_cal = calibration.get('system', 0)
            
            # Create properly formatted covariance matrices (must be exactly 9 float values)
            if sys_cal >= 2:
                # Well calibrated - set reasonable covariance
                orientation_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            else:
                # Poorly calibrated - set as unknown (first value as -1)
                orientation_cov = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                
            # Make sure we have exactly 9 float values of proper type
            imu_msg.orientation_covariance = [float(x) for x in orientation_cov]
            
            # Fill angular velocity
            gyro = payload.get('angular_velocity', {})
            imu_msg.angular_velocity.x = float(gyro.get('x', 0.0))
            imu_msg.angular_velocity.y = float(gyro.get('y', 0.0))
            imu_msg.angular_velocity.z = float(gyro.get('z', 0.0))
            
            # Angular velocity covariance - must be exactly 9 float values
            gyro_cal = calibration.get('gyro', 0)
            if gyro_cal >= 2:
                vel_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            else:
                vel_cov = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
            
            # Make sure we have exactly 9 float values of proper type
            imu_msg.angular_velocity_covariance = [float(x) for x in vel_cov]
            
            # Fill linear acceleration
            accel = payload.get('linear_acceleration', {})
            imu_msg.linear_acceleration.x = float(accel.get('x', 0.0))
            imu_msg.linear_acceleration.y = float(accel.get('y', 0.0))
            imu_msg.linear_acceleration.z = float(accel.get('z', 0.0))
            
            # Linear acceleration covariance - must be exactly 9 float values
            accel_cal = calibration.get('accel', 0)
            if accel_cal >= 2:
                accel_cov = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
            else:
                accel_cov = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
            
            # Make sure we have exactly 9 float values of proper type
            imu_msg.linear_acceleration_covariance = [float(x) for x in accel_cov]
            
            # Rate-limit publishing to avoid overwhelming ROS
            current_time = time.time()
            elapsed = current_time - self.last_publish_time
            
            # Only publish if enough time has passed (limit to publish_rate Hz)
            if elapsed >= (1.0 / self.publish_rate):
                self.imu_publisher.publish(imu_msg)
                self.last_publish_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {e}')
    
    def destroy_node(self):
        # Clean up MQTT connection
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()

def main(args=None):
    # Parse command-line arguments
    import argparse
    parser = argparse.ArgumentParser(description='IMU MQTT to ROS Bridge')
    parser.add_argument('--rate', type=float, default=0.0,
                        help='IMU publishing rate in Hz (default: use environment variable or 20 Hz)')
    parsed_args = parser.parse_args()
    
    # Initialize ROS
    rclpy.init(args=args)
    node = IMUMQTTBridge()
    
    # Override publish rate if specified on command line
    if parsed_args.rate > 0:
        node.publish_rate = parsed_args.rate
        node.get_logger().info(f"IMU publish rate set to {node.publish_rate} Hz from command line")
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()