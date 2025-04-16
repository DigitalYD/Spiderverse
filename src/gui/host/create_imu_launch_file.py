#!/usr/bin/env python3

import os
import sys
import argparse
import subprocess

def create_cartographer_imu_launch_file():
    """
    Creates a ROS2 launch file for Cartographer SLAM with IMU odometry
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Create Cartographer launch file with IMU odometry')
    parser.add_argument('--output', type=str, 
                        default='',
                        help='Output path for the launch file')
    args = parser.parse_args()
    
    # Always use the user's workspace to avoid permission issues
    user_ws_path = os.path.expanduser("~/Documents/Spiderverse/lidar_ws")
    
    # Create src directory if it doesn't exist
    src_dir = os.path.join(user_ws_path, "src")
    os.makedirs(src_dir, exist_ok=True)
    
    # Create directories for launch files
    cartographer_dir = os.path.join(src_dir, "cartographer_ros")
    os.makedirs(cartographer_dir, exist_ok=True)
    
    launch_dir = os.path.join(cartographer_dir, "launch")
    os.makedirs(launch_dir, exist_ok=True)
    
    # Set output path
    if args.output:
        output_path = os.path.expanduser(args.output)
    else:
        output_path = os.path.join(launch_dir, "cartographer_slam_imu.launch.py")
    
    print(f"Will create launch file at: {output_path}")
    
    # Generate the launch file content
    launch_content = """#!/usr/bin/env python3

# Licensed under the Apache License, Version 2.0

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')
    
    # Customized for IMU-based odometry
    imu_topic = LaunchConfiguration('imu_topic')
    
    # Set up Cartographer configuration
    cartographer_config = os.path.join(
        get_package_share_directory('cartographer_ros'),
        'configuration_files',
        LaunchConfiguration('configuration_basename')
    )
    
    # Create IMU to odometry node (Python script)
    imu_odom_script = os.path.expanduser('~/Documents/Spiderverse/lidar_ws/src/imu_to_odom.py')
    with open(imu_odom_script, 'w') as f:
        f.write('''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import tf2_ros
import math
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class ImuToOdomNode(Node):
    def __init__(self):
        super().__init__('imu_to_odom_node')
        
        # Create subscribers and publishers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',  # Will be remapped from launch file
            self.imu_callback,
            10)
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',  # Will be remapped from launch file
            10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # State variables
        self.position = [0.0, 0.0, 0.0]  # x, y, z
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # quaternion
        self.velocity = [0.0, 0.0, 0.0]  # vx, vy, vz
        self.prev_time = None
        
        self.get_logger().info('IMU to Odometry node initialized')
    
    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        
        # First message handling
        if self.prev_time is None:
            self.prev_time = current_time
            self.orientation = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]
            return
        
        # Calculate time delta in seconds
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time
        
        # Extract IMU data
        orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        
        # Get linear acceleration in body frame
        accel = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        
        # Convert to world frame using orientation
        roll, pitch, yaw = euler_from_quaternion(orientation)
        
        # Simple rotation to world frame (assuming z is up)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        # Transform acceleration to world frame (simplified)
        world_accel_x = accel[0] * cos_yaw - accel[1] * sin_yaw
        world_accel_y = accel[0] * sin_yaw + accel[1] * cos_yaw
        
        # Apply high-pass filter to remove bias
        accel_threshold = 0.15  # m/s²
        if abs(world_accel_x) < accel_threshold:
            world_accel_x = 0.0
        if abs(world_accel_y) < accel_threshold:
            world_accel_y = 0.0
        
        # Update velocity (integrate acceleration)
        self.velocity[0] += world_accel_x * dt
        self.velocity[1] += world_accel_y * dt
        
        # Apply damping to velocity
        damping = 0.98  # 0.95 to 0.99
        self.velocity[0] *= damping
        self.velocity[1] *= damping
        
        # Update position (integrate velocity)
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        
        # Update orientation from IMU
        self.orientation = orientation
        
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation
        odom_msg.pose.pose.orientation.x = self.orientation[0]
        odom_msg.pose.pose.orientation.y = self.orientation[1]
        odom_msg.pose.pose.orientation.z = self.orientation[2]
        odom_msg.pose.pose.orientation.w = self.orientation[3]
        
        # Set velocity
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = 0.0
        
        # Extract angular velocity from IMU
        odom_msg.twist.twist.angular.x = msg.angular_velocity.x
        odom_msg.twist.twist.angular.y = msg.angular_velocity.y
        odom_msg.twist.twist.angular.z = msg.angular_velocity.z
        
        # Publish odometry message
        self.odom_pub.publish(odom_msg)
        
        # Publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        transform.transform.translation.x = self.position[0]
        transform.transform.translation.y = self.position[1]
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = self.orientation[0]
        transform.transform.rotation.y = self.orientation[1]
        transform.transform.rotation.z = self.orientation[2]
        transform.transform.rotation.w = self.orientation[3]
        
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = ImuToOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
''')
    os.chmod(imu_odom_script, 0o755)

    # Create IMU MQTT to ROS bridge script
    imu_bridge_script = os.path.expanduser('~/Documents/Spiderverse/lidar_ws/src/imu_mqtt_bridge.py')
    with open(imu_bridge_script, 'w') as f:
        f.write('''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import json
import time
import math
import paho.mqtt.client as mqtt

# Default MQTT configuration
MQTT_BROKER = "192.168.0.49"
MQTT_PORT = 1883
MQTT_TOPIC = "pi5/imu"

class ImuMqttBridge(Node):
    def __init__(self):
        super().__init__('imu_mqtt_bridge')
        
        # Declare parameters
        self.declare_parameter('mqtt_broker', MQTT_BROKER)
        self.declare_parameter('mqtt_port', MQTT_PORT)
        self.declare_parameter('mqtt_topic', MQTT_TOPIC)
        
        # Get parameters
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        
        # Create publishers
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        
        # MQTT setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # Connect to MQTT broker
        self.get_logger().info(f'Connecting to MQTT broker {self.mqtt_broker}:{self.mqtt_port}')
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')
        
        self.get_logger().info('IMU MQTT Bridge started')
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f'Connected to MQTT broker {self.mqtt_broker}')
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
            imu_msg.header.frame_id = 'imu_link'
            
            # Fill orientation from quaternion
            quat = payload.get('quaternion', {})
            imu_msg.orientation.x = quat.get('x', 0.0)
            imu_msg.orientation.y = quat.get('y', 0.0)
            imu_msg.orientation.z = quat.get('z', 0.0)
            imu_msg.orientation.w = quat.get('w', 1.0)
            
            # Orientation covariance - set to unknown if calibration is poor
            calibration = payload.get('calibration', {})
            sys_cal = calibration.get('system', 0)
            if sys_cal >= 2:
                # Well calibrated - set reasonable covariance
                imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            else:
                # Poorly calibrated - set as unknown
                imu_msg.orientation_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]
            
            # Fill angular velocity
            gyro = payload.get('angular_velocity', {})
            imu_msg.angular_velocity.x = gyro.get('x', 0.0)
            imu_msg.angular_velocity.y = gyro.get('y', 0.0)
            imu_msg.angular_velocity.z = gyro.get('z', 0.0)
            
            # Angular velocity covariance - reasonable values
            gyro_cal = calibration.get('gyro', 0)
            if gyro_cal >= 2:
                imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            else:
                imu_msg.angular_velocity_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
            
            # Fill linear acceleration
            accel = payload.get('linear_acceleration', {})
            imu_msg.linear_acceleration.x = accel.get('x', 0.0)
            imu_msg.linear_acceleration.y = accel.get('y', 0.0)
            imu_msg.linear_acceleration.z = accel.get('z', 0.0)
            
            # Linear acceleration covariance - reasonable values
            accel_cal = calibration.get('accel', 0)
            if accel_cal >= 2:
                imu_msg.linear_acceleration_covariance = [0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05]
            else:
                imu_msg.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
            
            # Publish IMU message
            self.imu_publisher.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {e}')
    
    def destroy_node(self):
        # Clean up MQTT connection
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuMqttBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
''')
    os.chmod(imu_bridge_script, 0o755)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of the map (meters per pixel)'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='Map publishing period'),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=os.path.join(
                get_package_share_directory('cartographer_ros'), 'configuration_files'),
            description='Path to Cartographer configuration files'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='backpack_2d.lua',
            description='Cartographer configuration file name'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu',
            description='Topic for IMU data'),

        # IMU MQTT Bridge Node
        Node(
            package='cartographer_ros',  # Just for dependency, actual script is executed below
            executable='cartographer_node',  # Will be overridden
            name='imu_mqtt_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'mqtt_broker': '192.168.0.49'},
                {'mqtt_port': 1883},
                {'mqtt_topic': 'pi5/imu'}
            ],
            prefix=['python3 ' + imu_bridge_script],
            remappings=[('imu', '/imu')],
        ),

        # IMU to Odometry converter for better integration
        Node(
            package='cartographer_ros',  # Just for dependency, actual script is executed below
            executable='cartographer_node',  # Will be overridden
            name='imu_odom_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            prefix=['python3 ' + imu_odom_script],
            remappings=[
                ('imu', '/imu'),
                ('odom', '/odom')
            ],
        ),

        # Cartographer node with IMU configuration
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
                # Additional arguments for IMU integration
                '-subscribe_to_imu_needed=true'
            ],
            remappings=[
                ('scan', '/scan'),
                ('imu', '/imu'),
                ('odom', '/odom')
            ]),
        
        # Add a static transform publisher for base_link to lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_broadcaster',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'laser'],
            output='screen'),
            
        # Add a static transform publisher for base_link to imu
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_broadcaster',
            arguments=['0', '0', '0.02', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'),

        # Occupancy grid node
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                        'resolution': resolution,
                        'publish_period_sec': publish_period_sec}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),
    ])
"""
    
    # Write the launch file
    with open(output_path, 'w') as f:
        f.write(launch_content)
    
    # Make the file executable
    os.chmod(output_path, 0o755)
    
    print(f"Launch file created at: {output_path}")
    print("This launch file configures Cartographer to use IMU data for odometry.")
    print("NOTE: You may need to install the 'imu_odom_transformer' package or modify the launch file if this package isn't available.")

if __name__ == "__main__":
    # Ensure cartographer_ros package is installed
    try:
        subprocess.check_call(["ros2", "pkg", "list"], stdout=subprocess.PIPE)
    except subprocess.CalledProcessError:
        print("ROS 2 is not properly sourced. Make sure to source your ROS 2 installation.")
        sys.exit(1)
        
    create_cartographer_imu_launch_file()