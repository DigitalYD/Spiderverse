#!/bin/bash
#
# IMU-based SLAM launch script - Self-contained version
#

# Set script to exit on error
set -e

# Define workspace and package directories
LIDAR_WS="$HOME/Documents/Spiderverse/lidar_ws"
SRC_DIR="$LIDAR_WS/src"
ROS_LOG_DIR="$LIDAR_WS/logs"

# Create directories if they don't exist
mkdir -p "$ROS_LOG_DIR"
mkdir -p "$SRC_DIR"

# Set ROS environment variables
source /opt/ros/humble/setup.bash

# Log script start
echo "=================================================="
echo "      Starting IMU-based SLAM (Direct Version)    "
echo "=================================================="
echo "$(date)"

# Create an IMU publisher node script
IMU_MQTT_SCRIPT="$SRC_DIR/imu_mqtt_bridge.py"
cat > "$IMU_MQTT_SCRIPT" << 'EOF'
#!/usr/bin/env python3
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
        
        # Create publishers
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        
        # MQTT setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # Connect to MQTT broker
        self.get_logger().info(f'Connecting to MQTT broker {MQTT_BROKER}:{MQTT_PORT}')
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')
        
        self.get_logger().info('IMU MQTT Bridge started')
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f'Connected to MQTT broker {MQTT_BROKER}')
            client.subscribe(MQTT_TOPIC)
            self.get_logger().info(f'Subscribed to topic {MQTT_TOPIC}')
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
EOF
chmod +x "$IMU_MQTT_SCRIPT"

# Create an IMU to Odometry converter script
IMU_ODOM_SCRIPT="$SRC_DIR/imu_to_odom.py"
cat > "$IMU_ODOM_SCRIPT" << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import tf2_ros
import math
import numpy as np

# Simplified version without transforms3d dependency
def euler_from_quaternion(q):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x, y, z, w = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [x, y, z, w]

class ImuToOdomNode(Node):
    def __init__(self):
        super().__init__('imu_to_odom_node')
        
        # Create subscribers and publishers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
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
EOF
chmod +x "$IMU_ODOM_SCRIPT"

# Create a simple launch script 
LAUNCH_SCRIPT="$SRC_DIR/imu_slam_launch.py"
cat > "$LAUNCH_SCRIPT" << 'EOF'
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    
    # Get script paths from environment
    src_dir = os.environ.get('SRC_DIR', os.path.expanduser('~/Documents/Spiderverse/lidar_ws/src'))
    imu_mqtt_script = os.path.join(src_dir, 'imu_mqtt_bridge.py')
    imu_odom_script = os.path.join(src_dir, 'imu_to_odom.py')
    
    # Set the cartographer configuration
    cartographer_config_dir = os.path.join(
        get_package_share_directory('cartographer_ros'), 'configuration_files')
    configuration_basename = 'backpack_2d.lua'
    
    # Verify files exist
    for script in [imu_mqtt_script, imu_odom_script]:
        if not os.path.exists(script):
            raise FileNotFoundError(f"Required script not found: {script}")
    
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
            
        # IMU MQTT Bridge Node
        Node(
            package='cartographer_ros',  # Just for dependency, actual script is executed below
            executable='occupancy_grid_node',  # Will be overridden
            name='imu_mqtt_bridge',
            output='screen',
            prefix=['python3 ' + imu_mqtt_script],
        ),

        # IMU to Odometry converter for better integration
        Node(
            package='cartographer_ros',  # Just for dependency, actual script is executed below
            executable='occupancy_grid_node',  # Will be overridden
            name='imu_odom_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            prefix=['python3 ' + imu_odom_script],
        ),
            
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
EOF
chmod +x "$LAUNCH_SCRIPT"

# Check if we have required packages
if ! python3 -c "import paho.mqtt" 2>/dev/null; then
  echo "Installing paho-mqtt package..."
  pip3 install --user paho-mqtt || echo "Warning: Could not install paho-mqtt"
fi

# Launch the SLAM system with IMU
echo "Launching ROS2 with IMU-based SLAM..."
export SRC_DIR="$SRC_DIR"
exec ros2 launch "$LAUNCH_SCRIPT"

# Note: This script will exit once ROS2 launch exits
echo "IMU-based SLAM finished"