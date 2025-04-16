#!/usr/bin/env python3

import os
import sys
import subprocess
import threading
import time
import signal
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import tf2_ros
import math
import numpy as np
import paho.mqtt.client as mqtt

# Default MQTT configuration
MQTT_BROKER = "192.168.0.49"
MQTT_PORT = 1883
MQTT_TOPIC = "pi5/imu"

# Print colored messages
def print_green(text):
    print(f"\033[92m{text}\033[0m")

def print_yellow(text):
    print(f"\033[93m{text}\033[0m")

def print_red(text):
    print(f"\033[91m{text}\033[0m")

# Simplified quaternion functions
def euler_from_quaternion(q):
    """Convert a quaternion to Euler angles (roll, pitch, yaw)"""
    x, y, z, w = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion"""
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

class ImuMqttBridge(Node):
    """ROS node that bridges MQTT IMU data to ROS topics"""
    def __init__(self, mqtt_broker=MQTT_BROKER, mqtt_port=MQTT_PORT, mqtt_topic=MQTT_TOPIC):
        super().__init__('imu_mqtt_bridge')
        
        # Create publishers
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        
        # MQTT setup
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_topic = mqtt_topic
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
            imu_msg.angular_velocity.x = gyro.get('x', 0.0)
            imu_msg.angular_velocity.y = gyro.get('y', 0.0)
            imu_msg.angular_velocity.z = gyro.get('z', 0.0)
            
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
            imu_msg.linear_acceleration.x = accel.get('x', 0.0)
            imu_msg.linear_acceleration.y = accel.get('y', 0.0)
            imu_msg.linear_acceleration.z = accel.get('z', 0.0)
            
            # Linear acceleration covariance - must be exactly 9 float values
            accel_cal = calibration.get('accel', 0)
            if accel_cal >= 2:
                accel_cov = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
            else:
                accel_cov = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
            
            # Make sure we have exactly 9 float values of proper type
            imu_msg.linear_acceleration_covariance = [float(x) for x in accel_cov]
            
            # Publish IMU message
            self.imu_publisher.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {e}')
    
    def destroy_node(self):
        # Clean up MQTT connection
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()

class ImuToOdomNode(Node):
    """ROS node that converts IMU data to odometry information"""
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
        
        # Also publish static transforms
        self.publish_static_transforms()
        
        self.get_logger().info('IMU to Odometry node initialized')
    
    def publish_static_transforms(self):
        """Publish static transforms between coordinate frames"""
        # Create a static transform publisher
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Create base_link to laser transform
        laser_transform = TransformStamped()
        laser_transform.header.stamp = self.get_clock().now().to_msg()
        laser_transform.header.frame_id = 'base_link'
        laser_transform.child_frame_id = 'laser'
        
        laser_transform.transform.translation.x = 0.0
        laser_transform.transform.translation.y = 0.0
        laser_transform.transform.translation.z = 0.05
        
        quat = quaternion_from_euler(0, 0, 0)
        laser_transform.transform.rotation.x = quat[0]
        laser_transform.transform.rotation.y = quat[1]
        laser_transform.transform.rotation.z = quat[2]
        laser_transform.transform.rotation.w = quat[3]
        
        # Create base_link to imu_link transform - make it coincide with base_link
        imu_transform = TransformStamped()
        imu_transform.header.stamp = self.get_clock().now().to_msg()
        imu_transform.header.frame_id = 'base_link'
        imu_transform.child_frame_id = 'imu_link'
        
        # Zero translation to avoid Cartographer's error about sensor_to_tracking->translation().norm() < 1e-5
        imu_transform.transform.translation.x = 0.0
        imu_transform.transform.translation.y = 0.0
        imu_transform.transform.translation.z = 0.0
        
        quat = quaternion_from_euler(0, 0, 0)
        imu_transform.transform.rotation.x = quat[0]
        imu_transform.transform.rotation.y = quat[1]
        imu_transform.transform.rotation.z = quat[2]
        imu_transform.transform.rotation.w = quat[3]
        
        # Send the transforms
        self.static_broadcaster.sendTransform([laser_transform, imu_transform])
        self.get_logger().info('Published static transforms')
    
    def imu_callback(self, msg):
        """Process incoming IMU data and generate odometry"""
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

def run_imu_mqtt_bridge(mqtt_broker, mqtt_port, mqtt_topic, executor):
    """Run the IMU MQTT bridge node"""
    # rclpy.init is called in main() once for all nodes
    node = ImuMqttBridge(mqtt_broker, mqtt_port, mqtt_topic)
    # Add the node to the executor instead of spinning it separately
    executor.add_node(node)
    return node

def run_imu_odom_node(executor):
    """Run the IMU to odometry node"""
    # rclpy.init is called in main() once for all nodes
    node = ImuToOdomNode()
    # Add the node to the executor instead of spinning it separately
    executor.add_node(node)
    return node

def run_cartographer():
    """Run Cartographer SLAM"""
    # First try to find configuration directory
    cartographer_config_dir = None
    
    # List of possible paths for configuration files
    config_paths = [
        "/opt/ros/humble/share/cartographer_ros/configuration_files",
        "/opt/ros/foxy/share/cartographer_ros/configuration_files",
        os.path.expanduser("~/Documents/Spiderverse/lidar_ws/install/cartographer_ros/share/cartographer_ros/configuration_files"),
        os.path.expanduser("~/Documents/Spiderverse_old/lidar_ws/install/cartographer_ros/share/cartographer_ros/configuration_files")
    ]
    
    try:
        # Try to find the cartographer_ros package
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "cartographer_ros"], 
            capture_output=True, 
            text=True
        )
        if result.returncode == 0:
            pkg_path = result.stdout.strip()
            config_paths.insert(0, os.path.join(pkg_path, "share", "cartographer_ros", "configuration_files"))
    except:
        pass
    
    # Try each path
    for path in config_paths:
        if os.path.exists(path):
            cartographer_config_dir = path
            print_green(f"Found Cartographer configuration at: {path}")
            break
    
    if not cartographer_config_dir:
        print_red("Error: Could not find Cartographer configuration directory")
        
        # Try to create one as a last resort
        try:
            # Create a basic configuration
            config_path = os.path.expanduser("~/Documents/Spiderverse/lidar_ws/cartographer_config")
            os.makedirs(config_path, exist_ok=True)
            
            # Create a basic Cartographer configuration (backpack_2d.lua)
            lua_file = os.path.join(config_path, "backpack_2d.lua")
            with open(lua_file, "w") as f:
                f.write("""
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",   -- Changed from imu_link to base_link
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
                """)
            
            cartographer_config_dir = config_path
            print_yellow(f"Created basic configuration at: {config_path}")
        except Exception as e:
            print_red(f"Failed to create configuration: {e}")
            return False
    
    # Try different ways to run Cartographer
    
    # First try with ros2 run
    print_green("Trying to run Cartographer with 'ros2 run'...")
    cmd1 = [
        "ros2", "run", "cartographer_ros", "cartographer_node",
        "-configuration_directory", cartographer_config_dir,
        "-configuration_basename", "backpack_2d.lua",
        "-subscribe_to_imu_needed=true"
    ]
    
    # Direct executable path as fallback
    direct_path = "/opt/ros/humble/lib/cartographer_ros/cartographer_node"
    cmd2 = [
        direct_path,
        "-configuration_directory", cartographer_config_dir,
        "-configuration_basename", "backpack_2d.lua",
        "-subscribe_to_imu_needed=true"
    ]
    
    # Try the first command
    try:
        print_yellow(f"Command: {' '.join(cmd1)}")
        process = subprocess.Popen(cmd1)
        time.sleep(1)  # Give it a moment to start
        if process.poll() is None:  # Still running
            return process
        else:
            print_yellow(f"First attempt failed with code {process.returncode}, trying direct path...")
    except Exception as e:
        print_yellow(f"First attempt failed: {e}")
    
    # Try the direct path
    try:
        print_yellow(f"Command: {' '.join(cmd2)}")
        process = subprocess.Popen(cmd2)
        return process
    except Exception as e:
        print_red(f"Failed to start Cartographer: {e}")
        return None

def run_occupancy_grid():
    """Run the occupancy grid node"""
    # First try with ros2 run
    cmd1 = [
        "ros2", "run", "cartographer_ros", "occupancy_grid_node",
        "-resolution", "0.05",
        "-publish_period_sec", "1.0"
    ]
    
    # Direct executable path as fallback
    direct_path = "/opt/ros/humble/lib/cartographer_ros/occupancy_grid_node"
    cmd2 = [
        direct_path,
        "-resolution", "0.05",
        "-publish_period_sec", "1.0"
    ]
    
    # Try the first command
    try:
        print_yellow(f"Command: {' '.join(cmd1)}")
        process = subprocess.Popen(cmd1)
        time.sleep(1)  # Give it a moment to start
        if process.poll() is None:  # Still running
            return process
        else:
            print_yellow(f"First attempt failed with code {process.returncode}, trying direct path...")
    except Exception as e:
        print_yellow(f"First attempt failed: {e}")
    
    # Try the direct path
    try:
        print_yellow(f"Command: {' '.join(cmd2)}")
        process = subprocess.Popen(cmd2)
        return process
    except Exception as e:
        print_red(f"Failed to start Occupancy Grid node: {e}")
        return None

def main():
    """Main entry point for the IMU SLAM application"""
    print_green("="*70)
    print_green("         IMU-Based SLAM Launcher (Direct Version)             ")
    print_green("="*70)
    
    # Check for Cartographer installation
    cartographer_installed = False
    try:
        # Try to find the cartographer_ros package
        result = subprocess.run(
            ["ros2", "pkg", "list", "--packages-select", "cartographer_ros"], 
            capture_output=True, 
            text=True
        )
        if "cartographer_ros" in result.stdout:
            cartographer_installed = True
    except:
        pass
    
    if not cartographer_installed:
        print_yellow("\nNOTICE: cartographer_ros package not found!")
        print_yellow("This script will still run the IMU bridge and odometry components,")
        print_yellow("but the SLAM functionality will not be available.")
        print_yellow("If you want full SLAM functionality, please install cartographer_ros:")
        print_yellow("  sudo apt install ros-humble-cartographer-ros")
    
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Launch SLAM with IMU-based odometry')
    parser.add_argument('--broker', type=str, default=MQTT_BROKER,
                        help=f'MQTT broker address (default: {MQTT_BROKER})')
    parser.add_argument('--port', type=int, default=MQTT_PORT,
                        help=f'MQTT broker port (default: {MQTT_PORT})')
    parser.add_argument('--topic', type=str, default=MQTT_TOPIC,
                        help=f'MQTT topic for IMU data (default: {MQTT_TOPIC})')
    args = parser.parse_args()
    
    # Check if ROS is properly sourced
    try:
        subprocess.run(["ros2", "--help"], capture_output=True, check=True)
    except:
        print_red("Error: ROS 2 not properly sourced. Please source your ROS 2 installation first.")
        print_yellow("  source /opt/ros/humble/setup.bash")
        return 1
    
    # Make sure required packages are installed - more flexible detection
    result = subprocess.run(["ros2", "pkg", "list"], capture_output=True, text=True)
    if "cartographer_ros" not in result.stdout:
        # Double-check with a direct executable check
        cartographer_node_path = "/opt/ros/humble/lib/cartographer_ros/cartographer_node"
        if not os.path.exists(cartographer_node_path):
            print_red("Error: cartographer_ros package not found. Please install it with:")
            print_yellow("  sudo apt install ros-humble-cartographer-ros")
            print_yellow("\nIf you've already installed it, make sure ROS environment is properly sourced:")
            print_yellow("  source /opt/ros/humble/setup.bash")
            return 1
        else:
            print_yellow("Warning: cartographer_ros not found in package list, but executable exists.")
            print_yellow("Continuing with available executables...")
    
    # Start ROS processes
    processes = []
    
    # Initialize rclpy once for all nodes
    print_green("Initializing ROS client library...")
    rclpy.init(args=None)
    
    # Create the ROS executor
    print_green("Creating ROS executor...")
    executor = rclpy.executors.MultiThreadedExecutor()
    
    # Create ROS nodes and add them to the executor
    print_green("Starting IMU MQTT Bridge...")
    mqtt_node = run_imu_mqtt_bridge(args.broker, args.port, args.topic, executor)
    
    print_green("Starting IMU to Odometry node...")
    odom_node = run_imu_odom_node(executor)
    
    # Start the executor in a separate thread
    print_green("Starting ROS executor...")
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Keep track of nodes for cleanup
    ros_nodes = [mqtt_node, odom_node]
    
    # Wait a bit for the nodes to initialize
    time.sleep(2)
    
    # Only start Cartographer if it's available
    if cartographer_installed:
        # Start Cartographer
        print_green("Starting Cartographer SLAM...")
        cartographer_process = run_cartographer()
        if cartographer_process:
            processes.append(cartographer_process)
        
        # Start Occupancy Grid node
        print_green("Starting Occupancy Grid node...")
        occupancy_process = run_occupancy_grid()
        if occupancy_process:
            processes.append(occupancy_process)
    else:
        print_yellow("Skipping SLAM components since cartographer_ros is not installed.")
        print_yellow("Only IMU bridge and odometry components are running.")
    
    # Set up signal handler for clean shutdown
    def signal_handler(sig, frame):
        print_yellow("\nShutting down...")
        
        # Terminate external processes
        for process in processes:
            process.terminate()
        
        # Clean up ROS nodes
        print_yellow("Cleaning up ROS nodes...")
        for node in ros_nodes:
            try:
                node.destroy_node()
            except Exception as e:
                print_yellow(f"Error destroying node: {e}")
                
        # Give processes time to terminate
        time.sleep(2)
        for process in processes:
            if process.poll() is None:
                process.kill()
                
        # Shutdown ROS
        print_yellow("Shutting down ROS...")
        try:
            rclpy.shutdown()
        except Exception as e:
            print_yellow(f"ROS shutdown error (ignorable): {e}")
            
        print_green("Shutdown complete")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print_green("\nIMU SLAM system running. Press Ctrl+C to exit.")
    
    # Monitor processes and exit if any of them die
    try:
        while True:
            for i, process in enumerate(processes):
                if process.poll() is not None:
                    print_red(f"Process {i} exited with code {process.returncode}")
                    signal_handler(None, None)
            time.sleep(1)
    except KeyboardInterrupt:
        signal_handler(None, None)
    
    return 0

if __name__ == "__main__":
    try:
        # Check if ROS is sourced by testing for a ROS environment variable
        if "ROS_DISTRO" not in os.environ:
            print_red("ROS environment not detected!")
            print_yellow("Attempting to source ROS automatically...")
            
            # Try to source ROS
            ros_setup_paths = [
                "/opt/ros/humble/setup.bash",
                "/opt/ros/foxy/setup.bash",
                "~/Documents/Spiderverse/lidar_ws/install/setup.bash"
            ]
            
            for path in ros_setup_paths:
                expanded_path = os.path.expanduser(path)
                if os.path.exists(expanded_path):
                    print_yellow(f"Found ROS setup at: {expanded_path}")
                    print_yellow(f"Please run: source {expanded_path}")
                    break
            
            print_yellow("\nAlternatively, run this script in a terminal where ROS is already sourced.")
        
        # Run the main function
        sys.exit(main())
    except Exception as e:
        print_red(f"Unhandled exception: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)