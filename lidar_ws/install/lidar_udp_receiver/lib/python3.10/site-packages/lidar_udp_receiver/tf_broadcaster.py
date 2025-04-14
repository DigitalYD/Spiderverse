#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
import numpy as np

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        
        # Declare parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('use_odometry', True)  # Whether to use odometry messages or simple transforms
        self.declare_parameter('use_imu', True)  # Whether to use IMU data for orientation
        
        # Get parameters
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_odometry = self.get_parameter('use_odometry').value
        self.use_imu = self.get_parameter('use_imu').value
        
        # Create the transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Initialize robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()
        
        # IMU orientation (quaternion)
        self.imu_orientation = [0.0, 0.0, 0.0, 1.0]  # Default to identity quaternion
        
        if self.use_odometry:
            # Subscribe to odometry topic if available
            self.odom_subscription = self.create_subscription(
                Odometry,
                'odom',
                self.odom_callback,
                10)
        
        if self.use_imu:
            # Subscribe to IMU topic
            self.imu_subscription = self.create_subscription(
                Imu,
                'imu',
                self.imu_callback,
                10)
        
        # Always create a timer for static transforms (base_link -> lidar_link, base_link -> imu_link)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_transforms)
        
        self.get_logger().info('TF broadcaster started')
        
    def odom_callback(self, msg):
        """Callback for odometry messages"""
        # Extract position and orientation from the odometry message
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Convert quaternion to yaw (rotation around Z axis)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.theta = math.atan2(siny_cosp, cosy_cosp)
        
        # Publish the transforms
        self.publish_transforms()
        
    def imu_callback(self, msg):
        """Callback for IMU messages"""
        # Store orientation quaternion from IMU
        self.imu_orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        
    def publish_transforms(self):
        """Publish the necessary transforms for the robot"""
        # Get current time
        current_time = self.get_clock().now()
        
        # Publish odom -> base_link transform
        self.publish_odom_to_base_transform(current_time)
        
        # Publish base_link -> lidar_link transform (static)
        self.publish_base_to_lidar_transform(current_time)
        
        # Publish base_link -> imu_link transform (static)
        self.publish_base_to_imu_transform(current_time)
        
    def publish_odom_to_base_transform(self, time_stamp):
        """Publish the odom -> base_link transform"""
        t = TransformStamped()
        
        # Fill in header
        t.header.stamp = time_stamp.to_msg()
        t.header.frame_id = self.odom_frame
        
        # Fill in child frame
        t.child_frame_id = self.base_frame
        
        # Fill in transform
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Use IMU orientation if available, otherwise use computed theta
        if self.use_imu and any(self.imu_orientation):
            t.transform.rotation.x = self.imu_orientation[0]
            t.transform.rotation.y = self.imu_orientation[1]
            t.transform.rotation.z = self.imu_orientation[2]
            t.transform.rotation.w = self.imu_orientation[3]
        else:
            # Convert yaw (theta) to quaternion
            q = self.quaternion_from_euler(0, 0, self.theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)
        
    def publish_base_to_lidar_transform(self, time_stamp):
        """Publish the base_link -> lidar_link transform (static)"""
        t = TransformStamped()
        
        # Fill in header
        t.header.stamp = time_stamp.to_msg()
        t.header.frame_id = self.base_frame
        
        # Fill in child frame
        t.child_frame_id = self.lidar_frame
        
        # Fill in transform (adjust these values based on your robot's physical configuration)
        t.transform.translation.x = 0.0  # Lidar X offset from base
        t.transform.translation.y = 0.0  # Lidar Y offset from base
        t.transform.translation.z = 0.1  # Lidar Z offset from base (10cm above)
        
        # Identity quaternion (no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)
        
    def publish_base_to_imu_transform(self, time_stamp):
        """Publish the base_link -> imu_link transform (static)"""
        t = TransformStamped()
        
        # Fill in header
        t.header.stamp = time_stamp.to_msg()
        t.header.frame_id = self.base_frame
        
        # Fill in child frame
        t.child_frame_id = self.imu_frame
        
        # Fill in transform (adjust these values based on your robot's physical configuration)
        t.transform.translation.x = 0.0  # IMU X offset from base
        t.transform.translation.y = 0.0  # IMU Y offset from base
        t.transform.translation.z = 0.05  # IMU Z offset from base (5cm above)
        
        # Identity quaternion (no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)
        
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        
        Args:
            roll (float): Roll angle in radians
            pitch (float): Pitch angle in radians
            yaw (float): Yaw angle in radians
            
        Returns:
            list: Quaternion [x, y, z, w]
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0, 0, 0, 0]
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        
        return q

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()