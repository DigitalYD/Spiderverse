#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import time

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        
        # Create the transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create a timer to publish transforms
        self.timer = self.create_timer(0.1, self.publish_transforms)
        
        self.get_logger().info('TF broadcaster started')
        
    def publish_transforms(self):
        """Publish the necessary transforms for the robot"""
        # Get current time
        current_time = self.get_clock().now()
        
        # Publish base_link -> lidar_link transform
        self.publish_base_to_lidar_transform(current_time)
        
        # Publish odom -> base_link transform
        self.publish_odom_to_base_transform(current_time)
        
    def publish_base_to_lidar_transform(self, time_stamp):
        """Publish the base_link -> lidar_link transform"""
        t = TransformStamped()
        
        # Fill in header
        t.header.stamp = time_stamp.to_msg()
        t.header.frame_id = 'base_link'
        
        # Fill in child frame
        t.child_frame_id = 'lidar_link'
        
        # Fill in transform (identity transform - no offset)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Identity quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)
        
    def publish_odom_to_base_transform(self, time_stamp):
        """Publish the odom -> base_link transform"""
        t = TransformStamped()
        
        # Fill in header
        t.header.stamp = time_stamp.to_msg()
        t.header.frame_id = 'odom'
        
        # Fill in child frame
        t.child_frame_id = 'base_link'
        
        # Fill in transform (identity transform - no offset)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Identity quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)

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
