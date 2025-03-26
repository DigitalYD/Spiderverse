#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import struct
import threading
import numpy as np
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class LidarUdpReceiver(Node):
    def __init__(self):
        super().__init__('lidar_udp_receiver')
        
        # Declare parameters
        self.declare_parameter('udp_port', 8089)
        self.declare_parameter('udp_buffer_size', 131072)  # 128KB buffer should be enough for most scans
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('min_angle', -3.14159)  # -180 degrees in radians
        self.declare_parameter('max_angle', 3.14159)   # 180 degrees in radians
        self.declare_parameter('min_range', 0.15)      # 15cm minimum range
        self.declare_parameter('max_range', 40.0)      # 40m maximum range
        
        # Get parameters
        self.udp_port = self.get_parameter('udp_port').value
        self.udp_buffer_size = self.get_parameter('udp_buffer_size').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        
        # Create QoS profile for LaserScan that's compatible with RViz2
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Change from BEST_EFFORT to RELIABLE
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Create publisher
        self.scan_publisher = self.create_publisher(LaserScan, self.scan_topic, qos_profile)
        
        
        # Initialize UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.udp_port))
        self.sock.settimeout(1.0)  # 1 second timeout for socket operations
        
        # Start UDP receiver thread
        self.running = True
        self.receiver_thread = threading.Thread(target=self.receive_udp_data)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        
        self.get_logger().info(f"LIDAR UDP receiver started on port {self.udp_port}")
        self.get_logger().info(f"Publishing LaserScan messages on topic '{self.scan_topic}'")
    
    def receive_udp_data(self):
        """Thread function to receive UDP data and publish as LaserScan messages"""
        while self.running and rclpy.ok():
            try:
                # Receive data from socket
                data, addr = self.sock.recvfrom(self.udp_buffer_size)
                self.get_logger().debug(f"Received {len(data)} bytes from {addr}")
                
                if len(data) < 12:  # Minimum size: timestamp(8) + count(4)
                    self.get_logger().warning("Received packet too small, skipping")
                    continue
                
                # Parse header
                timestamp = struct.unpack('<Q', data[0:8])[0]  # Unpack 64-bit timestamp
                point_count = struct.unpack('<I', data[8:12])[0]  # Unpack 32-bit count
                
                expected_size = 12 + (point_count * 16)
                if len(data) != expected_size:
                    self.get_logger().warning(f"Data size mismatch: expected {expected_size}, got {len(data)}")
                    continue
                
                # Parse scan points
                angles = []
                distances = []
                qualities = []
                flags = []
                
                for i in range(point_count):
                    offset = 12 + (i * 16)
                    angle = struct.unpack('<f', data[offset:offset+4])[0]
                    distance = struct.unpack('<f', data[offset+4:offset+8])[0]
                    quality = struct.unpack('<I', data[offset+8:offset+12])[0]
                    flag = struct.unpack('<I', data[offset+12:offset+16])[0]
                    
                    # Convert angle from degrees to radians
                    angle_rad = np.radians(angle)
                    
                    # Store point data
                    angles.append(angle_rad)
                    distances.append(distance)
                    qualities.append(quality)
                    flags.append(flag)
                
                # Create and publish LaserScan message
                self.publish_laser_scan(timestamp, angles, distances, qualities, flags)
                
            except socket.timeout:
                # This is normal, just try again
                continue
            except Exception as e:
                self.get_logger().error(f"Error receiving UDP data: {str(e)}")
    
    def publish_laser_scan(self, timestamp, angles, distances, qualities, flags):
        """Publish LaserScan message from parsed LIDAR data"""
        if not angles:
            return
        
        # Create LaserScan message
        scan_msg = LaserScan()
        
        # Fill header
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        
        # Sort points by angle
        sorted_indices = np.argsort(angles)
        sorted_angles = [angles[i] for i in sorted_indices]
        sorted_distances = [distances[i] for i in sorted_indices]
        
        # Fill scan parameters
        scan_msg.angle_min = self.min_angle
        scan_msg.angle_max = self.max_angle
        scan_msg.angle_increment = (self.max_angle - self.min_angle) / len(sorted_angles) if len(sorted_angles) > 1 else 0.01
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1  # Assume 10Hz scan rate
        scan_msg.range_min = self.min_range
        scan_msg.range_max = self.max_range
        
        # Create range array with NaN values (no measurement)
        ranges = [float('nan')] * int((self.max_angle - self.min_angle) / scan_msg.angle_increment)
        intensities = [0.0] * len(ranges)
        
        # Fill in actual measurements
        for i, angle in enumerate(sorted_angles):
            # Skip points outside the min/max angle range
            if angle < self.min_angle or angle > self.max_angle:
                continue
                
            # Calculate index in ranges array
            idx = int((angle - self.min_angle) / scan_msg.angle_increment)
            if 0 <= idx < len(ranges):
                # Convert mm to meters and apply range limits
                distance_m = sorted_distances[i] / 1000.0  # Convert mm to meters
                if self.min_range <= distance_m <= self.max_range:
                    ranges[idx] = distance_m
                    intensities[idx] = float(qualities[sorted_indices[i]])
        
        scan_msg.ranges = ranges
        scan_msg.intensities = intensities
        
        # Publish the LaserScan message
        self.scan_publisher.publish(scan_msg)
        self.get_logger().debug(f"Published LaserScan with {len(sorted_angles)} points")
    
    def destroy_node(self):
        self.running = False
        if self.receiver_thread.is_alive():
            self.receiver_thread.join(2.0)  # Wait up to 2 seconds for thread to finish
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarUdpReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()