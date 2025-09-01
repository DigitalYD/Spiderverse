#!/usr/bin/env python3

import sys
import os
import json
import time
import math
import signal

# Check if we have ROS
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from sensor_msgs.msg import LaserScan
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("ROS libraries not available, exiting.")
    sys.exit(1)

class LidarDataServer(Node):
    """Node that receives LaserScan messages and writes them to a file for the GUI to read"""
    
    def __init__(self, output_file, topic='/scan'):
        super().__init__('lidar_data_server')
        
        self.output_file = output_file
        self.topic = topic
        self.running = True
        
        # Scan data buffer for smoothing (helps reduce jitter)
        self.last_scan = None
        self.scan_buffer = []
        self.buffer_size = 3  # Number of scans to average
        self.last_write_time = 0
        self.write_interval = 0.2  # Increase interval to reduce file I/O 
        self.last_data = None  # Cache previous data to avoid unnecessary writes
        
        # Create or clear output file
        with open(self.output_file, 'w') as f:
            f.write("")
            
        # Create QoS profile for reliable scan data
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to the scan topic
        self.scan_subscription = self.create_subscription(
            LaserScan,
            topic,
            self.scan_callback,
            qos_profile
        )
        
        self.get_logger().info(f'LiDAR data server started, listening on {topic} topic')
        self.get_logger().info(f'Writing data to {output_file}')
        
    def scan_callback(self, scan_msg):
        """Process incoming LaserScan message and write to file"""
        if not self.running:
            return
        
        # Process valid ranges
        valid_ranges = []
        valid_angles = []
        
        for i, r in enumerate(scan_msg.ranges):
            if not math.isnan(r) and not math.isinf(r):
                angle = scan_msg.angle_min + (i * scan_msg.angle_increment)
                valid_ranges.append(float(r))
                valid_angles.append(float(angle))
        
        # Store processed scan in buffer
        processed_scan = {
            'timestamp': time.time(),
            'angle_min': float(scan_msg.angle_min),
            'angle_max': float(scan_msg.angle_max),
            'angle_increment': float(scan_msg.angle_increment),
            'range_min': float(scan_msg.range_min),
            'range_max': float(scan_msg.range_max),
            'ranges': valid_ranges,
            'angles': valid_angles
        }
        
        self.scan_buffer.append(processed_scan)
        
        # Keep buffer at desired size
        while len(self.scan_buffer) > self.buffer_size:
            self.scan_buffer.pop(0)
        
        # Only write at certain intervals to reduce file I/O
        current_time = time.time()
        if current_time - self.last_write_time >= self.write_interval and len(self.scan_buffer) > 0:
            self.last_write_time = current_time
            self.write_averaged_scan()
    
    def write_averaged_scan(self):
        """Write averaged scan data to file to reduce jitter"""
        if not self.scan_buffer:
            return
            
        # If we only have one scan, just use it
        if len(self.scan_buffer) == 1:
            data = self.scan_buffer[0]
        else:
            # Average multiple scans
            # First, organize points by angle
            angle_to_ranges = {}
            
            # Collect all points from all scans
            for scan in self.scan_buffer:
                for i, angle in enumerate(scan['angles']):
                    # Round angle to a reasonable precision to group nearby angles
                    angle_key = round(angle, 3)
                    if angle_key not in angle_to_ranges:
                        angle_to_ranges[angle_key] = []
                    angle_to_ranges[angle_key].append(scan['ranges'][i])
            
            # Average the ranges for each angle
            avg_angles = []
            avg_ranges = []
            
            for angle in sorted(angle_to_ranges.keys()):
                ranges = angle_to_ranges[angle]
                if ranges:
                    avg_angles.append(float(angle))
                    avg_ranges.append(float(sum(ranges) / len(ranges)))
            
            # Create the averaged scan
            if self.scan_buffer:
                last_scan = self.scan_buffer[-1]
                data = {
                    'timestamp': time.time(),
                    'angle_min': last_scan['angle_min'],
                    'angle_max': last_scan['angle_max'],
                    'angle_increment': last_scan['angle_increment'],
                    'range_min': last_scan['range_min'],
                    'range_max': last_scan['range_max'],
                    'ranges': avg_ranges,
                    'angles': avg_angles
                }
            else:
                return
        
        # Check if data is significantly different from last write to avoid unnecessary I/O
        if self.last_data is not None:
            # Only compare ranges arrays as they're the most important part
            if len(data['ranges']) == len(self.last_data['ranges']):
                # Calculate difference percentage
                diff_count = 0
                threshold = 0.1  # 10% difference threshold
                
                for i, r in enumerate(data['ranges']):
                    old_r = self.last_data['ranges'][i]
                    if abs(r - old_r) > threshold:
                        diff_count += 1
                
                # If less than 10% of points changed significantly, skip the write
                if diff_count < len(data['ranges']) * 0.1:
                    return
        
        # Store the current data for future comparison
        self.last_data = data
        
        # Write to file (atomic write to avoid partial reads)
        try:
            # Write to temp file then rename for atomicity
            temp_file = f"{self.output_file}.tmp"
            with open(temp_file, 'w') as f:
                json.dump(data, f)
            os.rename(temp_file, self.output_file)
        except Exception as e:
            self.get_logger().error(f"Error writing data to file: {e}")
    
    def shutdown(self):
        """Clean shutdown of the server"""
        self.running = False
        # Remove the output file when shutting down
        try:
            if os.path.exists(self.output_file):
                os.unlink(self.output_file)
            temp_file = f"{self.output_file}.tmp"
            if os.path.exists(temp_file):
                os.unlink(temp_file)
        except Exception as e:
            self.get_logger().error(f"Error removing files: {e}")

def main():
    if len(sys.argv) < 2:
        print("Usage: lidar_receiver.py <output_file> [ros_topic]")
        sys.exit(1)
    
    output_file = sys.argv[1]
    topic = sys.argv[2] if len(sys.argv) > 2 else '/scan'
    
    # Handle Ctrl+C and termination gracefully
    def signal_handler(sig, frame):
        print('Shutting down LiDAR data server...')
        if 'lidar_node' in globals():
            lidar_node.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Initialize ROS
        rclpy.init()
        
        # Create and run the node
        global lidar_node
        lidar_node = LidarDataServer(output_file, topic)
        
        rclpy.spin(lidar_node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        if 'lidar_node' in locals():
            lidar_node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()