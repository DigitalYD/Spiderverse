#!/usr/bin/env python3

import socket
import json
import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Default anchor positions (x, y, z) in centimeters
ANCHOR_1_POSITION = (0, 0, 90)
ANCHOR_2_POSITION = (310, 0, 90)
ANCHOR_3_POSITION = (250, 600, 90)

# Server configuration
SERVER_IP = "0.0.0.0"  # Listen on all available interfaces
SERVER_PORT = 50000    # Port number

# Anchor addresses (example IPs and ports)
ANCHOR_IPS = [
    ("192.168.1.170", 50000),
    ("192.168.1.171", 50000),
    ("192.168.1.173", 50000)
]

class TrilaterationNode(Node):
    def __init__(self):
        super().__init__('trilateration_node')
        
        # Declare parameters
        self.declare_parameter('anchor1_pos', ANCHOR_1_POSITION)
        self.declare_parameter('anchor2_pos', ANCHOR_2_POSITION)
        self.declare_parameter('anchor3_pos', ANCHOR_3_POSITION)
        self.declare_parameter('server_ip', SERVER_IP)
        self.declare_parameter('server_port', SERVER_PORT)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('position_topic', 'trilateration_pose')
        self.declare_parameter('odometry_topic', 'trilateration_odom')
        self.declare_parameter('polling_period_ms', 100)
        self.declare_parameter('position_uncertainty', 0.5)  # Increased from 0.25 to 0.5 (50cm uncertainty)
        self.declare_parameter('use_moving_average', True)
        self.declare_parameter('moving_average_window', 5)
        self.declare_parameter('max_position_jump', 1.0)  # Maximum jump in meters
        
        # Get parameters
        self.anchor1_pos = tuple(self.get_parameter('anchor1_pos').value)
        self.anchor2_pos = tuple(self.get_parameter('anchor2_pos').value)
        self.anchor3_pos = tuple(self.get_parameter('anchor3_pos').value)
        self.server_ip = self.get_parameter('server_ip').value
        self.server_port = self.get_parameter('server_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.position_topic = self.get_parameter('position_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.polling_period_ms = self.get_parameter('polling_period_ms').value
        self.position_uncertainty = self.get_parameter('position_uncertainty').value
        self.use_moving_average = self.get_parameter('use_moving_average').value
        self.moving_average_window = self.get_parameter('moving_average_window').value
        self.max_position_jump = self.get_parameter('max_position_jump').value
        
        # Position history for moving average
        self.position_history = []
        
        # Create QoS profile for position messages
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Create publishers
        self.position_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.position_topic,
            qos_profile
        )
        
        self.odom_publisher = self.create_publisher(
            Odometry,
            self.odometry_topic,
            qos_profile
        )
        
        # Variables to store the latest distances (in cm)
        self.distance_from_anchor_1 = None
        self.distance_from_anchor_2 = None
        self.distance_from_anchor_3 = None
        self.latest_tag_position = None  # (x, y, z) in cm
        
        # Initialize UDP socket
        self.socket_connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_connection.bind((self.server_ip, self.server_port))
        self.socket_connection.settimeout(1.0)  # 1 second timeout
        
        # Start UDP receiver thread
        self.running = True
        self.receiver_thread = threading.Thread(target=self.receive_udp_data)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        
        self.get_logger().info(f"Trilateration node started, listening on {self.server_ip}:{self.server_port}")
        self.get_logger().info(f"Publishing position data on topic '{self.position_topic}'")
        self.get_logger().info(f"Publishing odometry data on topic '{self.odometry_topic}'")
        self.get_logger().info(f"Anchor 1 position: {self.anchor1_pos} cm")
        self.get_logger().info(f"Anchor 2 position: {self.anchor2_pos} cm")
        self.get_logger().info(f"Anchor 3 position: {self.anchor3_pos} cm")
        
        # Send initial polling update
        self.send_polling_update(self.polling_period_ms)
    
    def calculate_position(self, anchor1_pos, anchor2_pos, anchor3_pos, distance1, distance2, distance3):
        """
        Calculate the position of the tag using trilateration from three anchors.
        
        Args:
            anchor1_pos: Position of the first anchor (x, y, z) in cm
            anchor2_pos: Position of the second anchor (x, y, z) in cm
            anchor3_pos: Position of the third anchor (x, y, z) in cm
            distance1: Distance from first anchor in cm
            distance2: Distance from second anchor in cm
            distance3: Distance from third anchor in cm
            
        Returns:
            Tuple (x, y, z) with position coordinates in cm
        """
        # Trilateration algorithm
        A = 2*anchor2_pos[0] - 2*anchor1_pos[0]
        B = 2*anchor2_pos[1] - 2*anchor1_pos[1]
        C = distance1**2 - distance2**2 - anchor1_pos[0]**2 + anchor2_pos[0]**2 - anchor1_pos[1]**2 + anchor2_pos[1]**2
        D = 2*anchor3_pos[0] - 2*anchor2_pos[0]
        E = 2*anchor3_pos[1] - 2*anchor2_pos[1]
        F = distance2**2 - distance3**2 - anchor2_pos[0]**2 + anchor3_pos[0]**2 - anchor2_pos[1]**2 + anchor3_pos[1]**2
        
        # Check for potential division by zero or other numerical issues
        denominator = E*A - B*D
        if abs(denominator) < 1e-6:
            self.get_logger().warning("Trilateration math error: denominator near zero")
            return None
        
        x = (C*E - F*B) / denominator
        y = (C*D - A*F) / (B*D - A*E)
        z = anchor1_pos[2]  # Z coordinate remains the same as anchors
        
        return x, y, z
    
    def process_incoming_data(self, json_data):
        """
        Process the JSON data received from anchors and calculate position if possible.
        
        Args:
            json_data: JSON data containing distance measurements
        """
        try:
            # Ensure required keys exist
            if not all(k in json_data for k in ("device_address", "distance")):
                self.get_logger().warning("Invalid JSON format or missing keys.")
                return
            
            device_address = json_data.get("device_address")
            distance_str = json_data.get("distance")

            # Validate the distance format
            if not isinstance(distance_str, str) or "cm" not in distance_str:
                self.get_logger().warning(f"Invalid distance format: {distance_str}")
                return
            
            # Extract distance value (in cm)
            distance_value = float(distance_str.replace(" cm", "").strip())

            # Update respective distances based on device address
            if device_address == "10":
                self.distance_from_anchor_1 = distance_value
                self.get_logger().debug(f"Anchor 1 distance updated to: {self.distance_from_anchor_1} cm")
            elif device_address == "11":
                self.distance_from_anchor_2 = distance_value
                self.get_logger().debug(f"Anchor 2 distance updated to: {self.distance_from_anchor_2} cm")
            elif device_address == "12":
                self.distance_from_anchor_3 = distance_value
                self.get_logger().debug(f"Anchor 3 distance updated to: {self.distance_from_anchor_3} cm")

            # Calculate tag position if all distances are available
            if all(distance is not None for distance in [self.distance_from_anchor_1, 
                                                        self.distance_from_anchor_2, 
                                                        self.distance_from_anchor_3]):
                tag_position = self.calculate_position(
                    self.anchor1_pos, self.anchor2_pos, self.anchor3_pos, 
                    self.distance_from_anchor_1, self.distance_from_anchor_2, self.distance_from_anchor_3
                )

                if tag_position:
                    # Filter erratic position jumps
                    if self.latest_tag_position is not None:
                        # Calculate jump size in centimeters
                        dx = abs(tag_position[0] - self.latest_tag_position[0])
                        dy = abs(tag_position[1] - self.latest_tag_position[1])
                        dz = abs(tag_position[2] - self.latest_tag_position[2])
                        
                        # Convert max jump from meters to cm
                        max_jump_cm = self.max_position_jump * 100.0
                        
                        if dx > max_jump_cm or dy > max_jump_cm or dz > max_jump_cm:
                            self.get_logger().debug(f"Position jump too large: dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f} cm")
                            return
                    
                    # Apply moving average filter if enabled
                    if self.use_moving_average:
                        # Add position to history
                        self.position_history.append(tag_position)
                        # Keep only the most recent positions within window size
                        if len(self.position_history) > self.moving_average_window:
                            self.position_history.pop(0)
                        
                        # Calculate moving average
                        avg_x = sum(pos[0] for pos in self.position_history) / len(self.position_history)
                        avg_y = sum(pos[1] for pos in self.position_history) / len(self.position_history)
                        avg_z = sum(pos[2] for pos in self.position_history) / len(self.position_history)
                        
                        self.latest_tag_position = (avg_x, avg_y, avg_z)
                        self.get_logger().debug(f"Averaged position: ({avg_x:.2f}, {avg_y:.2f}, {avg_z:.2f}) cm")
                    else:
                        self.latest_tag_position = tag_position
                        self.get_logger().debug(f"Tag position calculated: ({tag_position[0]:.2f}, {tag_position[1]:.2f}, {tag_position[2]:.2f}) cm")
                    
                    # Publish position to ROS topics
                    self.publish_position()
                else:
                    self.get_logger().warning("No valid solution found for tag position.")
        except Exception as e:
            self.get_logger().error(f"Error processing incoming JSON data: {e}")
    
    def _set_high_velocity_uncertainty(self, odom_msg):
        """Set high uncertainty for velocity in odometry message when velocity is unknown"""
        velocity_uncertainty = 9999.0
        odom_msg.twist.covariance[0] = velocity_uncertainty
        odom_msg.twist.covariance[7] = velocity_uncertainty
        odom_msg.twist.covariance[14] = velocity_uncertainty
        odom_msg.twist.covariance[21] = velocity_uncertainty
        odom_msg.twist.covariance[28] = velocity_uncertainty
        odom_msg.twist.covariance[35] = velocity_uncertainty
    
    def publish_position(self):
        """
        Publish the latest calculated position to ROS topics.
        Converts from centimeters (internal calculation) to meters (ROS standard).
        """
        if not self.latest_tag_position:
            return
        
        # Get current timestamp
        current_time = self.get_clock().now().to_msg()
        
        # Convert from cm to meters for ROS
        x_m = self.latest_tag_position[0] / 100.0
        y_m = self.latest_tag_position[1] / 100.0
        z_m = self.latest_tag_position[2] / 100.0
        
        # Create and publish PoseWithCovarianceStamped message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.pose.position.x = x_m
        pose_msg.pose.pose.position.y = y_m
        pose_msg.pose.pose.position.z = z_m
        pose_msg.pose.pose.orientation.w = 1.0  # Default orientation (no rotation)
        
        # Set covariance (diagonal elements for x, y, z position uncertainty)
        # Higher values indicate more uncertainty - increased to favor LiDAR
        position_uncertainty = self.position_uncertainty * 3.0  # Increased significantly to rely more on LiDAR
        pose_msg.pose.covariance[0] = position_uncertainty  # x
        pose_msg.pose.covariance[7] = position_uncertainty  # y
        pose_msg.pose.covariance[14] = position_uncertainty  # z
        
        # Angular uncertainty (high value as we don't measure orientation)
        angular_uncertainty = 9999.0
        pose_msg.pose.covariance[21] = angular_uncertainty  # roll
        pose_msg.pose.covariance[28] = angular_uncertainty  # pitch
        pose_msg.pose.covariance[35] = angular_uncertainty  # yaw
        
        self.position_publisher.publish(pose_msg)
        
        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = "base_link"
        
        # Copy pose from the PoseWithCovarianceStamped message
        odom_msg.pose = pose_msg.pose
        
        # Estimate velocity if we have previous positions
        if hasattr(self, 'prev_position') and hasattr(self, 'prev_timestamp'):
            # Calculate time delta
            curr_time = self.get_clock().now()
            dt = (curr_time - self.prev_timestamp).nanoseconds / 1e9  # convert to seconds
            
            if dt > 0:
                # Calculate velocity (meters per second)
                dx = (x_m - self.prev_position[0]) / dt
                dy = (y_m - self.prev_position[1]) / dt
                dz = (z_m - self.prev_position[2]) / dt
                
                # Set velocity in twist
                odom_msg.twist.twist.linear.x = dx
                odom_msg.twist.twist.linear.y = dy
                odom_msg.twist.twist.linear.z = dz
                
                # Set reasonable uncertainty for the velocity
                velocity_uncertainty = 0.5  # 0.5 m/s uncertainty
                odom_msg.twist.covariance[0] = velocity_uncertainty  # x velocity
                odom_msg.twist.covariance[7] = velocity_uncertainty  # y velocity
                odom_msg.twist.covariance[14] = velocity_uncertainty  # z velocity
                odom_msg.twist.covariance[21] = 9999.0  # angular velocity uncertainties
                odom_msg.twist.covariance[28] = 9999.0
                odom_msg.twist.covariance[35] = 9999.0
            else:
                # If dt is too small, use high uncertainty
                self._set_high_velocity_uncertainty(odom_msg)
        else:
            # For first message, we have no velocity
            self._set_high_velocity_uncertainty(odom_msg)
        
        # Store current position and timestamp for next velocity calculation
        self.prev_position = (x_m, y_m, z_m)
        self.prev_timestamp = self.get_clock().now()
        
        self.odom_publisher.publish(odom_msg)
    
    def send_polling_update(self, polling_period_ms):
        """
        Send polling period update to all anchors.
        
        Args:
            polling_period_ms: Polling period in milliseconds
        """
        polling_message = json.dumps({"polling_period": polling_period_ms})
        for anchor_ip, anchor_port in ANCHOR_IPS:
            try:
                self.socket_connection.sendto(polling_message.encode('utf-8'), (anchor_ip, anchor_port))
                self.get_logger().debug(f"Sent polling period update to {anchor_ip}:{anchor_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to send polling update to {anchor_ip}:{anchor_port}: {e}")
    
    def receive_udp_data(self):
        """Thread function to receive UDP data and process it"""
        while self.running and rclpy.ok():
            try:
                # Receive data from the anchors
                data, addr = self.socket_connection.recvfrom(1024)  # Buffer size is 1024 bytes
                ip, port = addr
                
                try:
                    # Decode and parse the JSON data
                    json_data = json.loads(data.decode('utf-8'))
                    self.get_logger().debug(f"Received from {ip}:{port}: {json.dumps(json_data)}")
                    
                    # Process the data
                    self.process_incoming_data(json_data)
                    
                except json.JSONDecodeError:
                    self.get_logger().warning(f"Invalid JSON received from {ip}:{port}")
            
            except socket.timeout:
                # This is normal, just try again
                continue
            except Exception as e:
                if self.running and rclpy.ok():
                    self.get_logger().error(f"Error receiving UDP data: {str(e)}")
    
    def destroy_node(self):
        """Clean up resources when node is shut down"""
        self.get_logger().info("Shutting down trilateration node...")
        self.running = False
        
        if hasattr(self, 'receiver_thread') and self.receiver_thread.is_alive():
            self.receiver_thread.join(2.0)  # Wait up to 2 seconds for thread to finish
        
        if hasattr(self, 'socket_connection'):
            self.socket_connection.close()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrilaterationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()