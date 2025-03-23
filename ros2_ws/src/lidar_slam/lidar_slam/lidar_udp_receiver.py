#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class LidarUdpReceiverNode(Node):
    def __init__(self):
        super().__init__('lidar_udp_receiver')
        self.get_logger().info('LIDAR UDP Receiver started - this is a test')
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info('LIDAR UDP Receiver is running')

def main(args=None):
    rclpy.init(args=args)
    node = LidarUdpReceiverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()