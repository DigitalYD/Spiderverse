#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RealSenseStreamNode(Node):
    def __init__(self):
        super().__init__('realsense_stream_node')
        
        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        
        # Start streaming
        self.pipeline.start(config)
        
        # Create publisher
        self.publisher_ = self.create_publisher(Image, 'camera/color/image_raw', 10)
        
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Create CV bridge
        self.bridge = CvBridge()
        
    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            return
            
        color_image = np.asanyarray(color_frame.get_data())
        msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
