#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import threading
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, 
                            QLineEdit, QPushButton, QWidget, QStatusBar, QFrame)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, pyqtSlot, QTimer

class ImageViewerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Image Viewer")
        self.resize(800, 600)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Image display
        self.image_frame = QLabel()
        self.image_frame.setFrameStyle(QFrame.StyledPanel)
        self.image_frame.setAlignment(Qt.AlignCenter)
        self.image_frame.setMinimumSize(400, 300)
        self.image_frame.setMaximumSize(700, 800)
        main_layout.addWidget(self.image_frame)
        
        # Control panel
        control_layout = QHBoxLayout()
        
        # Topic entry
        control_layout.addWidget(QLabel("Topic:"))
        self.topic_entry = QLineEdit("/image_raw")
        self.topic_entry.setMinimumWidth(300)
        control_layout.addWidget(self.topic_entry)
        
        # Connect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        control_layout.addWidget(self.connect_button)
        
        # FPS display
        self.fps_label = QLabel("FPS: 0.0")
        control_layout.addWidget(self.fps_label)
        
        # Add control panel to main layout
        main_layout.addLayout(control_layout)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Not connected to ROS2")
        
        # Internal variables
        self.bridge = CvBridge()
        self.node = None
        self.subscription = None
        self.is_connected = False
        
        # FPS calculation - moving average approach
        self.frame_times = []
        self.max_frame_history = 30  # Number of frames to calculate average FPS
        self.fps_update_timer = QTimer()
        self.fps_update_timer.timeout.connect(self.update_fps)
        self.fps_update_timer.start(500)  # Update FPS display every 500ms
    
    def toggle_connection(self):
        if not self.is_connected:
            self.connect_to_ros()
        else:
            self.disconnect_from_ros()
    
    def connect_to_ros(self):
        topic = self.topic_entry.text()
        if not topic:
            self.status_bar.showMessage("Error: Please enter a valid topic name")
            return
            
        # Start ROS node in a separate thread
        self.status_bar.showMessage(f"Connecting to {topic}...")
        threading.Thread(target=self._start_ros_node, args=(topic,), daemon=True).start()
        
        self.connect_button.setText("Disconnect")
        self.is_connected = True
    
    def _start_ros_node(self, topic):
        try:
            # Print debug info
            print(f"Attempting to connect to topic: {topic}")
            
            # Initialize ROS2 node
            rclpy.init()
            self.node = ImageSubscriber(self.image_callback)
            
            # Subscribe to the image topic
            self.node.subscribe_to_topic(topic)
            
            # Update UI from the main thread
            QTimer.singleShot(100, lambda: self.status_bar.showMessage(f"Connected to {topic}"))
            
            # Start spinning the node
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS2 connection error: {str(e)}")
            # Update UI from the main thread
            QTimer.singleShot(0, lambda: self.status_bar.showMessage(f"Error: {str(e)}"))
            QTimer.singleShot(0, self.disconnect_from_ros)
    
    def disconnect_from_ros(self):
        if self.node:
            self.node.destroy_node()
            rclpy.shutdown()
            self.node = None
        
        self.is_connected = False
        self.connect_button.setText("Connect")
        self.status_bar.showMessage("Disconnected from ROS2")
    
    def update_fps(self):
        """Calculate and update FPS display using moving average"""
        if not self.frame_times:
            return
            
        # Calculate FPS based on frame time differences
        if len(self.frame_times) >= 2:
            # Calculate differences between consecutive timestamps
            time_diffs = [self.frame_times[i] - self.frame_times[i-1] 
                         for i in range(1, len(self.frame_times))]
            
            if time_diffs:
                # Average time between frames
                avg_time_between_frames = sum(time_diffs) / len(time_diffs)
                if avg_time_between_frames > 0:
                    fps = 1.0 / avg_time_between_frames
                    self.fps_label.setText(f"FPS: {fps:.1f}")
    
    @pyqtSlot(object)
    def image_callback(self, cv_image):
        try:
            # Record timestamp for FPS calculation
            current_time = time.time()
            self.frame_times.append(current_time)
            
            # Keep only recent frame times for moving average
            if len(self.frame_times) > self.max_frame_history:
                self.frame_times.pop(0)
            
            # Process the image for display
            height, width = cv_image.shape[:2]
            
            # Performance optimization: Convert directly to RGB format for display
            try:
                # Convert YUV422 to RGB using OpenCV's optimized function
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2RGB_YUY2)
                
                # Create QImage (use RGB888 format which is more compatible)
                bytes_per_line = rgb_image.strides[0]
                q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
                
                # Performance optimization: Pre-allocate pixmap size
                pixmap = QPixmap.fromImage(q_image)
                
                # Update image in UI (directly set without scaling for better performance)
                self.image_frame.setPixmap(pixmap)
                
            except Exception as e:
                print(f"Error in YUV conversion: {str(e)}")
                # Fallback to grayscale display if YUV conversion fails
                if len(cv_image.shape) == 3 and cv_image.shape[2] >= 1:
                    # Just use the Y channel (first channel) for grayscale display
                    gray = cv_image[:,:,0]
                    q_image = QImage(gray.data, width, height, gray.strides[0], QImage.Format_Grayscale8)
                    pixmap = QPixmap.fromImage(q_image)
                    self.image_frame.setPixmap(pixmap)
            
        except Exception as e:
            print(f"Error in image callback: {str(e)}")
            self.status_bar.showMessage(f"Error: {str(e)}")
    
    def closeEvent(self, event):
        self.disconnect_from_ros()
        event.accept()


class ImageSubscriber(Node):
    def __init__(self, callback):
        super().__init__('image_viewer_node')
        self.bridge = CvBridge()
        self.callback = callback
        self.subscription = None
    
    def subscribe_to_topic(self, topic_name):
        # Create the subscription to the topic
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            rclpy.qos.QoSProfile(
                depth=1,                          # Only keep latest message
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # Allow dropping messages
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,            # Only keep latest
                durability=rclpy.qos.DurabilityPolicy.VOLATILE        # No need to persist
            ))
    
    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Process the image in the main thread
            if cv_image is not None:
                self.callback(cv_image)
                
        except Exception as e:
            print(f"Error in listener_callback: {str(e)}")
            self.get_logger().error(f'Error processing image: {str(e)}')


def main():
    app = QApplication(sys.argv)
    viewer = ImageViewerGUI()
    viewer.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()