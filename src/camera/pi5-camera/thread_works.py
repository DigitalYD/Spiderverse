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
from PyQt5.QtGui import QImage, QPixmap, QPainter
from PyQt5.QtCore import Qt, pyqtSlot, QTimer, QThread, pyqtSignal

# Try to import Numba for JIT compilation
try:
    from numba import jit
    has_numba = True
    print("Numba JIT compilation available")
except ImportError:
    has_numba = False
    print("Numba not available, using standard processing")

# Define Numba-accelerated functions if available
if has_numba:
    @jit(nopython=True, fastmath=True, cache=True)  # Using cache=True for faster startup after first run
    def yuv_to_rgb_numba(yuv_image):
        """Optimized YUV to RGB conversion with Numba"""
        height, width = yuv_image.shape[:2]
        rgb = np.empty((height, width, 3), dtype=np.uint8)
        
        for y in range(height):
            for x in range(0, width, 2):
                if x+1 < width:
                    y0 = float(yuv_image[y, x, 0])
                    u0 = float(yuv_image[y, x, 1])
                    y1 = float(yuv_image[y, x+1, 0])
                    v0 = float(yuv_image[y, x+1, 1])
                    
                    # YUV to RGB conversion
                    r0 = y0 + 1.402 * (v0 - 128.0)
                    g0 = y0 - 0.344 * (u0 - 128.0) - 0.714 * (v0 - 128.0)
                    b0 = y0 + 1.772 * (u0 - 128.0)
                    
                    r1 = y1 + 1.402 * (v0 - 128.0)
                    g1 = y1 - 0.344 * (u0 - 128.0) - 0.714 * (v0 - 128.0)
                    b1 = y1 + 1.772 * (u0 - 128.0)
                    
                    # Clamp and assign
                    rgb[y, x, 0] = max(0, min(255, r0))
                    rgb[y, x, 1] = max(0, min(255, g0))
                    rgb[y, x, 2] = max(0, min(255, b0))
                    
                    rgb[y, x+1, 0] = max(0, min(255, r1))
                    rgb[y, x+1, 1] = max(0, min(255, g1))
                    rgb[y, x+1, 2] = max(0, min(255, b1))
        
        return rgb

class ImageProcessThread(QThread):
    processed = pyqtSignal(object)  # Signal emits processed QImage
    
    def __init__(self):
        super().__init__()
        self.cv_image = None
        self.mutex = threading.Lock()
        self.condition = threading.Condition(self.mutex)
        self.running = True
    
    def process_image(self, cv_image):
        with self.mutex:
            self.cv_image = cv_image
            self.condition.notify()
    
    def stop(self):
        with self.mutex:
            self.running = False
            self.condition.notify()
        self.wait()
    
    def run(self):
        while self.running:
            cv_image = None
            with self.mutex:
                while self.running and self.cv_image is None:
                    self.condition.wait(timeout=1.0)
                if not self.running:
                    break
                cv_image = self.cv_image
                self.cv_image = None
            
            if cv_image is not None:
                try:
                    # Get the dimensions
                    height, width = cv_image.shape[:2]
                    
                    # Choose conversion method based on available acceleration
                    if has_numba and cv_image.shape[2] == 2:  # Use Numba if available for YUV422
                        rgb_image = yuv_to_rgb_numba(cv_image)
                    else:
                        # Fall back to OpenCV
                        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2RGB_YUY2)
                    
                    # Create QImage with RGB32 format for better performance
                    bytes_per_line = rgb_image.strides[0]
                    q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
                    q_image = q_image.convertToFormat(QImage.Format_RGB32)
                    
                    # Emit the processed image
                    self.processed.emit(q_image)
                    
                    # Explicit memory management
                    del rgb_image
                except Exception as e:
                    print(f"Error processing image in thread: {str(e)}")

class PredrawPixmapLabel(QLabel):
    """Custom QLabel that pre-allocates pixmap buffers for better performance"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.buffer_pixmap = None
        self.display_pixmap = None
        self.setMinimumSize(400, 300)
        self.setMaximumSize(700, 800)
    
    def setPixmap(self, pixmap):
        """Override to use double-buffering for smoother updates"""
        self.display_pixmap = pixmap
        super().setPixmap(pixmap)
    
    def updateImage(self, qimage):
        """Efficient image update using pre-allocated pixmaps and double buffering"""
        if self.buffer_pixmap is None or self.buffer_pixmap.size() != qimage.size():
            self.buffer_pixmap = QPixmap(qimage.size())
            self.display_pixmap = QPixmap(qimage.size())
        
        # Paint to the buffer
        self.buffer_pixmap.fill(Qt.black)
        painter = QPainter(self.buffer_pixmap)
        painter.drawImage(0, 0, qimage)
        painter.end()
        
        # Swap buffers
        temp = self.display_pixmap
        self.display_pixmap = self.buffer_pixmap
        self.buffer_pixmap = temp
        
        # Display the new buffer
        super().setPixmap(self.display_pixmap)

class ImageViewerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Image Viewer")
        self.resize(800, 600)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Image display with optimized label
        self.image_frame = PredrawPixmapLabel()
        self.image_frame.setFrameStyle(QFrame.StyledPanel)
        self.image_frame.setAlignment(Qt.AlignCenter)
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
        
        # Create image processing thread
        self.process_thread = ImageProcessThread()
        self.process_thread.processed.connect(self.update_image)
        self.process_thread.start()
        
        # Internal variables
        self.bridge = CvBridge()
        self.node = None
        self.subscription = None
        self.is_connected = False
        self.ros_thread = None
        
        # FPS calculation - moving average approach
        self.frame_times = []
        self.max_frame_history = 30  # Number of frames to calculate average FPS
        
        # Use the main event loop's timer
        self.fps_timer = QTimer(self)
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps_timer.start(500)  # Update FPS display every 500ms
    
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
        
        # Set status before starting thread
        self.status_bar.showMessage(f"Connecting to {topic}...")
        self.connect_button.setText("Disconnect")
        self.is_connected = True
            
        # Start ROS node in a separate thread
        self.ros_thread = threading.Thread(target=self._start_ros_node, args=(topic,), daemon=True)
        self.ros_thread.start()
    
    def _start_ros_node(self, topic):
        try:
            # Initialize ROS2 node
            rclpy.init()
            self.node = ImageSubscriber(self.image_callback)
            
            # Subscribe to the image topic
            self.node.subscribe_to_topic(topic)
            
            # Update UI from the main thread safely
            QTimer.singleShot(100, lambda: self.status_bar.showMessage(f"Connected to {topic}"))
            
            # Start spinning the node
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS2 connection error: {str(e)}")
            # Update UI from the main thread safely
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
    def update_image(self, q_image):
        """Updates the UI with the processed image"""
        try:
            # Record timestamp for FPS calculation
            current_time = time.time()
            self.frame_times.append(current_time)
            
            # Keep only recent frame times for moving average
            if len(self.frame_times) > self.max_frame_history:
                self.frame_times.pop(0)
            
            # Use optimized image update
            self.image_frame.updateImage(q_image)
            
        except Exception as e:
            print(f"Error updating image: {str(e)}")
            self.status_bar.showMessage(f"Error: {str(e)}")
    
    def image_callback(self, cv_image):
        # Pass image to processing thread
        self.process_thread.process_image(cv_image)
    
    def closeEvent(self, event):
        # Stop the processing thread
        self.process_thread.stop()
        # Disconnect from ROS
        self.disconnect_from_ros()
        event.accept()


class ImageSubscriber(Node):
    def __init__(self, callback):
        super().__init__('image_viewer_node')
        self.bridge = CvBridge()
        self.callback = callback
        self.subscription = None
    
    def subscribe_to_topic(self, topic_name):
        # Create QoS profile for best performance
        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        
        # Subscribe to raw image topic
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            qos
        )
        self.get_logger().info(f'Subscribed to topic: {topic_name}')
    
    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Pass to the callback
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