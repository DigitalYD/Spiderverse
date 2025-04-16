#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import threading
import time
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, 
                            QComboBox, QPushButton, QWidget, QStatusBar, QFrame, QGridLayout,
                            QTextEdit, QGroupBox, QSplitter)
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QFont
from PyQt5.QtCore import Qt, pyqtSlot, QTimer, QThread, pyqtSignal, QSize

# Try to set process priority higher for better performance
try:
    import psutil
    process = psutil.Process(os.getpid())
    process.nice(psutil.HIGH_PRIORITY_CLASS if hasattr(psutil, 'HIGH_PRIORITY_CLASS') else -15)
    print("Process priority increased")
except:
    print("Could not set process priority")

# Enable OpenCV optimizations
try:
    cv2.setUseOptimized(True)
    cv2.setNumThreads(6)  # Adjust based on your CPU
    print(f"OpenCV optimizations enabled, using {cv2.getNumThreads()} threads")
except:
    print("Could not configure OpenCV optimizations")

# Try to import Numba for JIT compilation
try:
    from numba import jit, set_num_threads
    set_num_threads(6)  # Adjust based on your CPU
    has_numba = True
    print("Numba JIT compilation available")
except ImportError:
    has_numba = False
    print("Numba not available, using standard processing")

# Define Numba-accelerated functions if available
if has_numba:
    @jit(nopython=True, fastmath=True, cache=True)
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

# Use X11 acceleration if available (Linux)
try:
    os.environ['QT_X11_NO_MITSHM'] = '1'  # Avoid MIT-SHM which can be slow
    print("X11 optimizations enabled")
except:
    pass

class PreallocatedPixmap(QPixmap):
    """Pixmap that pre-allocates memory to avoid reallocations"""
    @classmethod
    def get_pixmap(cls, size):
        if not hasattr(cls, '_pixmap_cache'):
            cls._pixmap_cache = {}
        
        # Round up to nearest multiple of 32 for better memory alignment
        width = ((size.width() + 31) // 32) * 32
        height = ((size.height() + 31) // 32) * 32
        key = (width, height)
        
        if key not in cls._pixmap_cache or cls._pixmap_cache[key].size() != QSize(width, height):
            cls._pixmap_cache[key] = QPixmap(width, height)
            
        return cls._pixmap_cache[key]

class OptimizedImageLabel(QLabel):
    """Highly optimized label for image display"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 300)
        self.setMaximumSize(700, 800)
        self.setAttribute(Qt.WA_OpaquePaintEvent, True)  # Avoid unnecessary background repaints
        self.setAttribute(Qt.WA_NoSystemBackground, True)  # Avoid system background
        self.setUpdatesEnabled(True)
        
        # Pre-create buffers
        self.current_buffer = PreallocatedPixmap.get_pixmap(QSize(640, 480))
        self.back_buffer = PreallocatedPixmap.get_pixmap(QSize(640, 480))
        self.setPixmap(self.current_buffer)
    
    def updateImage(self, qimage):
        """Double-buffered, pre-allocated image update"""
        # Check if buffer size needs to change
        if self.back_buffer.size() != qimage.size():
            self.back_buffer = PreallocatedPixmap.get_pixmap(qimage.size())
            self.current_buffer = PreallocatedPixmap.get_pixmap(qimage.size())
        
        # Paint to back buffer
        painter = QPainter(self.back_buffer)
        painter.drawImage(0, 0, qimage)
        painter.end()
        
        # Swap buffers
        temp = self.current_buffer
        self.current_buffer = self.back_buffer
        self.back_buffer = temp
        
        # Update label
        self.setPixmap(self.current_buffer)
        
        # Force immediate update instead of waiting for event loop
        self.repaint()

class LidarVisualizer(QLabel):
    """Widget for visualizing LiDAR data"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)
        self.setMaximumSize(400, 400)
        self.scan_data = None
        self.setAttribute(Qt.WA_OpaquePaintEvent, True)
        
        # Create buffer
        self.pixmap = QPixmap(300, 300)
        self.pixmap.fill(Qt.black)
        self.setPixmap(self.pixmap)
    
    def update_scan(self, scan_data):
        self.scan_data = scan_data
        self.update_visualization()
    
    def update_visualization(self):
        if not self.scan_data:
            return
            
        # Create a new pixmap for drawing
        self.pixmap.fill(Qt.black)
        painter = QPainter(self.pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Set up coordinates
        center_x = self.pixmap.width() / 2
        center_y = self.pixmap.height() / 2
        max_radius = min(center_x, center_y) - 10
        
        # Draw the radar-like background
        painter.setPen(QColor(30, 30, 30))
        for r in range(0, int(max_radius), int(max_radius/5)):
            painter.drawEllipse(center_x - r, center_y - r, r * 2, r * 2)
        
        # Draw the coordinate lines
        painter.drawLine(center_x, 0, center_x, self.pixmap.height())
        painter.drawLine(0, center_y, self.pixmap.width(), center_y)
        
        # Draw scan data points
        painter.setPen(QColor(0, 255, 0))
        
        min_range = self.scan_data.range_min
        max_range = self.scan_data.range_max
        
        # Normalize and plot the points
        angle = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        
        for i, distance in enumerate(self.scan_data.ranges):
            if min_range <= distance <= max_range:
                # Convert polar to cartesian
                x = center_x + max_radius * (distance / max_range) * np.cos(angle)
                y = center_y + max_radius * (distance / max_range) * np.sin(angle)
                
                # Draw the point
                painter.drawPoint(int(x), int(y))
            
            angle += angle_increment
        
        painter.end()
        self.setPixmap(self.pixmap)
        self.repaint()

class ImageProcessThread(QThread):
    processed = pyqtSignal(object)  # Signal emits processed QImage
    
    def __init__(self):
        super().__init__()
        
        # Set thread priority
        self.setPriority(QThread.HighestPriority)
        
        # Image handling
        self.cv_image = None
        self.mutex = threading.Lock()
        self.condition = threading.Condition(self.mutex)
        self.running = True
        
        # Performance monitoring
        self.processing_times = []
        self.max_times = 30
        
        # Pre-allocate conversion buffer
        self.rgb_buffer = None
    
    def process_image(self, cv_image):
        with self.mutex:
            self.cv_image = cv_image
            self.condition.notify()
    
    def stop(self):
        with self.mutex:
            self.running = False
            self.condition.notify()
        self.wait()
    
    def get_avg_processing_time(self):
        if not self.processing_times:
            return 0
        return sum(self.processing_times) / len(self.processing_times)
    
    def run(self):
        while self.running:
            cv_image = None
            with self.mutex:
                while self.running and self.cv_image is None:
                    self.condition.wait(timeout=0.01)  # Shorter timeout for faster response
                if not self.running:
                    break
                cv_image = self.cv_image
                self.cv_image = None
            
            if cv_image is not None:
                try:
                    start_time = time.time()
                    
                    # Get the dimensions
                    height, width = cv_image.shape[:2]
                    
                    # Choose conversion method based on available acceleration
                    if has_numba and cv_image.shape[2] == 2:  # Use Numba if available for YUV422
                        rgb_image = yuv_to_rgb_numba(cv_image)
                    else:
                        # Fall back to OpenCV
                        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2RGB_YUY2)
                    
                    # Direct memory access for creating QImage
                    bytes_per_line = rgb_image.strides[0]
                    q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
                    
                    # Convert to optimized format with minimal copying
                    q_image = q_image.convertToFormat(QImage.Format_RGB32)
                    
                    # Emit the processed image
                    self.processed.emit(q_image)
                    
                    # Record processing time for monitoring
                    elapsed = time.time() - start_time
                    self.processing_times.append(elapsed)
                    if len(self.processing_times) > self.max_times:
                        self.processing_times.pop(0)
                    
                    # Explicit memory management
                    del rgb_image
                    
                except Exception as e:
                    print(f"Error processing image in thread: {str(e)}")

class Pi5StatsThread(QThread):
    stats_updated = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.running = True
        self.pi_address = "raspberrypi.local"  # Default Pi5 address
    
    def set_pi_address(self, address):
        self.pi_address = address
    
    def run(self):
        while self.running:
            try:
                # This would normally connect to Pi5 via SSH and get stats
                # For demonstration, we'll generate sample data
                stats = {
                    'cpu_usage': np.random.randint(0, 100),
                    'ram_usage': np.random.randint(0, 100),
                    'storage': np.random.randint(0, 100),
                    'temperature': 40 + np.random.randint(0, 20),
                    'network': {
                        'down': np.random.randint(0, 1000),
                        'up': np.random.randint(0, 500)
                    },
                    'gpu_usage': np.random.randint(0, 100)
                }
                
                self.stats_updated.emit(stats)
                
                time.sleep(1)  # Update every second
                
            except Exception as e:
                print(f"Error in Pi5 stats thread: {str(e)}")
                time.sleep(5)  # Wait longer if there's an error
    
    def stop(self):
        self.running = False
        self.wait()

class TopicDiscovery(QThread):
    topics_discovered = pyqtSignal(list)
    
    def __init__(self, msg_type=None):
        super().__init__()
        self.node = None
        self.running = True
        self.msg_type = msg_type  # If specified, filter by message type
    
    def run(self):
        try:
            # Initialize ROS2 in its own context for topic discovery
            context = rclpy.Context()
            rclpy.init(context=context)
            self.node = rclpy.create_node('topic_discovery_node', context=context)
            
            while self.running:
                # Get all topics
                topic_names_and_types = self.node.get_topic_names_and_types()
                
                # Filter for specific message type if requested
                filtered_topics = []
                for name, types in topic_names_and_types:
                    for type_name in types:
                        if self.msg_type is None or self.msg_type in type_name:
                            filtered_topics.append(name)
                
                # Emit the topics
                self.topics_discovered.emit(filtered_topics)
                
                # Sleep to avoid high CPU usage
                time.sleep(2)
                
        except Exception as e:
            print(f"Error in topic discovery: {str(e)}")
        finally:
            if self.node:
                self.node.destroy_node()
                rclpy.shutdown(context=context)
    
    def stop(self):
        self.running = False
        self.wait()

class ImageViewerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Multi-Sensor Viewer")
        self.resize(1200, 800)
        
        # Central widget with splitter
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        main_layout = QHBoxLayout(self.central_widget)
        
        # Create splitter for resizable sections
        self.splitter = QSplitter(Qt.Horizontal)
        
        # === LEFT SIDE: CAMERA FEED ===
        self.camera_widget = QWidget()
        camera_layout = QVBoxLayout(self.camera_widget)
        
        # Image display with optimized label
        self.image_frame = OptimizedImageLabel()
        self.image_frame.setFrameStyle(QFrame.StyledPanel)
        self.image_frame.setAlignment(Qt.AlignCenter)
        camera_layout.addWidget(self.image_frame)
        
        # Camera control panel
        control_layout = QHBoxLayout()
        
        # Topic dropdown
        control_layout.addWidget(QLabel("Camera Topic:"))
        self.camera_topic_combo = QComboBox()
        self.camera_topic_combo.setMinimumWidth(200)
        self.camera_topic_combo.setEditable(True)
        self.camera_topic_combo.addItem("/image_raw")  # Default topic
        control_layout.addWidget(self.camera_topic_combo)
        
        # Connect button
        self.camera_connect_button = QPushButton("Connect")
        self.camera_connect_button.clicked.connect(self.toggle_camera_connection)
        control_layout.addWidget(self.camera_connect_button)
        
        # Refresh topics button
        self.camera_refresh_button = QPushButton("Refresh")
        self.camera_refresh_button.clicked.connect(lambda: self.manual_refresh_topics("camera"))
        control_layout.addWidget(self.camera_refresh_button)
        
        # FPS display
        self.fps_label = QLabel("FPS: 0.0")
        control_layout.addWidget(self.fps_label)
        
        camera_layout.addLayout(control_layout)
        
        # === RIGHT SIDE: SENSORS WIDGET ===
        self.sensors_widget = QWidget()
        sensors_layout = QVBoxLayout(self.sensors_widget)
        
        # IMU Data Section
        self.imu_group = QGroupBox("IMU Data")
        imu_layout = QVBoxLayout(self.imu_group)
        
        # IMU topic selector
        imu_topic_layout = QHBoxLayout()
        imu_topic_layout.addWidget(QLabel("IMU Topic:"))
        self.imu_topic_combo = QComboBox()
        self.imu_topic_combo.setMinimumWidth(200)
        self.imu_topic_combo.setEditable(True)
        self.imu_topic_combo.addItem("/imu")  # Default topic
        imu_topic_layout.addWidget(self.imu_topic_combo)
        
        self.imu_connect_button = QPushButton("Connect")
        self.imu_connect_button.clicked.connect(self.toggle_imu_connection)
        imu_topic_layout.addWidget(self.imu_connect_button)
        
        self.imu_refresh_button = QPushButton("Refresh")
        self.imu_refresh_button.clicked.connect(lambda: self.manual_refresh_topics("imu"))
        imu_topic_layout.addWidget(self.imu_refresh_button)
        
        imu_layout.addLayout(imu_topic_layout)
        
        # IMU data display
        self.imu_data = QTextEdit()
        self.imu_data.setReadOnly(True)
        self.imu_data.setMinimumHeight(150)
        self.imu_data.setFont(QFont("Courier New", 10))
        imu_layout.addWidget(self.imu_data)
        
        sensors_layout.addWidget(self.imu_group)
        
        # Pi5 System Stats
        self.pi5_group = QGroupBox("Raspberry Pi 5 Stats")
        pi5_layout = QVBoxLayout(self.pi5_group)
        
        # Pi5 stats display
        self.pi5_stats = QTextEdit()
        self.pi5_stats.setReadOnly(True)
        self.pi5_stats.setMinimumHeight(180)
        self.pi5_stats.setFont(QFont("Courier New", 10))
        pi5_layout.addWidget(self.pi5_stats)
        
        sensors_layout.addWidget(self.pi5_group)
        
        # LiDAR Data Display
        self.lidar_group = QGroupBox("LiDAR Data")
        lidar_layout = QVBoxLayout(self.lidar_group)
        
        # LiDAR topic selector
        lidar_topic_layout = QHBoxLayout()
        lidar_topic_layout.addWidget(QLabel("LiDAR Topic:"))
        self.lidar_topic_combo = QComboBox()
        self.lidar_topic_combo.setMinimumWidth(200)
        self.lidar_topic_combo.setEditable(True)
        self.lidar_topic_combo.addItem("/scan")  # Default topic
        lidar_topic_layout.addWidget(self.lidar_topic_combo)
        
        self.lidar_connect_button = QPushButton("Connect")
        self.lidar_connect_button.clicked.connect(self.toggle_lidar_connection)
        lidar_topic_layout.addWidget(self.lidar_connect_button)
        
        self.lidar_refresh_button = QPushButton("Refresh")
        self.lidar_refresh_button.clicked.connect(lambda: self.manual_refresh_topics("lidar"))
        lidar_topic_layout.addWidget(self.lidar_refresh_button)
        
        lidar_layout.addLayout(lidar_topic_layout)
        
        # LiDAR visualization
        self.lidar_view = LidarVisualizer()
        lidar_layout.addWidget(self.lidar_view)
        
        sensors_layout.addWidget(self.lidar_group)
        
        # Add widgets to splitter
        self.splitter.addWidget(self.camera_widget)
        self.splitter.addWidget(self.sensors_widget)
        
        # Set default splitter sizes (60% left, 40% right)
        self.splitter.setSizes([600, 400])
        
        # Add splitter to main layout
        main_layout.addWidget(self.splitter)
        
        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready - Not connected to ROS2")
        
        # Create threads
        self.process_thread = ImageProcessThread()
        self.process_thread.processed.connect(self.update_image)
        self.process_thread.start()
        
        self.pi5_stats_thread = Pi5StatsThread()
        self.pi5_stats_thread.stats_updated.connect(self.update_pi5_stats)
        self.pi5_stats_thread.start()
        
        # Create topic discovery threads
        self.camera_discovery_thread = TopicDiscovery(msg_type='sensor_msgs/msg/Image')
        self.camera_discovery_thread.topics_discovered.connect(lambda topics: self.update_topics(topics, "camera"))
        self.camera_discovery_thread.start()
        
        self.imu_discovery_thread = TopicDiscovery(msg_type='sensor_msgs/msg/Imu')
        self.imu_discovery_thread.topics_discovered.connect(lambda topics: self.update_topics(topics, "imu"))
        self.imu_discovery_thread.start()
        
        self.lidar_discovery_thread = TopicDiscovery(msg_type='sensor_msgs/msg/LaserScan')
        self.lidar_discovery_thread.topics_discovered.connect(lambda topics: self.update_topics(topics, "lidar"))
        self.lidar_discovery_thread.start()
        
        # Node and subscriber variables
        self.camera_node = None
        self.imu_node = None
        self.lidar_node = None
        self.camera_thread = None
        self.imu_thread = None
        self.lidar_thread = None
        
        # FPS calculation
        self.frame_times = []
        self.max_frame_history = 30
        
        # Timers
        self.fps_timer = QTimer(self)
        self.fps_timer.timeout.connect(self.update_fps)
        self.fps_timer.start(500)
        
        self.stats_timer = QTimer(self)
        self.stats_timer.timeout.connect(self.update_stats)
        self.stats_timer.start(1000)
    
    def update_topics(self, topics, sensor_type):
        """Update the topic dropdowns based on discovered topics"""
        if sensor_type == "camera":
            combo_box = self.camera_topic_combo
        elif sensor_type == "imu":
            combo_box = self.imu_topic_combo
        elif sensor_type == "lidar":
            combo_box = self.lidar_topic_combo
        else:
            return
            
        current_text = combo_box.currentText()
        
        # Block signals to avoid triggering events while updating
        combo_box.blockSignals(True)
        
        # Clear and repopulate
        combo_box.clear()
        for topic in sorted(topics):
            combo_box.addItem(topic)
        
        # Restore current text if it exists
        index = combo_box.findText(current_text)
        if index >= 0:
            combo_box.setCurrentIndex(index)
        elif topics:
            combo_box.setCurrentIndex(0)
        else:
            combo_box.setEditText(current_text)
        
        # Unblock signals
        combo_box.blockSignals(False)
    
    def manual_refresh_topics(self, sensor_type):
        """Manually refresh the topic list"""
        self.status_bar.showMessage(f"Refreshing {sensor_type} topic list...")
        # The discovery thread will update automatically
    
    def update_stats(self):
        """Update processing statistics"""
        proc_time = self.process_thread.get_avg_processing_time() * 1000  # Convert to ms
        self.status_bar.showMessage(f"Processing time: {proc_time:.1f}ms")
    
    def toggle_camera_connection(self):
        """Toggle camera connection"""
        if not self.camera_node:
            topic = self.camera_topic_combo.currentText()
            if not topic:
                self.status_bar.showMessage("Error: Please enter a valid camera topic name")
                return
                
            self.status_bar.showMessage(f"Connecting to camera topic: {topic}...")
            self.camera_connect_button.setText("Disconnect")
            
            # Start ROS node in a separate thread
            self.camera_thread = threading.Thread(target=self._start_camera_node, args=(topic,), daemon=True)
            self.camera_thread.start()
        else:
            self._stop_camera_node()
            self.camera_connect_button.setText("Connect")
    
    def toggle_imu_connection(self):
        """Toggle IMU connection"""
        if not self.imu_node:
            topic = self.imu_topic_combo.currentText()
            if not topic:
                self.status_bar.showMessage("Error: Please enter a valid IMU topic name")
                return
                
            self.status_bar.showMessage(f"Connecting to IMU topic: {topic}...")
            self.imu_connect_button.setText("Disconnect")
            
            # Start ROS node in a separate thread
            self.imu_thread = threading.Thread(target=self._start_imu_node, args=(topic,), daemon=True)
            self.imu_thread.start()
        else:
            self._stop_imu_node()
            self.imu_connect_button.setText("Connect")
    
    def toggle_lidar_connection(self):
        """Toggle LiDAR connection"""
        if not self.lidar_node:
            topic = self.lidar_topic_combo.currentText()
            if not topic:
                self.status_bar.showMessage("Error: Please enter a valid LiDAR topic name")
                return
                
            self.status_bar.showMessage(f"Connecting to LiDAR topic: {topic}...")
            self.lidar_connect_button.setText("Disconnect")
            
            # Start ROS node in a separate thread
            self.lidar_thread = threading.Thread(target=self._start_lidar_node, args=(topic,), daemon=True)
            self.lidar_thread.start()
        else:
            self._stop_lidar_node()
            self.lidar_connect_button.setText("Connect")
    
    def _start_camera_node(self, topic):
        try:
            # Initialize ROS2 node
            rclpy.init(domain_id=None)  # Use default domain
            self.camera_node = CameraSubscriber(self.image_callback)
            
            # Subscribe to the image topic
            self.camera_node.subscribe_to_topic(topic)
            
            # Update UI from the main thread safely
            QTimer.singleShot(100, lambda: self.status_bar.showMessage(f"Connected to camera: {topic}"))
            
            # Start spinning the node
            rclpy.spin(self.camera_node)
        except Exception as e:
            print(f"ROS2 camera connection error: {str(e)}")
            # Update UI from the main thread safely
            QTimer.singleShot(0, lambda: self.status_bar.showMessage(f"Camera error: {str(e)}"))
            QTimer.singleShot(0, self._stop_camera_node)
    
    def _start_imu_node(self, topic):
        try:
            # Initialize ROS2 node
            context = rclpy.Context()
            rclpy.init(context=context)
            self.imu_node = ImuSubscriber(self.imu_callback, context=context)
            
            # Subscribe to the IMU topic
            self.imu_node.subscribe_to_topic(topic)
            
            # Update UI from the main thread safely
            QTimer.singleShot(100, lambda: self.status_bar.showMessage(f"Connected to IMU: {topic}"))
            
            # Start spinning the node
            rclpy.spin(self.imu_node, context=context)
        except Exception as e:
            print(f"ROS2 IMU connection error: {str(e)}")
            # Update UI from the main thread safely
            QTimer.singleShot(0, lambda: self.status_bar.showMessage(f"IMU error: {str(e)}"))
            QTimer.singleShot(0, self._stop_imu_node)
    
    def _start_lidar_node(self, topic):
        try:
            # Initialize ROS2 node
            context = rclpy.Context()
            rclpy.init(context=context)
            self.lidar_node = LidarSubscriber(self.lidar_callback, context=context)
            
            # Subscribe to the LiDAR topic
            self.lidar_node.subscribe_to_topic(topic)
            
           # Update UI from the main thread safely
            QTimer.singleShot(100, lambda: self.status_bar.showMessage(f"Connected to LiDAR: {topic}"))
            
            # Start spinning the node
            rclpy.spin(self.lidar_node, context=context)
        except Exception as e:
            print(f"ROS2 LiDAR connection error: {str(e)}")
            # Update UI from the main thread safely
            QTimer.singleShot(0, lambda: self.status_bar.showMessage(f"LiDAR error: {str(e)}"))
            QTimer.singleShot(0, self._stop_lidar_node)
    
    def _stop_camera_node(self):
        if self.camera_node:
            self.camera_node.destroy_node()
            rclpy.shutdown()
            self.camera_node = None
            self.status_bar.showMessage("Disconnected from camera")
    
    def _stop_imu_node(self):
        if self.imu_node:
            self.imu_node.destroy_node()
            self.imu_node = None
            self.status_bar.showMessage("Disconnected from IMU")
    
    def _stop_lidar_node(self):
        if self.lidar_node:
            self.lidar_node.destroy_node()
            self.lidar_node = None
            self.status_bar.showMessage("Disconnected from LiDAR")
    
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
    
    def image_callback(self, cv_image, stamp=None):
        # Pass image to processing thread
        self.process_thread.process_image(cv_image)
    
    def imu_callback(self, imu_msg):
        """Process IMU data and update the UI"""
        try:
            # Format IMU data for display
            text = f"Orientation (x,y,z,w): \n  {imu_msg.orientation.x:.4f}, {imu_msg.orientation.y:.4f}, "
            text += f"{imu_msg.orientation.z:.4f}, {imu_msg.orientation.w:.4f}\n\n"
            
            text += f"Angular Velocity (x,y,z): \n  {imu_msg.angular_velocity.x:.4f}, "
            text += f"{imu_msg.angular_velocity.y:.4f}, {imu_msg.angular_velocity.z:.4f}\n\n"
            
            text += f"Linear Acceleration (x,y,z): \n  {imu_msg.linear_acceleration.x:.4f}, "
            text += f"{imu_msg.linear_acceleration.y:.4f}, {imu_msg.linear_acceleration.z:.4f}\n"
            
            # Update the IMU text display
            self.imu_data.setText(text)
            
        except Exception as e:
            print(f"Error processing IMU data: {str(e)}")
    
    def lidar_callback(self, scan_msg):
        """Process LiDAR data and update the visualization"""
        try:
            # Update the LiDAR visualization
            self.lidar_view.update_scan(scan_msg)
            
        except Exception as e:
            print(f"Error processing LiDAR data: {str(e)}")
    
    @pyqtSlot(dict)
    def update_pi5_stats(self, stats):
        """Update the Pi5 system statistics display"""
        try:
            # Format stats for display
            text = f"CPU Usage: {stats['cpu_usage']}%\n"
            text += f"RAM Usage: {stats['ram_usage']}%\n"
            text += f"Storage Used: {stats['storage']}%\n"
            text += f"Temperature: {stats['temperature']}°C\n\n"
            text += f"Network:\n"
            text += f"  Download: {stats['network']['down']} KB/s\n"
            text += f"  Upload: {stats['network']['up']} KB/s\n\n"
            text += f"GPU Usage: {stats['gpu_usage']}%\n"
            
            # Update the Pi5 stats text display
            self.pi5_stats.setText(text)
            
        except Exception as e:
            print(f"Error updating Pi5 stats: {str(e)}")
    
    def closeEvent(self, event):
        # Stop all threads
        self.camera_discovery_thread.stop()
        self.imu_discovery_thread.stop()
        self.lidar_discovery_thread.stop()
        self.pi5_stats_thread.stop()
        self.process_thread.stop()
        
        # Disconnect from ROS
        self._stop_camera_node()
        self._stop_imu_node()
        self._stop_lidar_node()
        
        event.accept()


class CameraSubscriber(Node):
    def __init__(self, callback):
        super().__init__('camera_viewer_node')
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
        self.get_logger().info(f'Subscribed to camera topic: {topic_name}')
    
    def listener_callback(self, msg):
        try:
            # Convert message time to floating point seconds for latency calculation
            stamp = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
            
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Pass to the callback with timestamp
            if cv_image is not None:
                self.callback(cv_image, stamp)
                
        except Exception as e:
            print(f"Error in camera callback: {str(e)}")
            self.get_logger().error(f'Error processing image: {str(e)}')


class ImuSubscriber(Node):
    def __init__(self, callback, context=None):
        super().__init__('imu_viewer_node', context=context)
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
        
        # Subscribe to IMU topic
        self.subscription = self.create_subscription(
            Imu,
            topic_name,
            self.listener_callback,
            qos
        )
        self.get_logger().info(f'Subscribed to IMU topic: {topic_name}')
    
    def listener_callback(self, msg):
        try:
            # Pass IMU message to callback
            self.callback(msg)
        except Exception as e:
            print(f"Error in IMU callback: {str(e)}")
            self.get_logger().error(f'Error processing IMU data: {str(e)}')


class LidarSubscriber(Node):
    def __init__(self, callback, context=None):
        super().__init__('lidar_viewer_node', context=context)
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
        
        # Subscribe to LiDAR topic
        self.subscription = self.create_subscription(
            LaserScan,
            topic_name,
            self.listener_callback,
            qos
        )
        self.get_logger().info(f'Subscribed to LiDAR topic: {topic_name}')
    
    def listener_callback(self, msg):
        try:
            # Pass LiDAR message to callback
            self.callback(msg)
        except Exception as e:
            print(f"Error in LiDAR callback: {str(e)}")
            self.get_logger().error(f'Error processing LiDAR data: {str(e)}')


def main():
    # Set Qt application attributes for better performance
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    # Create application
    app = QApplication(sys.argv)
    
    # Set application-wide rendering settings
    app.setStyle('Fusion')  # Usually the fastest style
    
    # Create and show viewer
    viewer = ImageViewerGUI()
    viewer.show()
    
    # Start event loop
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()