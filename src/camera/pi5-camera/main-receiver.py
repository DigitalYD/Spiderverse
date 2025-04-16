# laptop_receiver_pyqt.py
import cv2
import zmq
import pickle
import numpy as np
import time
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

class VideoStreamViewer(QWidget):
    def __init__(self, zmq_address):
        super().__init__()
        
        # Set up ZeroMQ subscriber
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(zmq_address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # Configure socket for non-blocking receive
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
        
        # Set up the UI
        self.setWindowTitle("Grayscale to RGB - ZeroMQ Stream")
        self.setGeometry(100, 100, 800, 600)
        
        # Create layout and image label
        layout = QVBoxLayout()
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.image_label)
        self.setLayout(layout)
        
        # Set up timer for periodic frame updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # Update every 30 ms (approx. 33 FPS)
        
        # Statistics
        self.frame_count = 0
        self.start_time = time.time()
        
        print("PyQt5 video viewer started")
    
    def update_frame(self):
        try:
            # Receive compressed grayscale frame
            compressed_data = self.socket.recv()
            compressed_gray = pickle.loads(compressed_data)
            
            # Decompress the frame
            gray = cv2.imdecode(compressed_gray, cv2.IMREAD_GRAYSCALE)
            
            # Convert grayscale to RGB
            rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            
            # Add a colored timestamp
            timestamp = time.strftime("%H:%M:%S")
            cv2.putText(rgb, f"RGB Converted: {timestamp}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Convert to QImage for PyQt display
            height, width, channel = rgb.shape
            bytes_per_line = 3 * width
            q_img = QImage(rgb.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            
            # Display the frame
            self.image_label.setPixmap(QPixmap.fromImage(q_img).scaled(
                self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
            
            # Update statistics
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # Report every 30 frames
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed
                print(f"Received {self.frame_count} frames. Average FPS: {fps:.2f}")
                
        except zmq.error.Again:
            # No message received (timeout)
            pass
        except Exception as e:
            print(f"Error: {e}")
    
    def closeEvent(self, event):
        # Clean up when window is closed
        self.timer.stop()
        self.socket.close()
        self.context.term()
        print(f"Viewer stopped. Received {self.frame_count} frames total.")
        event.accept()

if __name__ == "__main__":
    # Parse command line args
    zmq_address = "tcp://192.168.0.115:5555"  # Default
    if len(sys.argv) > 1:
        zmq_address = sys.argv[1]
    
    app = QApplication(sys.argv)
    viewer = VideoStreamViewer(zmq_address)
    viewer.show()
    sys.exit(app.exec_())
