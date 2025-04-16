#!/usr/bin/env python3

import sys
import signal
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt
from lidar_widget import LidarWidget

class LidarTestWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LiDAR Widget Test")
        self.setGeometry(100, 100, 800, 800)
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Create and add LiDAR widget
        self.lidar_widget = LidarWidget()
        layout.addWidget(self.lidar_widget)
        
        # Status bar
        self.statusBar().showMessage('LiDAR Widget Test')
    
    def closeEvent(self, event):
        # Make sure the lidar widget is cleaned up properly
        if hasattr(self, 'lidar_widget'):
            self.lidar_widget.closeEvent(event)
        event.accept()

def signal_handler(sig, frame):
    # Handle Ctrl+C gracefully
    print("Exiting application...")
    QApplication.quit()

if __name__ == "__main__":
    # Setup signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create application
    app = QApplication(sys.argv)
    
    # Create and show main window
    window = LidarTestWindow()
    window.show()
    
    # Start the application event loop
    sys.exit(app.exec_())