#!/usr/bin/env python3

import sys
import os
import signal
import multiprocessing
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QCoreApplication
from gui_window import MainWindow

# Initialize ROS before Qt to avoid threading issues
# This works around the QSocketNotifier error
try:
    import rclpy
    # Tell ROS to use its own threading model rather than Qt's
    os.environ['PYTHONUNBUFFERED'] = '1'
    # Ensure we're using a ROS-compatible threading/event loop model
    if not rclpy.ok():
        rclpy.init(args=None)
    ROS_INITIALIZED = True
    print("ROS initialized successfully")
except ImportError:
    ROS_INITIALIZED = False
    print("ROS not available, running in demo mode")
except Exception as e:
    ROS_INITIALIZED = False
    print(f"Error initializing ROS: {e}, running in demo mode")

# Handle SIGINT (Ctrl+C) gracefully
def signal_handler(sig, frame):
    print("\nCtrl+C pressed, exiting gracefully...")
    QApplication.quit()
    
signal.signal(signal.SIGINT, signal_handler)

def main():
    # Use QCoreApplication.setAttribute before creating QApplication
    # This can help with threading issues
    QCoreApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QCoreApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    # Create application
    app = QApplication(sys.argv)
    
    # Set application name and organization
    app.setApplicationName("Pi5 Dashboard")
    app.setOrganizationName("Pi5Projects")
    
    # Create main window
    window = MainWindow()
    
    # Now show the window
    window.show()
    
    # Set up cleanup for when app finishes
    def cleanup():
        # Clean shutdown
        if 'ROS_INITIALIZED' in globals() and ROS_INITIALIZED and rclpy.ok():
            print("Shutting down ROS...")
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"Error shutting down ROS: {e}")
    
    # Connect cleanup to aboutToQuit signal
    # app.aboutToQuit.connect(cleanup)
    
    # Start event loop and exit cleanly
    try:
        return_code = app.exec_()
        sys.exit(return_code)
    except Exception as e:
        print(f"Error in application event loop: {e}")
        cleanup()
        sys.exit(1)

if __name__ == "__main__":
    # Use the "spawn" start method to avoid threading issues on Unix/Linux
    # This ensures child processes don't inherit thread states from parent
    try:
        multiprocessing.set_start_method('spawn')
    except RuntimeError:
        # Method may already be set
        pass
    
    main()