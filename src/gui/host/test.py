#!/usr/bin/env python3
import sys
import subprocess
import signal
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gst, GLib, GstVideo
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

# Initialize GStreamer
Gst.init(None)

class GstVideoWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Create black background
        self.setStyleSheet("background-color: black;")
        self.setMinimumSize(640, 480)

        # Create GStreamer pipeline
        self.pipeline = None
        self.bus = None
        self.window_id = None
    
    def start_pipeline(self, port=5000):
        pipeline_str = (
            f"udpsrc port={port} caps=\"application/x-rtp,encoding-name=JPEG,payload=26\" ! "
            f"rtpjpegdepay ! jpegdec ! videoconvert ! "
            f"xvimagesink name=sink"
        )
        
        # Create and start pipeline
        self.pipeline = Gst.parse_launch(pipeline_str)
        sink = self.pipeline.get_by_name("sink")
        
        if not sink:
            print("ERROR: Could not find xvimagesink in the pipeline")
            return False
            
        # Force sink to render on this widget
        self.window_id = self.winId()
        sink.set_window_handle(self.window_id)
        
        # Connect to bus for messages
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self.on_message)
        
        # Start playing
        self.pipeline.set_state(Gst.State.PLAYING)
        return True
    
    def stop_pipeline(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.bus.remove_signal_watch()
            self.pipeline = None
            self.bus = None
    
    def on_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}, {debug}")
            self.stop_pipeline()
        return True

class VideoWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Configure the window
        self.setWindowTitle("Pi5 Video Stream")
        self.setGeometry(100, 100, 800, 600)
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Video display
        self.video_widget = GstVideoWidget()
        layout.addWidget(self.video_widget)
        
        # Controls
        controls_layout = QHBoxLayout()
        
        # Start/Stop button
        self.toggle_button = QPushButton("Start Stream")
        self.toggle_button.clicked.connect(self.toggle_stream)
        controls_layout.addWidget(self.toggle_button)
        
        # Status label
        self.status_label = QLabel("Status: Idle")
        controls_layout.addWidget(self.status_label)
        
        layout.addLayout(controls_layout)
        
        # Initialize variables
        self.running = False
    
    def toggle_stream(self):
        if self.running:
            self.stop_stream()
            self.toggle_button.setText("Start Stream")
            self.status_label.setText("Status: Idle")
        else:
            self.start_stream()
            self.toggle_button.setText("Stop Stream")
            self.status_label.setText("Status: Running")
    
    def start_stream(self):
        if self.running:
            return
        
        # Start video pipeline
        if self.video_widget.start_pipeline():
            self.running = True
            print("Started video stream")
        else:
            self.status_label.setText("Status: Failed to start pipeline")
    
    def stop_stream(self):
        if not self.running:
            return
        
        print("Stopping video stream...")
        self.video_widget.stop_pipeline()
        self.running = False
    
    def closeEvent(self, event):
        """Handle window close event"""
        print("Closing window and cleaning up...")
        self.stop_stream()
        event.accept()

class GstQtApp:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.window = VideoWindow()
        self.window.show()
        
        # Setup GLib main loop
        self.timer = QTimer()
        self.timer.timeout.connect(self.process_glib)
        self.timer.start(10)
    
    def process_glib(self):
        context = GLib.MainContext.default()
        while context.pending():
            context.iteration(False)
    
    def run(self):
        sys.exit(self.app.exec_())

if __name__ == "__main__":
    # Create and run application
    app = GstQtApp()
    app.run()
