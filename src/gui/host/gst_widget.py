#!/usr/bin/env python3

import sys
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gst, GLib, GstVideo
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer

# Initialize GStreamer
Gst.init(None)

class GStreamerWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Initialize members
        self.pipeline = None
        self.bus = None
        self.running = False
        self.window_id = None
        
        # Set up the UI
        self.setup_ui()
        
        # Create timer for GLib main loop integration
        self.glib_timer = QTimer(self)
        self.glib_timer.timeout.connect(self.process_glib)
        self.glib_timer.start(10)  # 10ms interval
    
    def setup_ui(self):
        """Set up the user interface"""
        layout = QVBoxLayout(self)
        
        # Video display container
        self.video_container = QWidget()
        self.video_container.setStyleSheet("background-color: black;")
        self.video_container.setMinimumSize(480, 360)
        layout.addWidget(self.video_container)
        
        # Controls
        controls_layout = QHBoxLayout()
        
        # Stream control button
        self.stream_button = QPushButton("Start Stream")
        self.stream_button.clicked.connect(self.toggle_stream)
        controls_layout.addWidget(self.stream_button)
        
        # Status label
        self.status_label = QLabel("Status: Idle")
        controls_layout.addWidget(self.status_label)
        
        layout.addLayout(controls_layout)
    
    def toggle_stream(self):
        """Toggle streaming on/off"""
        if self.running:
            self.stop_stream()
            self.stream_button.setText("Start Stream")
            self.status_label.setText("Status: Idle")
        else:
            self.start_stream()
            self.stream_button.setText("Stop Stream")
            self.status_label.setText("Status: Running")
    
    def start_stream(self):
        """Start the GStreamer pipeline"""
        if self.running:
            return
        
        # Get configuration from imported config
        try:
            import config
            port = config.GSTREAMER_PORT
        except (ImportError, AttributeError):
            port = 5000  # Default if config not available
        
        # Create pipeline for receiving JPEG video
        pipeline_str = (
            f"udpsrc port={port} caps=\"application/x-rtp,encoding-name=JPEG,payload=26\" ! "
            f"rtpjpegdepay ! jpegdec ! videoconvert ! "
            f"xvimagesink name=sink"
        )
        
        # Create GStreamer pipeline
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            sink = self.pipeline.get_by_name("sink")
            
            if not sink:
                print("ERROR: Could not find xvimagesink in the pipeline")
                return
                
            # Force sink to render on our widget
            self.window_id = self.video_container.winId()
            sink.set_window_handle(self.window_id)
            
            # Connect to bus for messages
            self.bus = self.pipeline.get_bus()
            self.bus.add_signal_watch()
            self.bus.connect("message", self.on_message)
            
            # Start playing
            self.pipeline.set_state(Gst.State.PLAYING)
            self.running = True
            print(f"Started GStreamer pipeline receiving on port {port}")
            
        except Exception as e:
            print(f"Error starting GStreamer: {e}")
            self.status_label.setText(f"Error: {str(e)[:30]}")
    
    def process_glib(self):
        """Process pending GLib events (for GStreamer)"""
        context = GLib.MainContext.default()
        while context.pending():
            context.iteration(False)
    
    def on_message(self, bus, message):
        """Handle GStreamer bus messages"""
        t = message.type
        
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}, {debug}")
            self.stop_stream()
            self.stream_button.setText("Start Stream")
            self.status_label.setText(f"Error: {str(err)[:30]}")
        elif t == Gst.MessageType.EOS:
            print("End of stream")
            self.stop_stream()
            self.stream_button.setText("Start Stream")
            self.status_label.setText("Stream ended")
        
        return True
    
    def stop_stream(self):
        """Stop the GStreamer pipeline"""
        if not self.running:
            return
        
        print("Stopping GStreamer pipeline...")
        
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            if self.bus:
                self.bus.remove_signal_watch()
            self.pipeline = None
            self.bus = None
        
        self.running = False
    
    def closeEvent(self, event):
        """Handle window close event"""
        self.stop_stream()
        event.accept()
