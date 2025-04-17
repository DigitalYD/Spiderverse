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
        self.glib_timer.start(100)  # 100ms interval - less frequent to reduce thread contention
    
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
        
        # Try loading the pipeline from config if available
        try:
            import config
            if hasattr(config, 'GSTREAMER_PIPELINE'):
                pipeline_str = config.GSTREAMER_PIPELINE
                print(f"Using pipeline from config: {pipeline_str}")
            else:
                # Create pipeline for receiving JPEG video - use autovideosink
                pipeline_str = (
                    f"udpsrc port={port} caps=\"application/x-rtp,encoding-name=JPEG,payload=26\" ! "
                    f"rtpjpegdepay ! jpegdec ! videoconvert ! "
                    f"autovideosink sync=false name=sink"
                )
        except (ImportError, AttributeError):
            # Create pipeline for receiving JPEG video
            pipeline_str = (
                f"udpsrc port={port} caps=\"application/x-rtp,encoding-name=JPEG,payload=26\" ! "
                f"rtpjpegdepay ! jpegdec ! videoconvert ! "
                f"autovideosink sync=false name=sink"
            )
        
        try:
            print(f"Creating pipeline: {pipeline_str}")
            self.pipeline = Gst.parse_launch(pipeline_str)
            
            # Connect bus before setting window handle and playing
            self.bus = self.pipeline.get_bus()
            self.bus.add_signal_watch()
            self.bus.connect("message", self.on_message)
            
            # Start pipeline in READY state first
            ret = self.pipeline.set_state(Gst.State.READY)
            if ret == Gst.StateChangeReturn.FAILURE:
                print("Failed to set pipeline to READY")
                
                # Try multiple fallback pipelines - using JPEG format to match the sender
                print("Trying fallback pipeline 1...")
                fallback_pipeline = (
                    f"udpsrc port={port} ! application/x-rtp,media=video,encoding-name=JPEG,payload=26 "
                    f"! rtpjitterbuffer ! rtpjpegdepay ! jpegdec ! videoconvert "
                    f"! autovideosink name=sink sync=false"
                )
                print(f"Fallback pipeline: {fallback_pipeline}")
                
                # Create and connect fallback pipeline
                self.pipeline = Gst.parse_launch(fallback_pipeline)
                self.bus = self.pipeline.get_bus()
                self.bus.add_signal_watch()
                self.bus.connect("message", self.on_message)
                
                ret = self.pipeline.set_state(Gst.State.READY)
                if ret == Gst.StateChangeReturn.FAILURE:
                    print("Fallback pipeline 1 failed to set READY state")
                    
                    # Try an even simpler pipeline as fallback 2 - with minimal caps
                    print("Trying fallback pipeline 2...")
                    fallback_pipeline2 = (
                        f"udpsrc port={port} ! application/x-rtp ! rtpjpegdepay ! jpegdec ! "
                        f"videoconvert ! autovideosink name=sink sync=false"
                    )
                    print(f"Fallback pipeline 2: {fallback_pipeline2}")
                    
                    self.pipeline = Gst.parse_launch(fallback_pipeline2)
                    self.bus = self.pipeline.get_bus()
                    self.bus.add_signal_watch()
                    self.bus.connect("message", self.on_message)
                    
                    ret = self.pipeline.set_state(Gst.State.READY)
                    if ret == Gst.StateChangeReturn.FAILURE:
                        print("Fallback pipeline 2 also failed to set READY state")
                        return
            
            # Just setup the fallback display for autovideosink
            # Create and display a message in the video container
            self._setup_fallback_display()
            
            # Get and print information about the sink for debugging
            sink = self.pipeline.get_by_name("sink")
            if sink:
                print(f"Sink: {sink.get_name()} of type {sink.__class__.__name__}")
                print(f"Sink factory: {sink.get_factory().get_name()}")
                
                try:
                    # Log available methods for debugging
                    methods = [m for m in dir(sink) if not m.startswith('__')]
                    print(f"Sink methods: {methods[:10]}...")  # Show first 10 to avoid log flood
                    
                    print("Using autovideosink - video will appear in a separate window")
                except Exception as e:
                    print(f"Error getting sink info: {e}")
            else:
                print("WARNING: Could not find sink element in the pipeline")
            
            # Now play the pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                print("Failed to set pipeline to PLAYING")
                return
            
            self.running = True
            print(f"Started GStreamer pipeline receiving on port {port}")
            
        except Exception as e:
            print(f"Error starting GStreamer: {e}")
            import traceback
            traceback.print_exc()
            self.status_label.setText(f"Error: {str(e)[:30]}")
    
    def _setup_fallback_display(self):
        """Setup display for separate video window"""
        self.status_label.setText("Status: Video in separate window")
        self.video_container.setStyleSheet("background-color: #333333; color: white;")
        
        # Display message in video container
        # Create a label if it doesn't exist
        if not hasattr(self, 'info_label'):
            from PyQt5.QtWidgets import QLabel
            from PyQt5.QtCore import Qt
            self.info_label = QLabel(self.video_container)
            self.info_label.setAlignment(Qt.AlignCenter)
            self.info_label.setStyleSheet("color: white; font-size: 14px;")
            self.info_label.setWordWrap(True)
            self.info_label.resize(self.video_container.size())
        
        # Show message
        msg = ("Video is streaming in a separate window.\n\n" +
               "This implementation uses a separate window to avoid X11 integration issues.\n\n" +
               "If no window appears, check that the camera is streaming correctly.")
        self.info_label.setText(msg)
        self.info_label.show()
        
        # Make sure the label resizes with the container
        self.video_container.resizeEvent = lambda event: self.info_label.resize(self.video_container.size())
        
        print("Using separate window for video display")
    
    def process_glib(self):
        """Process pending GLib events (for GStreamer)"""
        try:
            context = GLib.MainContext.default()
            while context.pending():
                context.iteration(False)
        except Exception as e:
            print(f"Error in process_glib: {e}")
            # Don't let exceptions in GLib processing crash the app
            pass
    
    def on_message(self, bus, message):
        """Handle GStreamer bus messages"""
        t = message.type
        
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}, {debug}")
            
            if "timeout" in str(err).lower() or "connection refused" in str(err).lower():
                # Connection issues - show more helpful message
                # Get port number from config if possible
                try:
                    import config
                    port = config.GSTREAMER_PORT
                except:
                    port = 5000  # Default fallback
                    
                if hasattr(self, 'info_label'):
                    self.info_label.setText(f"Connection error: {str(err)}\n\nCheck that the camera is streaming to port {port}.\nVerify the streaming device is running and connected to the network.")
                self.status_label.setText("Error: Connection issue")
            else:
                # Generic error
                if hasattr(self, 'info_label'):
                    self.info_label.setText(f"GStreamer error: {str(err)}\n\n{debug}")
                self.status_label.setText(f"Error: {str(err)[:30]}")
            
        elif t == Gst.MessageType.EOS:
            print("End of stream")
            self.stop_stream()
            self.stream_button.setText("Start Stream")
            self.status_label.setText("Stream ended")
            
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                print(f"Pipeline state changed from {Gst.Element.state_get_name(old_state)} to {Gst.Element.state_get_name(new_state)}")
                
                if new_state == Gst.State.PLAYING:
                    # Pipeline is now playing
                    if hasattr(self, 'info_label') and self.info_label.isVisible():
                        self.info_label.setText("Video stream is running...\n\nIf no video appears, check that the camera is sending data.")
                    
        elif t == Gst.MessageType.ELEMENT:
            # Check for prepare-window-handle message to set the XWindow id
            struct = message.get_structure()
            if struct and struct.has_name("prepare-window-handle"):
                print("Received prepare-window-handle message")
                event_source = message.src
                if event_source and self.window_id:
                    print(f"Setting window handle: {self.window_id}")
                    try:
                        event_source.set_window_handle(self.window_id)
                    except Exception as e:
                        print(f"Error setting window handle: {e}")
        
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
        
        # Hide the info label if it exists
        if hasattr(self, 'info_label') and self.info_label.isVisible():
            self.info_label.hide()
            
        # Reset video container style
        self.video_container.setStyleSheet("background-color: black;")
    
    def closeEvent(self, event):
        """Handle window close event"""
        self.stop_stream()
        event.accept()