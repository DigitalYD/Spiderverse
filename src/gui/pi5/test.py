#!/usr/bin/env python3
import subprocess
import time
import signal
import sys

# Default settings
TARGET_IP = "192.168.0.41"  # Change this to your laptop's IP
PORT = 5000
WIDTH = 640
HEIGHT = 480
FRAMERATE = 30
DEVICE = "/dev/video0"

def start_streaming():
    # Pipeline for streaming using MJPG format
    pipeline = (
        f"gst-launch-1.0 v4l2src device={DEVICE} ! "
        f"image/jpeg,width={WIDTH},height={HEIGHT},framerate={FRAMERATE}/1 ! "
        f"jpegdec ! videoconvert ! "
        f"jpegenc quality=85 ! "
        f"rtpjpegpay ! "
        f"udpsink host={TARGET_IP} port={PORT}"
    )
    
    print(f"Starting MJPG GStreamer stream to {TARGET_IP}:{PORT}")
    print("Press Ctrl+C to stop")
    print(f"Pipeline: {pipeline}")
    
    # Execute the GStreamer pipeline as a shell command
    process = subprocess.Popen(pipeline, shell=True)
    
    # Set up signal handling for clean exit
    def signal_handler(sig, frame):
        print("\nStopping GStreamer...")
        process.terminate()
        process.wait(timeout=5)
        print("Stream ended")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Wait for the process to complete
    process.wait()

if __name__ == "__main__":
    start_streaming()
