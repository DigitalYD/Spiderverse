#!/usr/bin/env python3

import subprocess
import time
import signal
import sys
import os
import glob
import argparse
from typing import List, Optional, Tuple

# Default configurations
DEFAULT_IP = "192.168.0.41"
DEFAULT_PORT = 5000
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FRAMERATE = 30  # Increased from 15 for smoother video
DEFAULT_QUALITY = 60    # Reduced from 70 for lower latency
DEFAULT_LATENCY = 20    # Reduced from 100 for lower latency
DEFAULT_DEVICE = None   # Will be auto-detected

def get_available_video_devices() -> List[str]:
    """Get a list of available video devices."""
    devices = glob.glob('/dev/video*')
    return sorted(devices)

def test_video_device(device: str) -> bool:
    """Test if a video device can be accessed."""
    try:
        cmd = f"v4l2-ctl --device={device} --all"
        result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return result.returncode == 0
    except Exception:
        return False

def get_device_capabilities(device: str) -> dict:
    """Get capabilities of a video device."""
    caps = {}
    try:
        cmd = f"v4l2-ctl --device={device} --list-formats-ext"
        result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode == 0:
            # Parse the output to get supported formats
            output = result.stdout
            caps['formats'] = []
            if 'MJPG' in output:
                caps['formats'].append('MJPG')
            if 'YUYV' in output:
                caps['formats'].append('YUYV')
            if 'RGB' in output:
                caps['formats'].append('RGB')
            # Add more format parsing as needed
        return caps
    except Exception as e:
        print(f"Error getting device capabilities: {e}")
        return {}

def find_best_video_device() -> Optional[str]:
    """Find the best available video device."""
    devices = get_available_video_devices()
    if not devices:
        print("No video devices found!")
        return None
    
    print(f"Found {len(devices)} video devices: {', '.join(devices)}")
    
    # First try to test each device
    for device in devices:
        print(f"Testing {device}...")
        if test_video_device(device):
            caps = get_device_capabilities(device)
            print(f"  Supported formats: {caps.get('formats', ['Unknown'])}")
            return device
    
    # If no device passed the test, just return the first one
    print("Warning: Could not find a fully working video device, using first available.")
    return devices[0]

def create_pipeline(device: str, dest_ip: str, dest_port: int, width: int, height: int, 
                    framerate: int, quality: int, latency: int, debug: bool = False) -> str:
    """Create the GStreamer pipeline optimized for low latency."""
    # Get device capabilities
    caps = get_device_capabilities(device)
    formats = caps.get('formats', [])
    
    # Create pipeline based on supported formats with low-latency optimizations
    if 'MJPG' in formats:
        # Camera supports MJPG directly - optimized for low latency
        pipeline = (
            f"gst-launch-1.0 v4l2src device={device} do-timestamp=true ! "
            f"image/jpeg,width={width},height={480},framerate={framerate}/1 ! "
            f"jpegdec max-errors=-1 ! videoconvert ! "
            f"jpegenc quality={quality} idct-method=ifast ! "
            f"rtpjpegpay ! application/x-rtp,media=video ! "
            f"rtpjitterbuffer latency={latency} ! "
            f"udpsink host={dest_ip} port={dest_port} sync=false async=false buffer-size=32768"
        )
    else:
        # Fall back to raw format - optimized for low latency
        pipeline = (
            f"gst-launch-1.0 v4l2src device={device} do-timestamp=true ! "
            f"video/x-raw,width={width},height={height},framerate={framerate}/1 ! "
            f"videoconvert ! jpegenc quality={quality} idct-method=ifast ! "
            f"rtpjpegpay ! application/x-rtp,media=video ! "
            f"rtpjitterbuffer latency={latency} ! "
            f"udpsink host={dest_ip} port={dest_port} sync=false async=false buffer-size=32768"
        )
    
    # Add debug output if requested - optimized for low latency
    if debug:
        pipeline = pipeline.replace("udpsink", 
                                   f"tee name=t ! queue max-size-buffers=1 leaky=downstream ! udpsink")
        pipeline += f" t. ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! autovideosink"
    
    return pipeline

def start_streaming(device: str = None, dest_ip: str = DEFAULT_IP, dest_port: int = DEFAULT_PORT,
                   width: int = DEFAULT_WIDTH, height: int = DEFAULT_HEIGHT, 
                   framerate: int = DEFAULT_FRAMERATE, quality: int = DEFAULT_QUALITY,
                   latency: int = DEFAULT_LATENCY, debug: bool = False) -> None:
    """Start streaming from the camera."""
    # Auto-detect device if not specified
    if device is None:
        device = find_best_video_device()
        if device is None:
            print("Error: No video device found. Exiting.")
            sys.exit(1)
    
    # Create and print the pipeline
    pipeline = create_pipeline(device, dest_ip, dest_port, width, height, 
                              framerate, quality, latency, debug)
    
    print(f"Starting LOW LATENCY camera stream from {device} to {dest_ip}:{dest_port}")
    print(f"Using: width={width}, height={height}, framerate={framerate}, quality={quality}, latency={latency}ms")
    print("Press Ctrl+C to stop")
    print(f"Pipeline: {pipeline}")
    
    # Execute the GStreamer pipeline
    try:
        process = subprocess.Popen(pipeline, shell=True)
        
        # Set up signal handling for clean exit
        def signal_handler(sig, frame):
            print("\nStopping GStreamer...")
            process.terminate()
            try:
                process.wait(timeout=5)
                print("Stream ended")
            except subprocess.TimeoutExpired:
                print("Process did not terminate gracefully, forcing...")
                process.kill()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        
        # Wait for the process to complete
        process.wait()
        
    except Exception as e:
        print(f"Error starting stream: {e}")
        sys.exit(1)

def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Raspberry Pi Camera Streaming Tool (Low Latency Version)')
    parser.add_argument('--device', type=str, help='Video device to use (e.g., /dev/video10)')
    parser.add_argument('--ip', type=str, default=DEFAULT_IP, help='Destination IP address')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help='Destination port')
    parser.add_argument('--width', type=int, default=DEFAULT_WIDTH, help='Video width')
    parser.add_argument('--height', type=int, default=DEFAULT_HEIGHT, help='Video height')
    parser.add_argument('--framerate', type=int, default=DEFAULT_FRAMERATE, help='Video framerate')
    parser.add_argument('--quality', type=int, default=DEFAULT_QUALITY, help='JPEG quality (0-100)')
    parser.add_argument('--latency', type=int, default=DEFAULT_LATENCY, help='Jitter buffer latency in ms')
    parser.add_argument('--debug', action='store_true', help='Enable debug output')
    parser.add_argument('--list-devices', action='store_true', help='List available video devices and exit')
    
    return parser.parse_args()

def main():
    """Main function."""
    args = parse_args()
    
    # Just list devices if requested
    if args.list_devices:
        devices = get_available_video_devices()
        print("Available video devices:")
        for device in devices:
            if test_video_device(device):
                caps = get_device_capabilities(device)
                print(f"  {device}: Working - Formats: {caps.get('formats', ['Unknown'])}")
            else:
                print(f"  {device}: Not accessible")
        return
    
    # Start streaming
    start_streaming(device=args.device, dest_ip=args.ip, dest_port=args.port,
                   width=args.width, height=args.height, framerate=args.framerate,
                   quality=args.quality, latency=args.latency, debug=args.debug)

if __name__ == "__main__":
    main()
