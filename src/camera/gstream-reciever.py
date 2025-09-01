#!/usr/bin/env python3
import subprocess
import time
import signal
import sys
import threading
import socket
import re
import datetime

# Configuration
RTP_PORT = 5000
DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 720

# Performance metrics
frames_received = 0
start_time = time.time()
current_fps = 0
current_latency = 0
first_frame_received = False
resolution = "Unknown"
stats_lock = threading.Lock()

def get_local_ip():
    """Get the local IP address of this machine"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Doesn't need to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def overlay_stats(frame, text):
    """Callback for the textoverlay element"""
    now = datetime.datetime.now()
    with stats_lock:
        stats = f"FPS: {current_fps:.1f} | Latency: {current_latency:.0f}ms | Res: {resolution} | {now:%H:%M:%S}"
        return stats

def monitor_latency_fps():
    """Thread to monitor pipeline statistics using gst-stats"""
    global current_fps, current_latency, first_frame_received, resolution, frames_received
    
    # Simple UDP listener to count packets and get first-frame timing
    def udp_monitor():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', RTP_PORT))
        sock.settimeout(0.5)
        
        global first_frame_received, frames_received
        while True:
            try:
                data, addr = sock.recvfrom(65536)
                with stats_lock:
                    if not first_frame_received:
                        first_frame_received = True
                    frames_received += 1
            except socket.timeout:
                pass
    
    # Launch the UDP monitor in a separate thread
    udp_thread = threading.Thread(target=udp_monitor, daemon=True)
    udp_thread.start()
    
    # Monitor statistics
    fps_samples = []
    last_count = 0
    
    while True:
        time.sleep(1.0)
        
        with stats_lock:
            now_count = frames_received
            elapsed = time.time() - start_time
            
            # Calculate FPS over the last second
            period_fps = now_count - last_count
            fps_samples.append(period_fps)
            if len(fps_samples) > 5:  # Average over last 5 seconds
                fps_samples.pop(0)
            
            current_fps = sum(fps_samples) / len(fps_samples)
            
            # Estimated latency (based on buffer settings and network)
            # This is just an approximation - real latency would need timestamps
            if period_fps > 0:
                current_latency = (1000.0 / period_fps) * 1.5  # Rough estimate
            
            last_count = now_count
            
            # Print stats to console
            print(f"\rFPS: {current_fps:.1f} | Latency: {current_latency:.0f}ms | Frames: {now_count} | Res: {resolution}", end="")
            sys.stdout.flush()

def start_receiving():
    """Start GStreamer pipeline to receive and display video with performance metrics"""
    global start_time, resolution
    
    # Reset statistics
    with stats_lock:
        start_time = time.time()
    
    # Your laptop IP is 192.168.0.78
    local_ip = "192.168.0.78"
    
    # Enhanced pipeline with on-screen statistics
    pipeline = (
        f"gst-launch-1.0 "
        f"udpsrc port={RTP_PORT} caps=\"application/x-rtp,encoding-name=JPEG,payload=26\" ! "
        f"rtpjpegdepay ! jpegdec ! "
        f"identity signal-handoffs=true name=resolution-probe ! "  # For resolution detection
        f"videoconvert ! "
        f"videoscale method=lanczos ! video/x-raw,width={DISPLAY_WIDTH},height={DISPLAY_HEIGHT} ! "
        f"videoconvert ! "
        f"videobalance saturation=1.5 brightness=0.1 contrast=1.1 ! "
        f"timeoverlay valignment=top halignment=left font-desc=\"Sans 24\" ! "
        f"textoverlay text=\"Stats Loading...\" valignment=top halignment=right font-desc=\"Sans 24\" name=stats_overlay ! "
        f"autovideosink sync=false"
    )
    
    print(f"Starting GStreamer receiver on {local_ip}:{RTP_PORT}")
    print(f"Display size: {DISPLAY_WIDTH}x{DISPLAY_HEIGHT}")
    print("Press Ctrl+C to stop")
    
    # Start statistics monitoring thread
    monitor_thread = threading.Thread(target=monitor_latency_fps, daemon=True)
    monitor_thread.start()
    
    # Execute the GStreamer pipeline
    process = subprocess.Popen(
        pipeline,
        shell=True,
        stderr=subprocess.PIPE,
        universal_newlines=True
    )
    
    # Set up signal handling for clean exit
    def signal_handler(sig, frame):
        print("\nStopping GStreamer receiver...")
        process.terminate()
        process.wait(timeout=5)
        
        with stats_lock:
            elapsed = time.time() - start_time
            avg_fps = frames_received / elapsed if elapsed > 0 else 0
        
        print(f"\nReceiver stopped - Statistics:")
        print(f"Total frames received: {frames_received}")
        print(f"Average FPS: {avg_fps:.2f}")
        print(f"Final resolution detected: {resolution}")
        print(f"Total running time: {elapsed:.2f} seconds")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Monitor process output for resolution information and other diagnostics
    resolution_pattern = re.compile(r'width=(\d+), height=(\d+)')
    
    while True:
        line = process.stderr.readline()
        if not line:
            break
            
        # Look for resolution information
        match = resolution_pattern.search(line)
        if match:
            with stats_lock:
                resolution = f"{match.group(1)}x{match.group(2)}"
    
    # If we get here, the process ended
    process.wait()

if __name__ == "__main__":
    start_receiving()