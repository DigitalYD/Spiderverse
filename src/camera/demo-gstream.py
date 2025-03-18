# demo-gstream-receiver.py for laptop
import subprocess
import time
import signal
import sys
import threading

# Port to receive stream
RTP_PORT = 5000

def measure_fps():
    """Thread to measure received FPS using gst element stats"""
    cmd = f"gst-launch-1.0 -v udpsrc port={RTP_PORT} ! fakesink"
    try:
        process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        
        start_time = time.time()
        packets = 0
        
        while True:
            line = process.stderr.readline()
            if "chain" in line:
                packets += 1
            
            if packets % 30 == 0 and packets > 0:
                now = time.time()
                elapsed = now - start_time
                fps = packets / elapsed
                print(f"Receiving approximately {fps:.2f} FPS")
            
            time.sleep(0.01)
    except KeyboardInterrupt:
        process.terminate()
    finally:
        if 'process' in locals():
            process.terminate()

def start_receiving():
    # Pipeline for receiving and displaying grayscale video
    pipeline = (
        f"gst-launch-1.0 udpsrc port={RTP_PORT} caps=\"application/x-rtp,encoding-name=JPEG,payload=26\" ! "
        f"rtpjpegdepay ! jpegdec ! videoconvert ! "
        f"fpsdisplaysink video-sink=autovideosink text-overlay=true sync=false"
    )
    
    print(f"Starting GStreamer receiver on port {RTP_PORT}")
    print("Press Ctrl+C to stop")
    print(f"Pipeline: {pipeline}")
    
    # Execute the GStreamer pipeline as a shell command
    process = subprocess.Popen(pipeline, shell=True)
    
    # Set up signal handling for clean exit
    def signal_handler(sig, frame):
        print("\nStopping GStreamer...")
        process.terminate()
        process.wait(timeout=5)
        print("Receiver stopped")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Wait for the process to complete
    process.wait()

if __name__ == "__main__":
    start_receiving()
