import subprocess
import time
import signal
import sys

# Configure settings
DEST_IP = "192.168.0.77"  # Update to your laptop's IP
#DEST_IP = "192.168.0.132"
DEST_PORT = 5000

def start_streaming():
    # Modified pipeline using MJPG format while keeping the same structure
    #this is working
    '''pipeline = (
        f"gst-launch-1.0 v4l2src ! "
        f"image/jpeg,width=320,height=240,framerate=30/1 ! "
        f"jpegdec ! videoconvert !"
        f"jpegenc quality=85 ! "
        f"rtpjpegpay ! "
        f"udpsink host={DEST_IP} port={DEST_PORT}"
    )'''
    pipeline = (
    f"gst-launch-1.0 v4l2src do-timestamp=true ! "
    f"image/jpeg,width=640,height=480,framerate=30/1 ! "
    f"jpegdec max-errors=-1 ! videoconvert ! "
    f"jpegenc quality=80 idct-method=ifast ! "
    f"rtpjpegpay ! application/x-rtp,media=video ! "
    f"rtpjitterbuffer latency=40 ! "
    f"udpsink host={DEST_IP} port={DEST_PORT} sync=false async=false buffer-size=65536"
    )
    
    print(f"Starting MJPG GStreamer stream to {DEST_IP}:{DEST_PORT}")
    print("Press Ctrl+C to stop")
    print(pipeline)
    
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
