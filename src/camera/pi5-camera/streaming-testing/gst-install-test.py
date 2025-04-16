# test_gstreamer.py
import cv2
import subprocess

# Check GStreamer installation
try:
    # Try to list available GStreamer plugins
    result = subprocess.run(["gst-inspect-1.0", "--version"], 
                          capture_output=True, text=True)
    print("GStreamer version check:", result.stdout)
except FileNotFoundError:
    print("GStreamer tools not found. Please install gstreamer1.0-tools")

# Try a simple GStreamer pipeline
print("Testing simple GStreamer pipeline...")
test_pipeline = "videotestsrc ! videoconvert ! appsink"
test_cap = cv2.VideoCapture(test_pipeline, cv2.CAP_GSTREAMER)
if test_cap.isOpened():
    print("Test pipeline successful!")
    test_cap.release()
else:
    print("Test pipeline failed. GStreamer setup issue detected.")

# Check if we can access the camera
print("Testing camera access...")
camera = cv2.VideoCapture(0)
if camera.isOpened():
    ret, frame = camera.read()
    if ret:
        print(f"Camera working. Frame shape: {frame.shape}")
    else:
        print("Camera opened but couldn't read frame.")
    camera.release()
else:
    print("Could not open camera.")
