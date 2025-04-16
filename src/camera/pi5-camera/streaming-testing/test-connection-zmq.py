# pi_camera_sender.py
import cv2
import zmq
import time
import pickle
import numpy as np

# Initialize camera
cap = cv2.VideoCapture(0)  # Use 0 for default camera, or try 2, 1 if that doesn't work
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set resolution (lower for better performance)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Set up ZeroMQ publisher
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

print("Pi camera server started. Streaming grayscale frames...")

try:
    while True:
        # Capture frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame.")
            break
            
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Compress before sending
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
        _, compressed_gray = cv2.imencode('.jpg', gray, encode_param)
        
        # Serialize using pickle
        data = pickle.dumps(compressed_gray)
        
        # Send the frame
        socket.send(data)
        
        # Simple progress indicator
        print(".", end="", flush=True)
        if int(time.time()) % 5 == 0:
            print(" Frame sent")
        
        # Rate limiting
        time.sleep(0.1)  # 10 FPS, adjust as needed
        
except KeyboardInterrupt:
    print("\nStopping server...")
finally:
    # Clean up
    cap.release()
    socket.close()
    context.term()
    print("Server stopped.")
