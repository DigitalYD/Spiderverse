import os
import cv2
import zmq
import pickle
import numpy as np
import time

# Check if we're in a GUI environment
has_display = "DISPLAY" in os.environ

# Set up ZeroMQ subscriber
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://192.168.0.49:5555")  # Connect to Pi's IP address
socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

# Variables for FPS calculation
frame_count = 0
start_time = time.time()
fps = 0

# Variables for latency measurement
latencies = []

print("Camera receiver started. Press 'q' to quit.")

try:
    while True:
        try:
            # Receive frame with timeout
            serialized = socket.recv(flags=zmq.NOBLOCK)
            receive_time = time.time()
            
            # Deserialize the data
            data = pickle.loads(serialized)
            compressed_rgb = data["frame"]
            send_time = data["timestamp"]
            resolution = data["resolution"] if "resolution" in data else None
            
            # Calculate latency
            latency = (receive_time - send_time) * 1000  # in milliseconds
            latencies.append(latency)
            if len(latencies) > 100:  # Keep only the last 100 values
                latencies.pop(0)
            avg_latency = sum(latencies) / len(latencies)
            
            # Decompress the image - now using COLOR instead of GRAYSCALE
            frame = cv2.imdecode(compressed_rgb, cv2.IMREAD_COLOR)
            
            # Calculate FPS
            frame_count += 1
            elapsed_time = time.time() - start_time
            if elapsed_time >= 1.0:  # Update FPS every second
                fps = frame_count / elapsed_time
                frame_count = 0
                start_time = time.time()
            
            # Display stats on the frame
            if has_display:
                # Add text to the frame
                stats_frame = frame.copy()
                cv2.putText(stats_frame, f"Resolution: {frame.shape[1]}x{frame.shape[0]}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(stats_frame, f"FPS: {fps:.1f}", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(stats_frame, f"Latency: {avg_latency:.1f} ms", (10, 90), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Show the frame
                try:
                    cv2.imshow("Pi Camera Stream (RGB)", stats_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                except cv2.error as e:
                    print(f"OpenCV display error: {e}")
                    has_display = False
            else:
                # Print stats periodically if not displaying
                if frame_count % 10 == 0:
                    print(f"Receiving: {frame.shape[1]}x{frame.shape[0]} | FPS: {fps:.1f} | Latency: {avg_latency:.1f} ms")
                
        except zmq.Again:
            # No message received within timeout
            pass
        except Exception as e:
            print(f"Error: {e}")

except KeyboardInterrupt:
    print("\nStopping receiver...")
finally:
    # Clean up
    if has_display:
        try:
            cv2.destroyAllWindows()
        except:
            pass
    socket.close()
    context.term()
    print("Receiver stopped.")