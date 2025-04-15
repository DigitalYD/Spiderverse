#!/usr/bin/env python3

import os
import sys
import subprocess
import argparse
import json
import signal
import time
import math
import numpy as np
from threading import Thread
import paho.mqtt.client as mqtt

# Default MQTT configuration
MQTT_BROKER = "192.168.0.49"  # Default Pi address
MQTT_PORT = 1883
MQTT_TOPIC = "pi5/imu"

# Path to SLAM workspace
SLAM_WORKSPACE = os.path.expanduser("~/Documents/Spiderverse/lidar_ws")

class IMUOdometryProcessor:
    """
    Processes IMU data from MQTT and converts it to odometry for SLAM
    """
    def __init__(self, mqtt_broker=MQTT_BROKER, mqtt_port=MQTT_PORT, mqtt_topic=MQTT_TOPIC):
        # MQTT client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # MQTT connection parameters
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_topic = mqtt_topic
        self.mqtt_connected = False
        
        # IMU data tracking
        self.last_imu_data = None
        self.last_timestamp = None
        self.total_distance = 0
        self.position = [0.0, 0.0]  # [x, y] in meters
        self.orientation = 0.0       # yaw in radians
        self.linear_velocity = [0.0, 0.0]  # [vx, vy] in m/s
        
        # Configuration
        self.running = True
        self.verbose = False
        
        # Launch odometry publisher thread
        self.odom_thread = Thread(target=self.publish_odometry)
        self.odom_thread.daemon = True
    
    def start(self):
        """Start the IMU processor"""
        print(f"Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
        try:
            # Connect to MQTT broker
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            
            # Start odometry publisher
            self.odom_thread.start()
            
            print(f"IMU odometry processor started, listening on {self.mqtt_topic}")
            return True
            
        except Exception as e:
            print(f"Error starting IMU processor: {e}")
            return False
    
    def stop(self):
        """Stop the IMU processor"""
        self.running = False
        
        # Disconnect MQTT client
        if self.mqtt_connected:
            self.mqtt_client.disconnect()
            self.mqtt_client.loop_stop()
        
        # Wait for thread to finish
        if self.odom_thread.is_alive():
            self.odom_thread.join(timeout=1.0)
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback for MQTT connection"""
        if rc == 0:
            # Successfully connected
            self.mqtt_connected = True
            print("Connected to MQTT broker")
            
            # Subscribe to IMU topic
            client.subscribe(self.mqtt_topic)
            print(f"Subscribed to {self.mqtt_topic}")
        else:
            # Connection failed
            self.mqtt_connected = False
            print(f"Failed to connect to MQTT broker: {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback for MQTT disconnection"""
        self.mqtt_connected = False
        
        if rc == 0:
            print("Disconnected from MQTT broker")
        else:
            print(f"Unexpected disconnect from MQTT broker: {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """Process incoming IMU data from MQTT"""
        try:
            # Parse JSON payload
            payload = json.loads(msg.payload.decode('utf-8'))
            
            # Process IMU data for odometry
            self.process_imu_data(payload)
            
        except Exception as e:
            print(f"Error processing IMU message: {e}")
    
    def process_imu_data(self, imu_data):
        """Process IMU data and update odometry estimates"""
        # Get current timestamp
        current_time = time.time()
        
        # Store first timestamp
        if self.last_timestamp is None:
            self.last_timestamp = current_time
            self.last_imu_data = imu_data
            return
        
        # Calculate delta time
        dt = current_time - self.last_timestamp
        self.last_timestamp = current_time
        
        # Skip processing if dt is too small or too large (can happen with MQTT delays)
        if dt < 0.001 or dt > 0.5:
            return
        
        # Extract IMU data
        try:
            # Get orientation (yaw) from euler angles
            euler = imu_data.get("euler", {})
            yaw_deg = euler.get("yaw", 0)
            yaw_rad = math.radians(yaw_deg)
            
            # Get linear acceleration
            accel = imu_data.get("linear_acceleration", {})
            accel_x = accel.get("x", 0)
            accel_y = accel.get("y", 0)
            
            # Get angular velocity
            gyro = imu_data.get("angular_velocity", {})
            gyro_z = gyro.get("z", 0)  # Angular velocity around Z axis (yaw)
            
            # Update orientation based on gyro
            # We use gyro for short-term accuracy and magnetometer for long-term stability
            self.orientation += gyro_z * dt
            
            # But we also gently pull towards the magnetometer heading to avoid drift
            mag_weight = 0.02  # Weight for magnetometer correction (adjust as needed)
            orientation_error = yaw_rad - self.orientation
            # Normalize to [-pi, pi]
            if orientation_error > math.pi:
                orientation_error -= 2 * math.pi
            elif orientation_error < -math.pi:
                orientation_error += 2 * math.pi
            self.orientation += orientation_error * mag_weight
            
            # Update velocity using acceleration in world frame
            # Convert acceleration from body frame to world frame
            cos_yaw = math.cos(self.orientation)
            sin_yaw = math.sin(self.orientation)
            
            world_accel_x = accel_x * cos_yaw - accel_y * sin_yaw
            world_accel_y = accel_x * sin_yaw + accel_y * cos_yaw
            
            # Apply a high-pass filter to acceleration to remove bias/gravity
            accel_threshold = 0.1  # m/s² (adjust based on IMU noise)
            world_accel_x = world_accel_x if abs(world_accel_x) > accel_threshold else 0
            world_accel_y = world_accel_y if abs(world_accel_y) > accel_threshold else 0
            
            # Update velocity (integrate acceleration)
            self.linear_velocity[0] += world_accel_x * dt
            self.linear_velocity[1] += world_accel_y * dt
            
            # Apply damping to velocity to prevent drift (simulating friction)
            damping = 0.95  # Adjust based on desired behavior
            self.linear_velocity[0] *= damping
            self.linear_velocity[1] *= damping
            
            # Update position (integrate velocity)
            delta_x = self.linear_velocity[0] * dt
            delta_y = self.linear_velocity[1] * dt
            
            self.position[0] += delta_x
            self.position[1] += delta_y
            
            # Calculate total distance traveled
            self.total_distance += math.sqrt(delta_x**2 + delta_y**2)
            
            # Print status if verbose
            if self.verbose:
                print(f"Position: ({self.position[0]:.2f}, {self.position[1]:.2f}), " +
                      f"Orientation: {math.degrees(self.orientation):.2f}°, " +
                      f"Distance: {self.total_distance:.2f}m")
            
            # Store data for next iteration
            self.last_imu_data = imu_data
            
        except Exception as e:
            print(f"Error updating odometry: {e}")
    
    def publish_odometry(self):
        """Publish odometry to ROS for SLAM"""
        # This function runs in a separate thread
        # In a real implementation, you would use ROS publishers to send odometry data
        # For simplicity, we'll just simulate the publishing with a log message
        update_rate = 10  # Hz
        
        while self.running:
            if self.last_imu_data is not None:
                # Here we would publish to ROS
                # For now, just log position data every second
                if self.verbose:
                    print(f"[ODOMETRY] Position: ({self.position[0]:.2f}, {self.position[1]:.2f}), " +
                          f"Orientation: {math.degrees(self.orientation):.2f}°")
            
            # Sleep to maintain update rate
            time.sleep(1.0 / update_rate)

def launch_slam_with_imu():
    """Launch SLAM with IMU odometry"""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Launch SLAM with IMU-based odometry')
    parser.add_argument('--broker', type=str, default=MQTT_BROKER,
                        help=f'MQTT broker address (default: {MQTT_BROKER})')
    parser.add_argument('--port', type=int, default=MQTT_PORT,
                        help=f'MQTT broker port (default: {MQTT_PORT})')
    parser.add_argument('--topic', type=str, default=MQTT_TOPIC,
                        help=f'MQTT topic for IMU data (default: {MQTT_TOPIC})')
    parser.add_argument('--verbose', action='store_true',
                        help='Enable verbose output')
    args = parser.parse_args()
    
    # Print banner
    print("="*80)
    print(" IMU-Based SLAM Launcher ".center(80, '='))
    print("="*80)
    print(f"MQTT Broker: {args.broker}:{args.port}")
    print(f"IMU Topic: {args.topic}")
    print(f"SLAM Workspace: {SLAM_WORKSPACE}")
    print("="*80)
    
    # Create IMU odometry processor
    imu_processor = IMUOdometryProcessor(
        mqtt_broker=args.broker,
        mqtt_port=args.port,
        mqtt_topic=args.topic
    )
    imu_processor.verbose = args.verbose
    
    # Start IMU processor
    if not imu_processor.start():
        print("Failed to start IMU processor. Exiting.")
        return 1
    
    # Launch SLAM in a separate process
    try:
        print("\nLaunching SLAM with IMU odometry...")
        
        # First ensure we have the launch file created
        print("Ensuring IMU SLAM launch file is created...")
        launch_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "create_imu_launch_file.py")
        if os.path.exists(launch_script):
            try:
                subprocess.run(["python3", launch_script], check=True)
                print("IMU SLAM launch file created successfully.")
            except subprocess.CalledProcessError as e:
                print(f"Warning: Failed to create launch file: {e}")
        else:
            print(f"Warning: Launch file creator script not found at {launch_script}")
        
        # Use the self-contained script in our directory instead of the workspace
        slam_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "imu_slam.sh")
        if not os.path.exists(slam_script):
            print(f"Error: IMU SLAM script not found at {slam_script}")
            return 1
        
        # Make sure it's executable
        os.chmod(slam_script, 0o755)
        
        # Launch SLAM in a new terminal window
        if sys.platform == 'linux':
            # For Linux, use xterm, gnome-terminal, or konsole
            for terminal in ['xterm', 'gnome-terminal', 'konsole']:
                if subprocess.call(['which', terminal], stdout=subprocess.PIPE) == 0:
                    if terminal == 'gnome-terminal':
                        cmd = [terminal, '--', 'bash', '-c', f"cd {SLAM_WORKSPACE} && bash imu_slam.sh; exec bash"]
                    elif terminal == 'konsole':
                        cmd = [terminal, '-e', f"cd {SLAM_WORKSPACE} && bash imu_slam.sh; exec bash"]
                    else:  # xterm
                        cmd = [terminal, '-e', f"cd {SLAM_WORKSPACE} && bash imu_slam.sh; exec bash"]
                    
                    slam_process = subprocess.Popen(cmd)
                    print(f"SLAM launched in separate {terminal} window")
                    break
            else:
                # No terminal found, run script directly
                cmd = ["bash", slam_script]
                slam_process = subprocess.Popen(
                    cmd,
                    cwd=SLAM_WORKSPACE
                )
                print("SLAM launched (no terminal available)")
        else:
            # For other platforms
            cmd = ["bash", slam_script]
            slam_process = subprocess.Popen(
                cmd,
                cwd=SLAM_WORKSPACE
            )
            print("SLAM launched")
        
        # Setup signal handlers for clean shutdown
        def signal_handler(sig, frame):
            print("\nShutting down...")
            imu_processor.stop()
            if slam_process.poll() is None:
                slam_process.terminate()
                try:
                    slam_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    slam_process.kill()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        print("\nIMU SLAM system running. Press Ctrl+C to exit.")
        
        # Main loop - just wait for signal or process exit
        while True:
            if slam_process.poll() is not None:
                # SLAM process has exited
                print(f"\nSLAM process exited with code {slam_process.returncode}")
                break
            time.sleep(1)
        
        # Cleanup
        imu_processor.stop()
        return 0
        
    except Exception as e:
        print(f"Error launching SLAM: {e}")
        imu_processor.stop()
        return 1

def create_imu_slam_script(script_path):
    """Create the IMU SLAM launch script if it doesn't exist"""
    script_content = """#!/bin/bash
#
# IMU-based SLAM launch script
#

# Set script to exit on error
set -e

# Define workspace and package directories
LIDAR_WS="$HOME/Documents/Spiderverse/lidar_ws"
ROS_LOG_DIR="$LIDAR_WS/logs"

# Create log directory if it doesn't exist
mkdir -p "$ROS_LOG_DIR"

# Set ROS environment variables
source /opt/ros/humble/setup.bash
if [ -f "$LIDAR_WS/install/setup.bash" ]; then
  source "$LIDAR_WS/install/setup.bash"
fi

# Make sure required ROS packages are installed
if ! ros2 pkg list | grep -q "cartographer_ros"; then
  echo "cartographer_ros package not found, attempting to install..."
  sudo apt-get update && sudo apt-get install -y ros-humble-cartographer-ros
fi

# Check if we have pip and needed packages
if ! python3 -c "import transforms3d" 2>/dev/null; then
  echo "Installing transforms3d package..."
  pip3 install --user transforms3d || echo "Warning: Could not install transforms3d"
fi

if ! python3 -c "import paho.mqtt" 2>/dev/null; then
  echo "Installing paho-mqtt package..."
  pip3 install --user paho-mqtt || echo "Warning: Could not install paho-mqtt"
fi

# Check if we have tf2_ros package
if ! ros2 pkg list | grep -q "tf2_ros"; then
  echo "tf2_ros package not found, attempting to install..."
  sudo apt-get install -y ros-humble-tf2-ros || echo "Warning: Could not install tf2_ros, continuing anyway"
fi

# Log script start
echo "[$(date)] Starting IMU-based SLAM"

# First check if launch file exists
if ros2 pkg list | grep -q "cartographer_ros"; then
  LAUNCH_FILE=$(ros2 pkg prefix cartographer_ros)/share/cartographer_ros/launch/cartographer_slam_imu.launch.py
  FALLBACK_LAUNCH_FILE="$LIDAR_WS/src/cartographer_ros/launch/cartographer_slam_imu.launch.py"
  
  if [ -f "$LAUNCH_FILE" ]; then
    echo "Found launch file at $LAUNCH_FILE"
    # Launch SLAM with IMU odometry
    ros2 launch cartographer_ros cartographer_slam_imu.launch.py
  elif [ -f "$FALLBACK_LAUNCH_FILE" ]; then
    echo "Found launch file at $FALLBACK_LAUNCH_FILE"
    # Launch using full path
    ros2 launch "$FALLBACK_LAUNCH_FILE"
  else
    echo "Error: Launch file not found at $LAUNCH_FILE or $FALLBACK_LAUNCH_FILE"
    echo "Please run the create_imu_launch_file.py script first"
    exit 1
  fi
else
  echo "Error: cartographer_ros package not found"
  echo "Please install ROS2 Cartographer package with:"
  echo "  sudo apt-get install ros-humble-cartographer-ros"
  exit 1
fi

# Exit message - this will only show if ROS2 is terminated normally
echo "[$(date)] SLAM system has terminated"
"""
    
    # Write the script to file
    with open(script_path, 'w') as f:
        f.write(script_content)
    
    # Make the script executable
    os.chmod(script_path, 0o755)

if __name__ == "__main__":
    sys.exit(launch_slam_with_imu())