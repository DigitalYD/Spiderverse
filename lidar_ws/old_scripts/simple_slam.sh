#!/bin/bash
# Simple SLAM script that focuses just on map creation
# Uses the ROS2 Navigation2 stack for SLAM instead of Cartographer

# Source ROS2 and workspace setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# First, run cleanup to ensure no processes are already running
echo "Running cleanup to ensure no existing processes..."
~/Documents/Spiderverse/lidar_ws/cleanup.sh

# Define cleanup function for clean shutdown
cleanup() {
  echo "Shutting down all processes..."
  for pid in ${PIDS[@]}; do
    if [ -n "$pid" ]; then
      kill -9 $pid 2>/dev/null
    fi
  done
  
  echo "Running final cleanup to ensure all processes terminated..."
  ~/Documents/Spiderverse/lidar_ws/cleanup.sh
  exit 0
}

# Setup trap for clean shutdown
trap cleanup SIGINT SIGTERM EXIT

# Array to store PIDs
PIDS=()

echo "Starting LiDAR UDP receiver..."
# Start the LIDAR UDP receiver
ros2 launch lidar_udp_receiver lidar_receiver.launch.py &
PIDS+=($!)

# Sleep briefly to ensure the LIDAR node starts up first
sleep 2

# Start the TF broadcaster
echo "Starting TF broadcaster..."
ros2 run lidar_udp_receiver tf_broadcaster &
PIDS+=($!)

# Give TF broadcaster time to start
sleep 2

# Check if SLAM toolbox is installed
if ! ros2 pkg list | grep -q slam_toolbox; then
  echo "SLAM Toolbox is not installed. Installing..."
  sudo apt update
  sudo apt install -y ros-humble-slam-toolbox
fi

# Start SLAM Toolbox
echo "Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false &
PIDS+=($!)

sleep 3

echo "Checking published topics..."
ros2 topic list

echo "Starting RViz2 for visualization..."
# Run RViz2 with slam configuration
ros2 run rviz2 rviz2 -d ~/Documents/Spiderverse/lidar_ws/src/lidar_udp_receiver/config/slam.rviz

# RViz closed, cleanup is handled by the EXIT trap