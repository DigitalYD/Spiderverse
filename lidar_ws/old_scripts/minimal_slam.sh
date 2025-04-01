#!/bin/bash
# Minimal SLAM script for debugging - runs only essential components

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# First, clean up existing processes
echo "Cleaning up existing processes..."
~/Documents/Spiderverse/lidar_ws/cleanup.sh

# Define cleanup function
cleanup() {
    echo "Shutting down all processes..."
    if [ -n "$LIDAR_PID" ]; then
        kill -9 $LIDAR_PID 2>/dev/null
    fi
    if [ -n "$TF_PID" ]; then
        kill -9 $TF_PID 2>/dev/null
    fi
    
    echo "Running final cleanup..."
    ~/Documents/Spiderverse/lidar_ws/cleanup.sh
    exit 0
}

# Set trap for clean shutdown
trap cleanup SIGINT SIGTERM EXIT

# Step 1: Start LiDAR UDP receiver
echo "Step 1: Starting LiDAR UDP receiver..."
ros2 launch lidar_udp_receiver lidar_receiver.launch.py &
LIDAR_PID=$!

# Wait to verify LiDAR started
sleep 5

# Check if LiDAR process is running
if ! ps -p $LIDAR_PID > /dev/null; then
    echo "ERROR: LiDAR receiver failed to start."
    exit 1
fi

# Look for scan topic
echo "Checking for /scan topic..."
if ! timeout 5s ros2 topic list | grep -q "/scan"; then
    echo "WARNING: /scan topic not found. LiDAR may not be publishing data."
    echo "Available topics:"
    ros2 topic list
else
    echo "SUCCESS: /scan topic found!"
    echo "Topic info:"
    ros2 topic info /scan
fi

# Step 2: Start TF broadcaster
echo "Step 2: Starting TF broadcaster..."
ros2 run lidar_udp_receiver tf_broadcaster &
TF_PID=$!

# Wait to verify TF broadcaster started
sleep 2

# Check if TF process is running
if ! ps -p $TF_PID > /dev/null; then
    echo "ERROR: TF broadcaster failed to start."
    exit 1
fi

# Check for TF topic
echo "Checking for /tf topic..."
if ! timeout 5s ros2 topic list | grep -q "/tf"; then
    echo "WARNING: /tf topic not found. TF broadcaster may not be working."
    echo "Available topics:"
    ros2 topic list
else
    echo "SUCCESS: /tf topic found!"
    echo "TF messages:"
    timeout 5s ros2 topic echo /tf --once
fi

echo ""
echo "Basic components are running. Would you like to continue with SLAM? (y/n)"
read -p "> " response

if [[ "$response" == "y" || "$response" == "Y" ]]; then
    # Continue with Cartographer
    echo "Starting Cartographer..."
    CONFIG_DIR=~/Documents/Spiderverse/lidar_ws/src/lidar_udp_receiver/config
    ros2 run cartographer_ros cartographer_node \
        -configuration_directory $CONFIG_DIR \
        -configuration_basename simple_cartographer_config.lua
else
    echo "Exiting without starting SLAM."
    exit 0
fi