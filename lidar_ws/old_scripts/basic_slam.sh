#!/bin/bash
# Most basic SLAM script with minimal configuration

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
    if [ -n "$CART_PID" ]; then
        kill -9 $CART_PID 2>/dev/null
    fi
    if [ -n "$GRID_PID" ]; then
        kill -9 $GRID_PID 2>/dev/null
    fi
    
    echo "Running final cleanup..."
    ~/Documents/Spiderverse/lidar_ws/cleanup.sh
    exit 0
}

# Set trap for clean shutdown
trap cleanup SIGINT SIGTERM EXIT

CONFIG_DIR=~/Documents/Spiderverse/lidar_ws/src/lidar_udp_receiver/config

# Start LiDAR UDP receiver
echo "Starting LiDAR UDP receiver..."
ros2 launch lidar_udp_receiver lidar_receiver.launch.py &
LIDAR_PID=$!

sleep 3

# Start TF broadcaster
echo "Starting TF broadcaster..."
ros2 run lidar_udp_receiver tf_broadcaster &
TF_PID=$!

sleep 2

# Start Cartographer with simplified config
echo "Starting Cartographer SLAM..."
ros2 run cartographer_ros cartographer_node \
    -configuration_directory $CONFIG_DIR \
    -configuration_basename fixed_cartographer_config.lua &
CART_PID=$!

sleep 3

# Start Occupancy Grid node
echo "Starting Occupancy Grid node..."
ros2 run cartographer_ros occupancy_grid_node \
    -resolution 0.05 \
    -publish_period_sec 1.0 &
GRID_PID=$!

echo "All components started."
echo "To save a map, run: ./save_latest_map.sh in another terminal"
echo "Press Ctrl+C to stop SLAM"

wait