#!/bin/bash
# =============================================================================
# SLAM.SH - Main script to start SLAM mapping with the RPLidar
# =============================================================================
# This script:
# 1. Starts the LiDAR UDP receiver
# 2. Sets up the proper coordinate transforms (TF)
# 3. Runs Cartographer for SLAM mapping
# 4. Publishes occupancy grid (for visualization)
# 
# Usage: ./slam.sh
# Press Ctrl+C to stop mapping when done
# =============================================================================

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# First, clean up existing processes
echo "Cleaning up existing processes..."
~/Documents/Spiderverse/lidar_ws/cleanup.sh

# Define cleanup function
cleanup() {
    echo "Shutting down SLAM processes..."
    for pid in $LIDAR_PID $TF_PID $CART_PID $GRID_PID; do
        if [ -n "$pid" ] && ps -p $pid > /dev/null; then
            kill -9 $pid 2>/dev/null
        fi
    done
    
    echo "Running final cleanup..."
    ~/Documents/Spiderverse/lidar_ws/cleanup.sh
    exit 0
}

# Set trap for clean shutdown
trap cleanup SIGINT SIGTERM EXIT

# Create Cartographer configuration
CONFIG_DIR=~/Documents/Spiderverse/lidar_ws/src/lidar_udp_receiver/config
echo "Using RPLidar-optimized Cartographer configuration..."

echo "==== STEP 1: Starting LiDAR UDP receiver ===="
ros2 launch lidar_udp_receiver lidar_receiver.launch.py &
LIDAR_PID=$!
sleep 3

echo "==== STEP 2: Starting TF broadcaster ===="
ros2 run lidar_udp_receiver tf_broadcaster &
TF_PID=$!
sleep 2

echo "==== STEP 3: Starting Cartographer SLAM ===="
ros2 run cartographer_ros cartographer_node \
    -configuration_directory $CONFIG_DIR \
    -configuration_basename udp_lidar_cartographer_config.lua &
CART_PID=$!
sleep 3

echo "==== STEP 4: Starting Occupancy Grid Publisher ===="
# Fixed: Using the correct executable name cartographer_occupancy_grid_node
ros2 run cartographer_ros cartographer_occupancy_grid_node \
    -resolution 0.05 \
    -publish_period_sec 1.0 &
GRID_PID=$!
sleep 2

echo ""
echo "====================== SLAM STARTED ======================"
echo "SLAM is now running and mapping your environment."
echo ""
echo "INSTRUCTIONS:"
echo "1. Move your RPLidar around to map the area"
echo "2. Run './save_map.sh' in another terminal to save the map"
echo "3. Press Ctrl+C in this terminal when done mapping"
echo ""
echo "Available topics:"
ros2 topic list | grep -E "scan|map|tf|submap" | sort
echo "========================================================="

# Keep the script running until Ctrl+C is pressed
echo "Mapping in progress... (Press Ctrl+C to stop)"
wait