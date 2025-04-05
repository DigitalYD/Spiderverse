#!/bin/bash
# =============================================================================
# POSITIONING_SLAM.SH - SLAM mapping with the RPLidar and positioning system
# =============================================================================
# This script:
# 1. Starts the LiDAR UDP receiver
# 2. Sets up the proper coordinate transforms (TF)
# 3. Runs a positioning system (trilateration or bilateration)
# 4. Runs Cartographer for SLAM mapping
# 5. Publishes occupancy grid (for visualization)
# 
# Usage: ./positioning_slam.sh [tri|bi|none]
#        tri - use trilateration (3 anchors)
#        bi - use bilateration (2 anchors)
#        none - no positioning system (default)
#
# Press Ctrl+C to stop mapping when done
# =============================================================================

# Check if positioning method is provided
POSITIONING="none"
if [ $# -ge 1 ]; then
    case "$1" in
        tri|trilateration)
            POSITIONING="trilateration"
            ;;
        bi|bilateration)
            POSITIONING="bilateration"
            ;;
        none)
            POSITIONING="none"
            ;;
        *)
            echo "Unknown positioning method: $1"
            echo "Valid options: tri, bi, none"
            exit 1
            ;;
    esac
fi

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# First, clean up existing processes
echo "Cleaning up existing processes..."
~/Documents/Spiderverse/lidar_ws/cleanup.sh

# Define cleanup function
cleanup() {
    echo "Shutting down SLAM processes..."
    for pid in $LIDAR_PID $TF_PID $LAUNCH_PID; do
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

echo "==== STEP 3: Starting SLAM with $POSITIONING positioning ===="
ros2 launch lidar_udp_receiver positioning_slam_launch.py positioning_method:=$POSITIONING &
LAUNCH_PID=$!
sleep 3

echo ""
echo "====================== SLAM STARTED ======================"
echo "SLAM is now running and mapping your environment."
if [ "$POSITIONING" != "none" ]; then
    echo "Using $POSITIONING for additional positioning data."
fi
echo ""
echo "INSTRUCTIONS:"
echo "1. Move your RPLidar around to map the area"
echo "2. Run './save_map.sh' in another terminal to save the map"
echo "3. Press Ctrl+C in this terminal when done mapping"
echo ""
echo "Available topics:"
ros2 topic list | grep -E "scan|map|tf|submap|positioning" | sort
echo "========================================================="

# Keep the script running until Ctrl+C is pressed
echo "Mapping in progress... (Press Ctrl+C to stop)"
wait