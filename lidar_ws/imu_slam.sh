#!/bin/bash
# =============================================================================
# IMU_SLAM.SH - Script to start SLAM mapping with IMU odometry
# =============================================================================
# This script:
# 1. Starts the LiDAR UDP receiver
# 2. Sets up IMU MQTT to ROS bridge
# 3. Sets up the proper coordinate transforms (TF)
# 4. Runs Cartographer with IMU data integration
# 5. Publishes occupancy grid (for visualization)
# 
# Usage: ./imu_slam.sh
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
    echo "Shutting down IMU SLAM processes..."
    for pid in $LIDAR_PID $LAUNCH_PID; do
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

# Create log directory if it doesn't exist
mkdir -p ~/Documents/Spiderverse/lidar_ws/logs

# Make sure scripts are executable
chmod +x ~/Documents/Spiderverse/lidar_ws/src/imu_mqtt_bridge_node.py
chmod +x ~/Documents/Spiderverse/lidar_ws/src/imu_odometry_slam_launch.py

echo "==== STEP 1: Starting LiDAR UDP receiver ===="
ros2 launch lidar_udp_receiver lidar_receiver.launch.py &
LIDAR_PID=$!
sleep 3

echo "==== STEP 2: Starting IMU-based SLAM ===="
# Note: The launch file already includes all needed components:
# - IMU MQTT bridge
# - Static transform publishers
# - Cartographer node
# - Occupancy grid node
ros2 launch ~/Documents/Spiderverse/lidar_ws/src/imu_odometry_slam_launch.py &
LAUNCH_PID=$!
sleep 3

echo ""
echo "================ IMU-BASED SLAM STARTED ================"
echo "SLAM is now running with IMU-based odometry."
echo ""
echo "INSTRUCTIONS:"
echo "1. Move your robot around to map the area"
echo "2. Run './save_map.sh' in another terminal to save the map"
echo "3. Press Ctrl+C in this terminal when done mapping"
echo ""
echo "Available topics:"
ros2 topic list | grep -E "scan|map|tf|submap|imu|odom" | sort
echo "========================================================="

# Keep the script running until Ctrl+C is pressed
echo "Mapping in progress... (Press Ctrl+C to stop)"
wait