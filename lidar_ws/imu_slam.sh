#!/bin/bash

# Script to run LiDAR + IMU SLAM using Cartographer

# Navigate to the lidar_ws directory
cd "$(dirname "$0")"

# Stop any existing ROS processes
pkill -f ros2

# Source ROS 2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the SLAM launch file
ros2 launch lidar_udp_receiver imu_slam_launch.py

# Clean exit
echo "Shutting down..."