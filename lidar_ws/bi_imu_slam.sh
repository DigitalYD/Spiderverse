#!/bin/bash

# Script to run LiDAR + IMU + Bilateration SLAM using Cartographer

# Navigate to the lidar_ws directory
cd "$(dirname "$0")"

# Stop any existing ROS processes
pkill -f ros2

# Source ROS 2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the combined SLAM launch file
ros2 launch lidar_udp_receiver bilateration_imu_slam_launch.py

# Clean exit
echo "Shutting down..."