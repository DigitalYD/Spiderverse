#!/bin/bash
# =============================================================================
# ZERO_TO_360_VISUALIZE.SH - Run the 0° to 360° LiDAR visualizer
# =============================================================================
# This script runs a visualizer specially designed to convert the LiDAR data
# from the -180° to 180° format to a 0° to 360° format for better visualization.
# 
# Note: This requires the LiDAR data to be published on the /scan topic.
# You should have already started the LiDAR node with one of the following:
#   ./slam.sh, ./tri_slam.sh, ./bi_slam.sh, or ./lidar_and_rviz.sh
# 
# Usage: ./zero_to_360_visualize.sh [topic_name]
#        Default topic: /scan
# =============================================================================

# Default topic
TOPIC="/scan"

# Use custom topic if provided
if [ $# -ge 1 ]; then
    TOPIC="$1"
fi

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# Set Qt platform to xcb (X11) instead of Wayland
export QT_QPA_PLATFORM=xcb

# Run the visualizer
echo "Starting 0° to 360° LiDAR visualizer..."
echo "Listening on topic: $TOPIC"
ros2 run lidar_udp_receiver vizualize_lidar --ros-args -r scan:=$TOPIC