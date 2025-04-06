#!/bin/bash
# Build the workspace with bilateration support

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Clean build artifacts
echo "Cleaning old build artifacts..."
colcon build --cmake-clean-cache --cmake-clean-first
rm -rf build/lidar_udp_receiver install/lidar_udp_receiver

# Build the package
echo "Building lidar_udp_receiver package..."
colcon build --packages-select lidar_udp_receiver

# Source the new setup
source install/setup.bash

echo "Build complete! You can now run:"
echo "  ./tri_slam.sh - for SLAM with trilateration (3 anchors)"
echo "  ./bi_slam.sh - for SLAM with bilateration (2 anchors)"
echo "  ./slam.sh - for regular SLAM (no positioning)"