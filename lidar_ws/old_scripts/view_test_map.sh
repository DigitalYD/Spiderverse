#!/bin/bash
# Script to view the test map

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# First run cleanup to ensure no existing processes
echo "Running cleanup to ensure no existing processes..."
~/Documents/Spiderverse/lidar_ws/cleanup.sh

TEST_MAP=~/Documents/Spiderverse/test_maps/test_map.yaml

if [ ! -f "$TEST_MAP" ]; then
  echo "Test map not found: $TEST_MAP"
  exit 1
fi

echo "Loading test map: $TEST_MAP"

# Define cleanup function for clean shutdown
cleanup() {
  echo "Shutting down map viewer..."
  if [ -n "$MAP_SERVER_PID" ]; then
    kill $MAP_SERVER_PID 2>/dev/null
  fi
  
  # Run final cleanup to ensure all processes are gone
  ~/Documents/Spiderverse/lidar_ws/cleanup.sh
  exit 0
}

# Set up trap for clean shutdown
trap cleanup SIGINT SIGTERM EXIT

# Start map server with verbose output
echo "Starting map server..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$TEST_MAP -p use_sim_time:=false &
MAP_SERVER_PID=$!

# Wait for map server to start
sleep 2

# Print topic list to verify map is being published
echo "Checking published topics..."
ros2 topic list

# Check if map server is actually running
if ! ps -p $MAP_SERVER_PID > /dev/null; then
  echo "Map server failed to start. Check if the map file is valid."
  cleanup
  exit 1
fi

# Start RViz2 with specific settings
echo "Starting RViz2..."
ros2 run rviz2 rviz2 -d ~/Documents/Spiderverse/lidar_ws/src/lidar_udp_receiver/config/map_view.rviz --fixed-frame map

# RViz is now closed, clean up happens via EXIT trap