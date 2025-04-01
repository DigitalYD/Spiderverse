#!/bin/bash
# Script to view saved maps with robust cleanup

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# First run cleanup to ensure no existing processes
echo "Running cleanup to ensure no existing processes..."
~/Documents/Spiderverse/lidar_ws/cleanup.sh

# Check if a map file was provided
if [ $# -eq 0 ]; then
  # No map specified, find the most recent one
  echo "Looking for the most recent map file..."
  
  # Look for .yaml files first
  LATEST_MAP=$(find ~/Documents/Spiderverse/maps -name "*.yaml" -type f -printf "%T@ %p\n" | sort -nr | head -1 | cut -d' ' -f2-)
  
  # If no .yaml files, try looking for .pgm files
  if [ -z "$LATEST_MAP" ]; then
    LATEST_PGM=$(find ~/Documents/Spiderverse/maps -name "*.pgm" -type f -printf "%T@ %p\n" | sort -nr | head -1 | cut -d' ' -f2-)
    if [ -n "$LATEST_PGM" ]; then
      # Convert .pgm to .yaml path by changing extension
      LATEST_MAP="${LATEST_PGM%.pgm}.yaml"
      
      # If .yaml doesn't exist, create a simple one
      if [ ! -f "$LATEST_MAP" ]; then
        echo "Creating a simple YAML file for $LATEST_PGM"
        MAP_BASE=$(basename "$LATEST_PGM" .pgm)
        cat > "$LATEST_MAP" << EOF
image: $LATEST_PGM
resolution: 0.050000
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
EOF
      fi
    fi
  fi
  
  if [ -z "$LATEST_MAP" ]; then
    echo "No maps found in ~/Documents/Spiderverse/maps/"
    echo "Available files in maps directory:"
    ls -la ~/Documents/Spiderverse/maps/
    exit 1
  fi
  
  MAP_FILE=$LATEST_MAP
else
  # Use the specified map
  MAP_FILE=$1
  
  # Check if file exists
  if [ ! -f "$MAP_FILE" ]; then
    echo "Map file not found: $MAP_FILE"
    echo "Available map files:"
    find ~/Documents/Spiderverse/maps -name "*.yaml" -o -name "*.pgm" | sort
    exit 1
  fi
fi

echo "Loading map: $MAP_FILE"

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

# Start map server
echo "Starting map server..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP_FILE &
MAP_SERVER_PID=$!

# Wait for map server to start
sleep 2

# Check if map server is actually running
if ! ps -p $MAP_SERVER_PID > /dev/null; then
  echo "Map server failed to start. Check if the map file is valid."
  cleanup
  exit 1
fi

# Start RViz2
echo "Starting RViz2..."
ros2 run rviz2 rviz2 -d ~/Documents/Spiderverse/lidar_ws/src/lidar_udp_receiver/config/map_view.rviz

# RViz is now closed, clean up happens via EXIT trap