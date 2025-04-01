#!/bin/bash
# Very simple map viewing script

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# First clean up any existing processes
~/Documents/Spiderverse/lidar_ws/cleanup.sh

# Create a simple map directory if it doesn't exist
SIMPLE_MAP_DIR=~/Documents/Spiderverse/simple_test_map
mkdir -p $SIMPLE_MAP_DIR

# Create a simple map file if not exists
if [ ! -f "$SIMPLE_MAP_DIR/simple_map.pgm" ]; then
  echo "Creating a simple test map..."
  
  # Create a simple PGM file (P5 format - binary)
  cat > $SIMPLE_MAP_DIR/simple_map.pgm << 'EOF'
P2
# Simple square map
10 10
255
255 255 255 255 255 255 255 255 255 255
255 0   0   0   0   0   0   0   0   255
255 0   255 255 255 255 255 255 0   255
255 0   255 0   0   0   0   255 0   255
255 0   255 0   255 255 0   255 0   255
255 0   255 0   255 255 0   255 0   255
255 0   255 0   0   0   0   255 0   255
255 0   255 255 255 255 255 255 0   255
255 0   0   0   0   0   0   0   0   255
255 255 255 255 255 255 255 255 255 255
EOF
  
  # Create YAML metadata file
  cat > $SIMPLE_MAP_DIR/simple_map.yaml << EOF
image: $SIMPLE_MAP_DIR/simple_map.pgm
mode: trinary
resolution: 0.5
origin: [-2.5, -2.5, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
EOF
fi

echo "Simple map located at: $SIMPLE_MAP_DIR/simple_map.yaml"

# Define cleanup function
cleanup() {
  echo "Shutting down map viewer..."
  if [ -n "$MAP_SERVER_PID" ]; then
    kill -9 $MAP_SERVER_PID 2>/dev/null
  fi
  ~/Documents/Spiderverse/lidar_ws/cleanup.sh
  exit 0
}

# Set trap for cleanup
trap cleanup SIGINT SIGTERM EXIT

# Start map server with debug logging
echo "Starting map server..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$SIMPLE_MAP_DIR/simple_map.yaml -p frame_id:=map --log-level debug &
MAP_SERVER_PID=$!

# Wait for map server to start
sleep 2

# Verify map server is running
if ! ps -p $MAP_SERVER_PID > /dev/null; then
  echo "Map server failed to start!"
  exit 1
fi

# Check for map topic
echo "Verifying map topic exists..."
if ! ros2 topic list | grep -q "/map"; then
  echo "No /map topic found! Map server may not be working correctly."
  echo "Available topics:"
  ros2 topic list
  exit 1
fi

echo "Map topic found. Starting RViz2..."
echo "If you don't see the map in RViz2:"
echo "1. In the 'Displays' panel, check if 'Map' is enabled"
echo "2. Verify 'Fixed Frame' is set to 'map' in Global Options"
echo "3. Try clicking the 'Reset' button in the Views panel"

# Start RViz2 with our minimal config
ros2 run rviz2 rviz2 -d ~/Documents/Spiderverse/lidar_ws/minimal_map_view.rviz