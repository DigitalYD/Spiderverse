#!/bin/bash
# Script to create a simple test map and test it

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Clean up any existing processes
~/Documents/Spiderverse/lidar_ws/cleanup.sh

# Create test map directory
TEST_DIR=~/Documents/Spiderverse/simple_test_map
mkdir -p $TEST_DIR

# Create a very simple map - just a black and white image
cd $TEST_DIR
echo "Creating simple test map..."

# Create a simple PGM file directly (P2 format - ASCII)
cat > simple_map.pgm << 'EOF'
P2
# Simple test map
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

# Create YAML metadata file for the map
cat > simple_map.yaml << EOF
image: $TEST_DIR/simple_map.pgm
resolution: 0.5
origin: [-2.5, -2.5, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
EOF

echo "Test map created at: $TEST_DIR/simple_map.pgm"
echo "Map YAML at: $TEST_DIR/simple_map.yaml"

# Define cleanup function
cleanup() {
  echo "Shutting down..."
  if [ -n "$MAP_SERVER_PID" ]; then
    kill -9 $MAP_SERVER_PID 2>/dev/null
  fi
  ~/Documents/Spiderverse/lidar_ws/cleanup.sh
  exit 0
}

# Set up trap
trap cleanup SIGINT SIGTERM EXIT

# Start map server
echo "Starting map server..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$TEST_DIR/simple_map.yaml -p frame_id:=map --log-level debug &
MAP_SERVER_PID=$!

# Wait for map server to start
sleep 3

# Check published topics
echo "Checking for map topic..."
ros2 topic list | grep map

# Try to get map info
echo "Map topic info:"
ros2 topic info /map

# Now run RViz2 to visualize
echo "Starting RViz2 (you should see a small map in the center)..."
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz
