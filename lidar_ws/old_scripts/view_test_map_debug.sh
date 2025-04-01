#!/bin/bash
# Script to view the test map with detailed debugging

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
echo "Map contents:"
cat "$TEST_MAP"

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

# Start map server with explicit parameters and debugging
echo "Starting map server with debugging..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$TEST_MAP -p frame_id:=map -p use_sim_time:=false --log-level info &
MAP_SERVER_PID=$!

# Wait for map server to start
sleep 3

# Print all topics to verify map is being published
echo "Checking published topics..."
ros2 topic list

# Print map topic info
echo "Map topic details:"
ros2 topic info /map

# Echo a message from the map topic to verify data
echo "Trying to get a message from map topic..."
timeout 5s ros2 topic echo /map --once

# Check if map server is actually running
if ! ps -p $MAP_SERVER_PID > /dev/null; then
  echo "Map server failed to start. Check if the map file is valid."
  cleanup
  exit 1
fi

# Create and use a minimal RViz config
RVIZ_CONFIG=$(mktemp --suffix=.rviz)
cat > $RVIZ_CONFIG << 'EOF'
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Reference Frame: <Fixed Frame>
    - Class: rviz_default_plugins/Map
      Name: Map
      Topic:
        Value: /map
        Depth: 5
        Reliability Policy: Reliable
        History Policy: Keep Last
      Enabled: true
      Draw Behind: false
      Use Timestamp: false
      Color Scheme: map
  Global Options:
    Fixed Frame: map
    Background Color: 48; 48; 48
    Frame Rate: 30
EOF

echo "Starting RViz2 with minimal config..."
echo "RViz config at: $RVIZ_CONFIG"
ros2 run rviz2 rviz2 -d $RVIZ_CONFIG --fixed-frame map

# Clean up happens via the EXIT trap
