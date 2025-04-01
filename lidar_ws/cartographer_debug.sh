#!/bin/bash
# Debug script to isolate and fix Cartographer issues

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# First, clean up existing processes
echo "Cleaning up existing processes..."
~/Documents/Spiderverse/lidar_ws/cleanup.sh

# Function to check if process is running
check_process() {
    local pid=$1
    local name=$2
    if [ -z "$pid" ] || ! ps -p $pid > /dev/null; then
        echo "ERROR: $name process is not running!"
        return 1
    else
        echo "$name is running with PID $pid"
        return 0
    fi
}

# Define cleanup function
cleanup() {
    echo "Shutting down processes..."
    for pid in $LIDAR_PID $TF_PID $CART_PID $GRID_PID; do
        if [ -n "$pid" ] && ps -p $pid > /dev/null; then
            echo "Killing process $pid"
            kill -9 $pid 2>/dev/null
        fi
    done
    
    echo "Running final cleanup..."
    ~/Documents/Spiderverse/lidar_ws/cleanup.sh
    exit 0
}

# Set trap for clean shutdown
trap cleanup SIGINT SIGTERM EXIT

echo "=== STEP 1: Starting LiDAR UDP receiver ==="
ros2 launch lidar_udp_receiver lidar_receiver.launch.py &
LIDAR_PID=$!

# Wait and verify
sleep 3
if ! check_process $LIDAR_PID "LiDAR receiver"; then
    echo "Possible issues:"
    echo "- ROS2 is not properly installed or sourced"
    echo "- The lidar_udp_receiver package has issues"
    echo "- The launch file is missing or incorrect"
    exit 1
fi

echo "Checking for /scan topic..."
if ! ros2 topic list | grep -q "/scan"; then
    echo "WARNING: /scan topic not found! LiDAR may not be connected or sending data."
else
    echo "SUCCESS: /scan topic found!"
fi

echo "=== STEP 2: Starting TF broadcaster ==="
ros2 run lidar_udp_receiver tf_broadcaster &
TF_PID=$!

# Wait and verify
sleep 2
if ! check_process $TF_PID "TF broadcaster"; then
    echo "Possible issues:"
    echo "- The tf_broadcaster node is missing or has errors"
    echo "- The python module may have syntax errors"
    exit 1
fi

echo "Checking for /tf topic..."
if ! ros2 topic list | grep -q "/tf"; then
    echo "WARNING: /tf topic not found! TF broadcaster may not be working."
else
    echo "SUCCESS: /tf topic found!"
fi

# Create a very simple Cartographer config
CONFIG_DIR=~/Documents/Spiderverse/lidar_ws/src/lidar_udp_receiver/config
echo "Creating minimal Cartographer configuration..."
cat > $CONFIG_DIR/debug_config.lua << 'EOL'
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "lidar_link",
  published_frame = "lidar_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = false,
  num_laser_scans = 1,
  use_imu_data = false,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

return options
EOL

echo "=== STEP 3: Starting Cartographer SLAM with minimal config ==="
echo "Running Cartographer with debugging options..."
ros2 run cartographer_ros cartographer_node \
    -configuration_directory $CONFIG_DIR \
    -configuration_basename debug_config.lua --ros-args --log-level debug &
CART_PID=$!

# Wait and verify
sleep 5
if ! check_process $CART_PID "Cartographer"; then
    echo "ERROR: Cartographer failed to start!"
    echo "Likely causes:"
    echo "- Invalid configuration file"
    echo "- Missing dependencies"
    echo "- TF broadcaster not providing required transforms"
    echo ""
    echo "Trying a different approach..."
    
    # Print Cartographer version
    echo "Checking Cartographer version:"
    apt-cache policy ros-humble-cartographer-ros
    
    # Try with defaults
    echo "Starting Cartographer with default config..."
    ros2 run cartographer_ros cartographer_node \
        -configuration_directory $(ros2 pkg prefix cartographer_ros)/share/cartographer_ros/configuration_files \
        -configuration_basename backpack_2d.lua --ros-args --log-level debug &
    CART_PID=$!
    
    sleep 5
    if ! check_process $CART_PID "Cartographer with default config"; then
        echo "ERROR: Cartographer failed even with default config!"
        echo "This suggests issues with your ROS2 installation or Cartographer package."
        exit 1
    fi
fi

echo "Checking for submap topic..."
if ! ros2 topic list | grep -q "submap"; then
    echo "WARNING: No submap topics found! Cartographer may not be mapping correctly."
else
    echo "SUCCESS: Found submap topic!"
fi

echo "=== STEP 4: Starting Occupancy Grid Node ==="
ros2 run cartographer_ros occupancy_grid_node \
    -resolution 0.05 \
    -publish_period_sec 1.0 &
GRID_PID=$!

# Wait and verify
sleep 2
if ! check_process $GRID_PID "Occupancy Grid Node"; then
    echo "WARNING: Occupancy Grid Node failed to start!"
    echo "This means we won't be able to visualize the map."
    echo "But Cartographer is still mapping in the background."
fi

echo "Checking for map topic..."
if ! ros2 topic list | grep -q "/map"; then
    echo "WARNING: /map topic not found! Map visualization won't be available."
else
    echo "SUCCESS: /map topic found!"
fi

echo ""
echo "=== SUMMARY OF AVAILABLE TOPICS ==="
ros2 topic list | sort

echo ""
echo "=== ALL COMPONENTS STARTED ==="
echo "Cartographer is now mapping your environment."
echo "To save a map, run: ./submap_to_map.sh in another terminal"
echo "Press Ctrl+C when done mapping"
echo ""

# Keep the script running until Ctrl+C is pressed
# Print status update every 10 seconds
while true; do
    echo "Checking process status..."
    
    # Check each process
    check_process $LIDAR_PID "LiDAR receiver"
    check_process $TF_PID "TF broadcaster"
    check_process $CART_PID "Cartographer"
    check_process $GRID_PID "Occupancy Grid Node"
    
    echo "Mapping in progress... (Press Ctrl+C to stop)"
    
    # Wait for 10 seconds
    sleep 10
done