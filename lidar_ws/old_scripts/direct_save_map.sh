#!/bin/bash
# Directly save map from the /map topic

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# Create maps directory if it doesn't exist
MAPS_DIR=~/Documents/Spiderverse/maps
mkdir -p $MAPS_DIR

# Get current date and time for unique map name
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAP_NAME="map_$TIMESTAMP"

echo "Saving map as $MAP_NAME..."
echo "Map will be saved to $MAPS_DIR/$MAP_NAME.pgm and $MAPS_DIR/$MAP_NAME.yaml"

# Check if map topic exists
echo "Checking for /map topic..."
if ! ros2 topic list | grep -q "/map"; then
    echo "ERROR: /map topic not found! Make sure SLAM is running."
    echo "Available topics:"
    ros2 topic list
    exit 1
fi

# Directly save map using map_saver_cli
echo "Saving map from /map topic..."
ros2 run nav2_map_server map_saver_cli -f $MAPS_DIR/$MAP_NAME

# Check if map was saved
if [ -f "$MAPS_DIR/$MAP_NAME.pgm" ] && [ -s "$MAPS_DIR/$MAP_NAME.pgm" ]; then
    echo "Map saved successfully to: $MAPS_DIR/$MAP_NAME.pgm"
    
    # Display the map
    if command -v eog > /dev/null; then
        echo "Opening map with image viewer..."
        eog "$MAPS_DIR/$MAP_NAME.pgm" &
    fi
else
    echo "Failed to save map or map is empty."
    echo "This could be because:"
    echo "1. Cartographer isn't publishing a map yet"
    echo "2. The map is empty (no scan data)"
    echo "3. There's an issue with the map_saver_cli tool"
fi