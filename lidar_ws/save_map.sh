#!/bin/bash
# Save as ~/save_map.sh

# Source ROS2 and workspace setup
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

# First check what map topics are available
MAP_TOPICS=$(ros2 topic list | grep -E "map|grid")
echo "Available map topics: $MAP_TOPICS"

# Try to save using standard method with timeout
echo "Attempting to save map using standard map_saver..."
timeout 5 ros2 run nav2_map_server map_saver_cli -f $MAPS_DIR/$MAP_NAME

# Check if the map was saved
if [ -f "$MAPS_DIR/$MAP_NAME.pgm" ]; then
    echo "Map saved successfully!"
else
    echo "Standard map saving failed. Trying Cartographer-specific method..."
    
    # Try using Cartographer's built-in functionality
    echo "Saving Cartographer state..."
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" || echo "Failed to finish trajectory"
    
    sleep 1
    
    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$MAPS_DIR/$MAP_NAME.pbstream'}" || echo "Failed to write state"
    
    if [ -f "$MAPS_DIR/$MAP_NAME.pbstream" ]; then
        echo "Cartographer state saved. Converting to map..."
        ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
            -pbstream_filename $MAPS_DIR/$MAP_NAME.pbstream \
            -map_filestem $MAPS_DIR/$MAP_NAME
            
        if [ -f "$MAPS_DIR/$MAP_NAME.pgm" ]; then
            echo "Map converted successfully!"
        else
            echo "Failed to convert pbstream to map."
        fi
    else
        echo "Failed to save Cartographer state."
    fi
fi

# Print final result
if [ -f "$MAPS_DIR/$MAP_NAME.pgm" ]; then
    echo "Map saved to: $MAPS_DIR/$MAP_NAME.pgm"
    echo "Map info saved to: $MAPS_DIR/$MAP_NAME.yaml"
else
    echo "Failed to save map using all available methods."
    
    # Last resort - try manual republishing
    echo "Attempting to save with explicit topic subscription..."
    
    # Find the actual map topic
    ACTUAL_MAP_TOPIC=$(ros2 topic list | grep -E "map|grid" | head -1)
    
    if [ ! -z "$ACTUAL_MAP_TOPIC" ]; then
        echo "Found potential map topic: $ACTUAL_MAP_TOPIC"
        # Try to save from this specific topic
        ros2 run nav2_map_server map_saver_cli -f $MAPS_DIR/$MAP_NAME --ros-args -r /map:=$ACTUAL_MAP_TOPIC
    else
        echo "No map topics found. Make sure Cartographer is running properly."
    fi
fi