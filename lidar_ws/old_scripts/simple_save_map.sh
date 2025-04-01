#!/bin/bash
# Simple map saving script that works with SLAM Toolbox

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
echo "Checking if map topic exists..."
if ros2 topic list | grep -q "/map"; then
    echo "Found /map topic, saving..."
    
    # Use map_saver from nav2_map_server
    ros2 run nav2_map_server map_saver_cli -f $MAPS_DIR/$MAP_NAME
    
    # Check if map was saved
    if [ -f "$MAPS_DIR/$MAP_NAME.pgm" ]; then
        echo "Map saved successfully!"
        echo "Map saved to: $MAPS_DIR/$MAP_NAME.pgm"
        echo "Map info saved to: $MAPS_DIR/$MAP_NAME.yaml"
    else
        echo "Failed to save map."
    fi
else
    echo "No /map topic found. Make sure SLAM is running."
fi

# Try using SLAM Toolbox's serialization method if available
if ros2 service list | grep -q "/slam_toolbox/serialize_map"; then
    echo "Found SLAM Toolbox serialization service, saving..."
    
    # Create directory for serialized maps
    SERIAL_DIR=$MAPS_DIR/serialized
    mkdir -p $SERIAL_DIR
    
    # Save serialized map
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$SERIAL_DIR/$MAP_NAME'}"
    
    echo "Serialized map saved to: $SERIAL_DIR/$MAP_NAME"
fi

# Show saved files
echo "Saved map files:"
find $MAPS_DIR -name "${MAP_NAME}*" -type f | sort

echo "Map saving process complete."