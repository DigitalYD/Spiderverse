#!/bin/bash
# Save the most recent map from Cartographer
# This script does not rely on RViz2

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

# Check if Cartographer is running
if pgrep -f "cartographer_node" > /dev/null; then
    echo "Cartographer is running, trying to save map directly..."
    
    # Save the map using map_saver_cli
    echo "Saving map directly from /map topic..."
    timeout 5s ros2 run nav2_map_server map_saver_cli -f $MAPS_DIR/$MAP_NAME
    
    # Check if map was saved successfully
    if [ -f "$MAPS_DIR/$MAP_NAME.pgm" ] && [ -s "$MAPS_DIR/$MAP_NAME.pgm" ]; then
        echo "Map saved successfully!"
    else
        echo "Failed to save map directly. Trying to save pbstream..."
        
        # Finish trajectory
        timeout 5s ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" || echo "Failed to finish trajectory (this can be normal)"
        
        sleep 1
        
        # Save Cartographer state
        timeout 5s ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$MAPS_DIR/$MAP_NAME.pbstream'}" || echo "Failed to write state"
        
        # Check if pbstream was saved
        if [ -f "$MAPS_DIR/$MAP_NAME.pbstream" ] && [ -s "$MAPS_DIR/$MAP_NAME.pbstream" ]; then
            echo "Saved Cartographer state to: $MAPS_DIR/$MAP_NAME.pbstream"
            
            # Try to convert pbstream to pgm
            echo "Converting pbstream to pgm..."
            timeout 10s ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
                -pbstream_filename $MAPS_DIR/$MAP_NAME.pbstream \
                -map_filestem $MAPS_DIR/$MAP_NAME || echo "Failed to convert pbstream"
                
            if [ -f "$MAPS_DIR/$MAP_NAME.pgm" ] && [ -s "$MAPS_DIR/$MAP_NAME.pgm" ]; then
                echo "Map converted successfully!"
            else
                echo "Failed to convert pbstream to map. Using screenshot method..."
                
                # Create a metadata file for the pbstream at least
                echo "Creating info file for the pbstream..."
                cat > $MAPS_DIR/$MAP_NAME.pbstream.info << EOF
This is a Cartographer pbstream file saved on $(date).
To view this map, you need to convert it to a pgm file using:

ros2 run cartographer_ros cartographer_pbstream_to_ros_map \\
  -pbstream_filename $MAPS_DIR/$MAP_NAME.pbstream \\
  -map_filestem $MAPS_DIR/$MAP_NAME
EOF
            fi
        else
            echo "Failed to save Cartographer state."
        fi
    fi
else
    echo "Cartographer is not running. Taking a screenshot instead..."
fi

# List all created files
echo "Created map files:"
find $MAPS_DIR -name "${MAP_NAME}*" | sort

# If map was saved successfully, display it
if [ -f "$MAPS_DIR/$MAP_NAME.pgm" ] && [ -s "$MAPS_DIR/$MAP_NAME.pgm" ]; then
    echo "Map saved successfully! Opening with image viewer..."
    # Check if eog is installed
    if command -v eog > /dev/null; then
        eog "$MAPS_DIR/$MAP_NAME.pgm" &
    else
        echo "No image viewer found. You can open the map manually at: $MAPS_DIR/$MAP_NAME.pgm"
    fi
else
    echo "Could not create a viewable map file."
    echo "Check the logs for errors."
fi