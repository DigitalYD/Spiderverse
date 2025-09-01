#!/bin/bash
# =============================================================================
# SAVE_MAP.SH - Save the current map from Cartographer
# =============================================================================
# This script:
# 1. Saves the map from Cartographer SLAM
# 2. Tries multiple methods to ensure a successful save
# 3. Opens the saved map in an image viewer
#
# Usage: ./save_map.sh
# Run this while slam.sh is still running in another terminal
# =============================================================================

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# Create maps directory if it doesn't exist
MAPS_DIR=~/Documents/Spiderverse/maps
mkdir -p $MAPS_DIR

# Get current date and time for unique map name
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAP_NAME="map_$TIMESTAMP"

# Check if SLAM is running (look for Cartographer node)
if ! pgrep -f "cartographer_node" > /dev/null; then
    echo "ERROR: Cartographer doesn't appear to be running!"
    echo "Please make sure slam.sh is running in another terminal."
    exit 1
fi

# Banner
echo "============================================"
echo "Saving map as: $MAP_NAME"
echo "Location: $MAPS_DIR/$MAP_NAME.pgm"
echo "============================================"

# Try direct map saving first
echo "STEP 1: Checking for map topic..."
if ros2 topic list | grep -q "/map"; then
    echo "Map topic found! Saving map directly..."
    ros2 run nav2_map_server map_saver_cli -f $MAPS_DIR/$MAP_NAME
    
    # Check if map was saved successfully
    if [ -f "$MAPS_DIR/$MAP_NAME.pgm" ] && [ -s "$MAPS_DIR/$MAP_NAME.pgm" ]; then
        echo "✓ Map saved successfully via map topic!"
        
        # Check if yaml was created, if not create it
        if [ ! -f "$MAPS_DIR/$MAP_NAME.yaml" ]; then
            echo "Creating YAML metadata file..."
            cat > "$MAPS_DIR/$MAP_NAME.yaml" << EOF
image: $MAPS_DIR/$MAP_NAME.pgm
mode: trinary
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
EOF
            echo "✓ YAML metadata file created!"
        fi
    else
        echo "✗ Failed to save map directly."
    fi
else
    echo "No /map topic found. Moving to next method..."
fi

# If map wasn't saved or is empty, try Cartographer state method
if [ ! -f "$MAPS_DIR/$MAP_NAME.pgm" ] || [ ! -s "$MAPS_DIR/$MAP_NAME.pgm" ]; then
    echo "STEP 2: Saving Cartographer state..."
    
    # Finish trajectory first
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" || echo "Note: Trajectory may already be finished"
    sleep 1
    
    # Save state
    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$MAPS_DIR/$MAP_NAME.pbstream'}" || echo "Failed to write state"
    
    # Check if pbstream was saved
    if [ -f "$MAPS_DIR/$MAP_NAME.pbstream" ] && [ -s "$MAPS_DIR/$MAP_NAME.pbstream" ]; then
        echo "Cartographer state saved. Converting to map..."
        
        # Convert pbstream to pgm
        ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
            -pbstream_filename $MAPS_DIR/$MAP_NAME.pbstream \
            -map_filestem $MAPS_DIR/$MAP_NAME
            
        if [ -f "$MAPS_DIR/$MAP_NAME.pgm" ] && [ -s "$MAPS_DIR/$MAP_NAME.pgm" ]; then
            echo "✓ Map converted successfully from pbstream!"
            
            # Check if yaml was created, if not create it
            if [ ! -f "$MAPS_DIR/$MAP_NAME.yaml" ]; then
                echo "Creating YAML metadata file..."
                cat > "$MAPS_DIR/$MAP_NAME.yaml" << EOF
image: $MAPS_DIR/$MAP_NAME.pgm
mode: trinary
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
EOF
                echo "✓ YAML metadata file created!"
            fi
        else
            echo "✗ Failed to convert pbstream to map."
        fi
    else
        echo "✗ Failed to save Cartographer state."
    fi
fi

# Final check and display
if [ -f "$MAPS_DIR/$MAP_NAME.pgm" ] && [ -s "$MAPS_DIR/$MAP_NAME.pgm" ]; then
    echo "============================================"
    echo "SUCCESS: Map saved successfully!"
    echo "Map file: $MAPS_DIR/$MAP_NAME.pgm"
    echo "YAML file: $MAPS_DIR/$MAP_NAME.yaml"
    echo "============================================"
    
    # Open map in image viewer
    if command -v eog > /dev/null; then
        echo "Opening map in image viewer..."
        eog "$MAPS_DIR/$MAP_NAME.pgm" &
    else
        echo "No image viewer found. You can open the map manually."
    fi
else
    echo "============================================"
    echo "ERROR: Failed to save a usable map file."
    echo "Possible issues:"
    echo "1. Cartographer is not mapping properly"
    echo "2. The map is empty (not enough scan data)"
    echo "3. The conversion tools failed"
    echo "============================================"
fi