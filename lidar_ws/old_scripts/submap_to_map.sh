#!/bin/bash
# Script to convert submaps to a map and display it

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# Create maps directory if it doesn't exist
MAPS_DIR=~/Documents/Spiderverse/maps
mkdir -p $MAPS_DIR

# Get current date and time for unique map name
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAP_NAME="map_$TIMESTAMP"

echo "Getting map from submaps as $MAP_NAME..."
echo "Map will be saved to $MAPS_DIR/$MAP_NAME.pgm and $MAPS_DIR/$MAP_NAME.yaml"

# Check if submap_list topic exists
echo "Checking for /submap_list topic..."
if ! ros2 topic list | grep -q "/submap_list"; then
    echo "ERROR: /submap_list topic not found! Make sure Cartographer is running."
    echo "Available topics:"
    ros2 topic list
    exit 1
fi

# First, we need to start a node that subscribes to /submap_list and publishes to /map
echo "Starting submap conversion node..."
ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=false &
MAP_SERVER_PID=$!

# Wait for the map server to start
sleep 3

# Check if /map topic now exists
if ! ros2 topic list | grep -q "/map"; then
    echo "WARNING: /map topic still not found after starting map server."
    echo "Trying alternative approach..."
    
    # Kill the map server
    kill $MAP_SERVER_PID 2>/dev/null
    
    # Try to directly get the map from Cartographer
    echo "Saving Cartographer state as pbstream..."
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" || echo "Failed to finish trajectory (this can be normal)"
    sleep 1
    
    # Save the state
    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$MAPS_DIR/$MAP_NAME.pbstream'}" || echo "Failed to write state"
    
    # Check if pbstream was saved
    if [ -f "$MAPS_DIR/$MAP_NAME.pbstream" ] && [ -s "$MAPS_DIR/$MAP_NAME.pbstream" ]; then
        echo "Saved Cartographer state to: $MAPS_DIR/$MAP_NAME.pbstream"
        
        # Convert pbstream to pgm
        echo "Converting pbstream to pgm..."
        ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
            -pbstream_filename $MAPS_DIR/$MAP_NAME.pbstream \
            -map_filestem $MAPS_DIR/$MAP_NAME
    else
        echo "Failed to save Cartographer state."
        exit 1
    fi
else
    # Save the map now that it exists
    echo "Map topic found! Saving map..."
    ros2 run nav2_map_server map_saver_cli -f $MAPS_DIR/$MAP_NAME
    
    # Kill the map server
    kill $MAP_SERVER_PID 2>/dev/null
fi

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
    
    # Try yet another approach - convert submaps directly using Cartographer tools
    echo "Trying one more approach with occupancy_grid_node..."
    
    # Start the occupancy grid node if it's not running
    if ! ros2 topic list | grep -q "/map"; then
        ros2 run cartographer_ros occupancy_grid_node -resolution 0.05 &
        GRID_PID=$!
        
        # Wait for the occupancy grid node to start
        sleep 3
        
        # Check if /map topic now exists
        if ros2 topic list | grep -q "/map"; then
            echo "Map topic now available! Saving map..."
            ros2 run nav2_map_server map_saver_cli -f $MAPS_DIR/$MAP_NAME
            
            # Kill the occupancy grid node
            kill $GRID_PID 2>/dev/null
            
            # Check if map was saved
            if [ -f "$MAPS_DIR/$MAP_NAME.pgm" ] && [ -s "$MAPS_DIR/$MAP_NAME.pgm" ]; then
                echo "Map saved successfully to: $MAPS_DIR/$MAP_NAME.pgm"
                
                # Display the map
                if command -v eog > /dev/null; then
                    echo "Opening map with image viewer..."
                    eog "$MAPS_DIR/$MAP_NAME.pgm" &
                fi
            else
                echo "Failed to save map again."
            fi
        else
            echo "Still no map topic available."
            kill $GRID_PID 2>/dev/null
        fi
    fi
fi

echo "Process completed."