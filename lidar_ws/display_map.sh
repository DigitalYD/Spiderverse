#!/bin/bash
# Script to display maps using EOG (Eye of GNOME) image viewer
# This avoids RViz2 entirely

# Source ROS2 setup (needed for the mapping part)
source /opt/ros/humble/setup.bash
source ~/Documents/Spiderverse/lidar_ws/install/setup.bash

# Create maps directory if it doesn't exist
MAPS_DIR=~/Documents/Spiderverse/maps
mkdir -p $MAPS_DIR

# Function to display a map
display_map() {
    local map_path="$1"
    
    # Check if the file exists
    if [ ! -f "$map_path" ]; then
        echo "ERROR: Map file not found: $map_path"
        return 1
    fi
    
    # Get file extension
    local ext="${map_path##*.}"
    
    # If it's a yaml file, extract the image path
    if [ "$ext" = "yaml" ]; then
        local image_path=$(grep "image:" "$map_path" | sed 's/image: //')
        
        # Check if the image path exists
        if [ ! -f "$image_path" ]; then
            echo "WARNING: Image file not found: $image_path"
            # Try to guess the image path by changing extension
            local base_path="${map_path%.*}"
            local pgm_path="${base_path}.pgm"
            
            if [ -f "$pgm_path" ]; then
                echo "Found PGM file: $pgm_path"
                image_path="$pgm_path"
            else
                echo "ERROR: Cannot find image file for YAML: $map_path"
                return 1
            fi
        fi
        
        # Display the map's metadata
        echo "Map metadata from $map_path:"
        cat "$map_path"
        echo ""
        
        # Open the image file
        echo "Opening map image: $image_path"
        eog "$image_path" &
    else
        # Open the image file directly
        echo "Opening map image: $map_path"
        eog "$map_path" &
    fi
}

# Find the latest map file by default
find_latest_map() {
    local latest_yaml=$(find "$MAPS_DIR" -name "*.yaml" -type f -printf "%T@ %p\n" | sort -nr | head -1 | cut -d' ' -f2-)
    
    if [ -n "$latest_yaml" ]; then
        echo "$latest_yaml"
        return 0
    fi
    
    # If no yaml found, look for pgm
    local latest_pgm=$(find "$MAPS_DIR" -name "*.pgm" -type f -printf "%T@ %p\n" | sort -nr | head -1 | cut -d' ' -f2-)
    
    if [ -n "$latest_pgm" ]; then
        echo "$latest_pgm"
        return 0
    fi
    
    # No maps found
    return 1
}

# Display all available maps
list_maps() {
    echo "Available maps in $MAPS_DIR:"
    echo "---------------------------------"
    
    local yaml_files=$(find "$MAPS_DIR" -name "*.yaml" -type f)
    local pgm_files=$(find "$MAPS_DIR" -name "*.pgm" -type f | grep -v "$(echo "$yaml_files" | sed 's/\.yaml/\.pgm/g')")
    
    if [ -n "$yaml_files" ]; then
        echo "YAML Map Files (with metadata):"
        for file in $yaml_files; do
            local timestamp=$(date -r "$file" "+%Y-%m-%d %H:%M:%S")
            echo "- $file (Modified: $timestamp)"
        done
        echo ""
    fi
    
    if [ -n "$pgm_files" ]; then
        echo "PGM Image Files (without metadata):"
        for file in $pgm_files; do
            local timestamp=$(date -r "$file" "+%Y-%m-%d %H:%M:%S")
            local filesize=$(du -h "$file" | cut -f1)
            echo "- $file (Modified: $timestamp, Size: $filesize)"
        done
        echo ""
    fi
    
    if [ -z "$yaml_files" ] && [ -z "$pgm_files" ]; then
        echo "No map files found in $MAPS_DIR"
        return 1
    fi
}

# Main script logic
case "$1" in
    "list")
        list_maps
        ;;
    "")
        # No arguments, find and display the latest map
        echo "Looking for latest map..."
        latest_map=$(find_latest_map)
        
        if [ $? -eq 0 ]; then
            echo "Found latest map: $latest_map"
            display_map "$latest_map"
        else
            echo "No maps found in $MAPS_DIR"
            exit 1
        fi
        ;;
    *)
        # Argument is a specific map to display
        display_map "$1"
        ;;
esac