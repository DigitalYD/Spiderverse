#!/bin/bash
# =============================================================================
# VIEW_MAPS.SH - View saved maps from Cartographer
# =============================================================================
# This script:
# 1. Lists all saved maps
# 2. Lets you view a specific map or the most recent one
# 3. Opens maps directly in an image viewer (EOG)
#
# Usage: 
#   ./view_maps.sh           - View the most recent map
#   ./view_maps.sh list      - List all available maps
#   ./view_maps.sh [path]    - View a specific map file
# =============================================================================

# Source ROS2 setup (needed just in case)
source /opt/ros/humble/setup.bash

# Maps directory
MAPS_DIR=~/Documents/Spiderverse/maps

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
        # Display the map's metadata
        echo "Map metadata from $map_path:"
        cat "$map_path"
        echo ""
        
        # Extract image path
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
        
        # Open the image file
        echo "Opening map image: $image_path"
        eog "$image_path" &
    else
        # Open the image file directly
        echo "Opening map image: $map_path"
        eog "$map_path" &
    fi
}

# Find the latest map file
find_latest_map() {
    # First try to find yaml files
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

# List all available maps
list_maps() {
    echo "================ AVAILABLE MAPS ================"
    if [ ! -d "$MAPS_DIR" ] || [ -z "$(ls -A $MAPS_DIR 2>/dev/null)" ]; then
        echo "No maps found in $MAPS_DIR"
        return 1
    fi
    
    echo "Maps in $MAPS_DIR:"
    
    # Find all yaml map files
    local yaml_files=$(find "$MAPS_DIR" -name "*.yaml" -type f)
    if [ -n "$yaml_files" ]; then
        echo "Map Files (with metadata):"
        echo "--------------------------"
        for file in $yaml_files; do
            local timestamp=$(date -r "$file" "+%Y-%m-%d %H:%M:%S")
            echo "- $file (Created: $timestamp)"
        done
        echo ""
    fi
    
    # Find pgm files that don't have corresponding yaml files
    local pgm_files=$(find "$MAPS_DIR" -name "*.pgm" -type f)
    if [ -n "$pgm_files" ]; then
        echo "Image Files (may lack metadata):"
        echo "-------------------------------"
        for file in $pgm_files; do
            local timestamp=$(date -r "$file" "+%Y-%m-%d %H:%M:%S")
            local size=$(du -h "$file" | cut -f1)
            echo "- $file (Created: $timestamp, Size: $size)"
        done
    fi
    
    echo "================================================"
    return 0
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