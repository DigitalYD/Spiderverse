# RPLidar Mapping System

This folder contains scripts for creating 2D maps using an RPLidar with ROS2 and Google Cartographer.

## Quick Start

1. **Start Mapping**
   ```bash
   # Basic SLAM (no positioning)
   ./slam.sh
   
   # SLAM with trilateration (3 anchors)
   ./tri_slam.sh
   
   # SLAM with bilateration (2 anchors)
   ./bi_slam.sh
   
   # SLAM with positioning options
   ./positioning_slam.sh [tri|bi|none]
   ```
   This starts the SLAM system. Move your RPLidar around to map the environment.

2. **Save a Map**
   ```bash
   ./save_map.sh
   ```
   Run this in a separate terminal while SLAM is running to save the current map.

3. **View Saved Maps**
   ```bash
   # View the most recent map
   ./view_maps.sh
   
   # List all available maps
   ./view_maps.sh list
   
   # View a specific map
   ./view_maps.sh /path/to/map.yaml
   ```

## Positioning Options

The system supports multiple positioning methods to enhance mapping:

- **Trilateration**: Uses 3 anchors (IDs 10, 11, 12) to determine position
  ```bash
  ./tri_slam.sh
  ```

- **Bilateration**: Uses 2 anchors (IDs 10, 11) to determine position
  ```bash
  ./bi_slam.sh
  ```

- **Direct Positioning**: Run only the positioning system without SLAM
  ```bash
  # Run trilateration node
  ros2 run lidar_udp_receiver trilateration_node
  
  # Run bilateration node
  ros2 run lidar_udp_receiver bilateration_node
  ```

## Additional Commands

- **Cleanup**: Stop all ROS2 processes
  ```bash
  ./cleanup.sh
  ```

## Maps Location

Maps are saved to:
```
~/Documents/Spiderverse/maps/
```

## Troubleshooting

1. If mapping isn't working:
   - Make sure your RPLidar is properly connected
   - Check that data is being received on the `/scan` topic

2. If map saving fails:
   - Make sure SLAM is running when you try to save
   - Try moving the RPLidar more to gather more data

3. If viewing maps fails:
   - Install EOG (Eye of GNOME): `sudo apt install eog`
   - Check that map files exist in the maps directory