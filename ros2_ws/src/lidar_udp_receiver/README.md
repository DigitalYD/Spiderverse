# LIDAR UDP Receiver for ROS2

This ROS2 package receives LIDAR data sent over UDP from a Raspberry Pi and visualizes it in RViz.

## Overview

The package consists of a UDP receiver node that:
1. Listens for UDP packets containing LIDAR scan data
2. Converts the received data into ROS2 LaserScan messages
3. Publishes these messages to a ROS2 topic for visualization in RViz

## Prerequisites

- ROS2 Humble on Ubuntu 22.04
- A SLAMTEC LIDAR connected to a Raspberry Pi running the UDP publisher code

## Installation

1. Clone this package into your ROS2 workspace's `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> lidar_udp_receiver
   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select lidar_udp_receiver
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

Launch the UDP receiver and RViz visualization:

```bash
ros2 launch lidar_udp_receiver lidar_udp_viz.launch.py
```

### Launch Parameters

- `port` (default: 8089): UDP port to listen for LIDAR data
- `frame_id` (default: lidar_link): TF frame ID for the LIDAR data

Example with custom parameters:
```bash
ros2 launch lidar_udp_receiver lidar_udp_viz.launch.py port:=8090 frame_id:=my_lidar
```

## Node Details

### lidar_udp_receiver_node

This node listens for UDP data on the specified port and converts it to LaserScan messages.

#### Published Topics

- `/scan` ([sensor_msgs/LaserScan](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html)): The LIDAR scan data

#### Parameters

- `port` (int, default: 8089): UDP port to listen for LIDAR data
- `frame_id` (string, default: lidar_link): TF frame ID for the LIDAR data
- `scan_topic` (string, default: scan): Topic to publish LaserScan messages
- `angle_min` (double, default: -π): Minimum angle of the scan [rad]
- `angle_max` (double, default: π): Maximum angle of the scan [rad]
- `angle_increment` (double, default: 0.0174533): Angular resolution [rad]
- `range_min` (double, default: 0.15): Minimum range [m]
- `range_max` (double, default: 12.0): Maximum range [m]
- `scan_time` (double, default: 0.1): Time between scans [s]
- `publish_rate` (double, default: 10.0): Rate at which to publish scan messages [Hz]

## Data Format

The UDP packets from the Raspberry Pi should contain:
- 8-byte timestamp (uint64_t)
- 4-byte point count (uint32_t)
- Array of scan points, each containing:
  - 4-byte angle (float, degrees)
  - 4-byte distance (float, millimeters)
  - 4-byte quality (uint32_t)
  - 4-byte flag (uint32_t)

## Troubleshooting

### No data received
- Verify that the Raspberry Pi is running and publishing data
- Check that the UDP port matches between the Raspberry Pi publisher and this receiver
- Ensure there are no firewall rules blocking UDP traffic on the specified port

### RViz not showing data
- Verify that the LaserScan is being published: `ros2 topic echo /scan`
- Check that the correct frame is selected in RViz
- Ensure that the TF transform from base_link to lidar_link is being published