# Installation Guide

This guide will walk you through setting up the LIDAR UDP Receiver package on your Ubuntu 22.04 system with ROS2 Humble.

## 1. Prerequisites

Ensure you have the following installed:

- Ubuntu 22.04 LTS
- ROS2 Humble (follow the [official ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html))
- Git

## 2. Create a ROS2 Workspace (if you don't already have one)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

## 3. Clone and Build the Package

```bash
cd ~/ros2_ws/src
```

Now, create a new package directory:

```bash
mkdir -p lidar_udp_receiver/src
mkdir -p lidar_udp_receiver/launch
mkdir -p lidar_udp_receiver/rviz
```

Copy the files from this repository to their respective locations:

1. Copy `lidar_udp_receiver_node.cpp` to `lidar_udp_receiver/src/`
2. Copy `lidar_udp_viz.launch.py` to `lidar_udp_receiver/launch/`
3. Copy `lidar_config.rviz` to `lidar_udp_receiver/rviz/`
4. Copy `CMakeLists.txt` to `lidar_udp_receiver/`
5. Copy `package.xml` to `lidar_udp_receiver/`

Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select lidar_udp_receiver
source ~/ros2_ws/install/setup.bash
```

## 4. Configure Your Raspberry Pi

Ensure your Raspberry Pi is running the LIDAR UDP publisher code provided in your original code snippet. The UDP publishing should be configured to send data to your Ubuntu machine's IP address.

On your Raspberry Pi, you can run the LIDAR publisher with:

```bash
sudo ./your_lidar_program --channel --serial /dev/ttyUSB0 115200 --publish <ubuntu_machine_ip> 8089
```

Replace `/dev/ttyUSB0` with the correct serial port for your LIDAR and `<ubuntu_machine_ip>` with the IP address of your Ubuntu machine running ROS2.

## 5. Run the ROS2 Node

Launch the ROS2 node and RViz for visualization:

```bash
ros2 launch lidar_udp_receiver lidar_udp_viz.launch.py
```

You should now see RViz start up with a visualization of your LIDAR data.

## 6. Troubleshooting

### Network Configuration

- Ensure both machines are on the same network and can ping each other
- Check that no firewall is blocking UDP traffic on port 8089
- Verify the IP address being used for publishing from the Raspberry Pi

### LIDAR Connection

- Make sure the LIDAR is properly connected to the Raspberry Pi
- Check that the correct serial port and baud rate are being used

### ROS2 Issues

- If you see TF errors in RViz, make sure the static_transform_publisher is running
- Try echoing the scan topic to see if data is being received: `ros2 topic echo /scan`
- Check if the UDP receiver is running without errors: `ros2 node list` should show `lidar_udp_receiver`