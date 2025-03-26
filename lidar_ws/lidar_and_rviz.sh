#!/bin/bash
# Save as ~/lidar_and_rviz.sh

# Start the LIDAR UDP receiver in the background
source ~/lidar_ws/install/setup.bash
ros2 launch lidar_udp_receiver lidar_receiver.launch.py &
LIDAR_PID=$!

# Sleep briefly to ensure the LIDAR node starts up first
sleep 2

# Run RViz2 with the isolated environment
env -i \
  HOME=$HOME \
  USER=$USER \
  DISPLAY=$DISPLAY \
  XAUTHORITY=$XAUTHORITY \
  DBUS_SESSION_BUS_ADDRESS=$DBUS_SESSION_BUS_ADDRESS \
  XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  PATH=/usr/bin:/bin:/usr/local/bin \
  LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu \
  bash -c 'source /opt/ros/humble/setup.bash && source ~/lidar_ws/install/setup.bash && exec /opt/ros/humble/bin/rviz2 -d ~/lidar_ws/src/lidar_udp_receiver/config/lidar.rviz'

# When RViz2 is closed, kill the LIDAR process
kill $LIDAR_PID