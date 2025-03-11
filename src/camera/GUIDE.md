I am working on the Streaming of the live-feed and the challenging part about this project is to stream a video over wifi at 720p @ 30 Hz with a stero camera (Intel RealSense d435i). Due to the diffculty I have switched to a normal USB Webcam, that can stream 720p @ 30 Hz.

Intel RealSense d435i has alot of issues with it. This camera has 3 types: 2 are IR and 1 is RGB. With this the stream would consist of data from all the cameras including this version of the camera has IMU on it. I have tried to make my own node and attend to implement custime FASTDDS but it did not work so I have put this aside in the best interest of time and progress.

Follow this https://github.com/ros-drivers/usb_cam for more info about how to build the nodes.
