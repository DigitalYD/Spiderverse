# Stream Test
I am working on the Streaming of the live-feed and the challenging part about this project is to stream a video over WIFI at 720p @ 30 Hz with any camera. The initial plan was to use a stero camera (Intel RealSense d435i) for its ability of depth. That data would be used for the 3D rendering of a 3D space. Due to the diffculty I have switched to a normal USB Webcam, that can stream 720p @ 30 Hz.

## Background
To be able to stream livefeed with  any cameraI would need a communiation method that has high bandwidth, tis unltimatly leads to use WIFI to stream the video over other protcols such as Bluetooth, Zigbee, LoRA, and etc. 

With WIFI there are two protcal calls UDP and TCP, for something like streaming a video TCP is needed for its high speed with reliabliliuty. But the issue with TCP it needs to have more ACK check for each package sent and recieve so it would generate a lag with you are streaming. So using UDP is the best option. With the UDP the only software that uses it is the GStreamer. With testing on TCP software such as PYZMQ and ROS2's DDS. Both generated great resolution and frame recovery but terrible FPS (about 9 ~ 15) with this it is not a subtainable method for a live feed. The using the UDP software GStreamer it has deliver great FPS (about 15 ~ 23) with 640 x 480 p, the overall decent frame recovery that does not make it laggy also with the properties of UDP it would require less overhead.

## Intel RealSense d435i:

This has alot of issues with it. This camera has 3 types: 2 are IR and 1 is RGB. With this the stream would consist of data from all the cameras including this version of the camera has IMU on it. I have tried to make my own node and attend to implement custime FASTDDS but it did not work so I have put this aside in the best interest of time and progress. 

After further testing and switching to a Pi5 with a M.2 SSD PiHat the improvement of the stream as increased by a tiny bit with the current method. This hardware change has shown that the issue is limited by the throughput by the router and not the currenting computing hardware. 

## USB-Camera:

Follow this https://github.com/ros-drivers/usb_cam for more info about how to build the nodes. 