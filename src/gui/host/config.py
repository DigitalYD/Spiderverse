#!/usr/bin/env python3

# GStreamer configuration
GSTREAMER_PORT = 5000
GSTREAMER_HOST = "192.168.0.113"  # Change to your Raspberry Pi's IP address
GSTREAMER_PIPELINE = f"udpsrc port={GSTREAMER_PORT} caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! rtph264depay ! h264parse ! decodebin ! videoconvert ! video/x-raw,format=RGB ! autovideosink name=sink"

# ROS configuration for LiDAR
ROS_LIDAR_TOPIC = "/scan"  # Default LiDAR topic

# UI configuration
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
IMU_PANEL_HEIGHT = 150
VIDEO_PANEL_WIDTH = 700
LIDAR_PANEL_WIDTH = 500