#!/usr/bin/env python3

# GStreamer configuration
GSTREAMER_PORT = 5000
GSTREAMER_HOST = "192.168.0.112"  # Change to your Raspberry Pi's IP address
# MJPEG format with autovideosink (separate window) - this works reliably
GSTREAMER_PIPELINE = f"udpsrc port={GSTREAMER_PORT} caps=\"application/x-rtp,media=video,encoding-name=JPEG,payload=26\" ! rtpjitterbuffer latency=40 ! rtpjpegdepay ! jpegdec ! videoconvert ! autovideosink name=sink sync=false"

# ROS configuration for LiDAR
ROS_LIDAR_TOPIC = "/scan"  # Default LiDAR topic

# UI configuration
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
IMU_PANEL_HEIGHT = 150
VIDEO_PANEL_WIDTH = 700
LIDAR_PANEL_WIDTH = 500
