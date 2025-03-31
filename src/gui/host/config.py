#!/usr/bin/env python3

# GStreamer configuration
GSTREAMER_PORT = 5000
GSTREAMER_HOST = "192.168.0.115"  # Change to your Raspberry Pi's IP address
GSTREAMER_PIPELINE = f"udpsrc port={GSTREAMER_PORT} caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! rtph264depay ! h264parse ! decodebin ! videoconvert ! video/x-raw,format=RGB ! appsink name=sink emit-signals=True"

# MQTT configuration
MQTT_BROKER = "localhost"  # Change to your MQTT broker address
MQTT_PORT = 1883
MQTT_TOPIC_PI_STATS = "pi5/stats"
MQTT_TOPIC_IMU = "pi5/imu"
MQTT_CLIENT_ID = "pi5_dashboard"

# ROS configuration
ROS_LIDAR_TOPIC = "/scan"  # Default LiDAR topic

# UI configuration
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
STREAM_PANEL_WIDTH = 600
STATS_PANEL_WIDTH = 300
LIDAR_PANEL_WIDTH = 300
