#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import struct
import time

# Callback when connected to MQTT broker
def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")
    client.subscribe("r")  # Subscribe to the robot command topic

# Callback when a message is received
def on_message(client, userdata, msg):
    # Check if we have the right amount of data
    if len(msg.payload) != 17:
        print(f"Error: Received {len(msg.payload)} bytes, expected 17")
        return
    
    # Unpack binary data
    try:
        # Extract joystick values (2 bytes each)
        move_x = struct.unpack("<h", msg.payload[0:2])[0]  # Little-endian 16-bit signed int
        move_y = struct.unpack("<h", msg.payload[2:4])[0]
        look_x = struct.unpack("<h", msg.payload[4:6])[0]
        look_y = struct.unpack("<h", msg.payload[6:8])[0]
        
        # Extract button states (1 byte total)
        button_byte = msg.payload[8]
        move_switch = bool(button_byte & (1 << 0))
        look_switch = bool(button_byte & (1 << 1))
        emote_button = bool(button_byte & (1 << 2))
        payload_drop = bool(button_byte & (1 << 3))
        gait_button = bool(button_byte & (1 << 4))
        mode_button = bool(button_byte & (1 << 5))
        
        # Extract percentage values (1 byte each)
        move_x_percent = msg.payload[9]
        move_y_percent = msg.payload[10]
        look_x_percent = msg.payload[11]
        look_y_percent = msg.payload[12]
        
        # Extract timestamp (4 bytes)
        timestamp = struct.unpack("<I", msg.payload[13:17])[0]  # Little-endian 32-bit unsigned int
        
        # Print decoded data (for debugging)
        print(f"Received at: {time.time():.3f}, ESP timestamp: {timestamp/1000:.3f}")
        print(f"Move: ({move_x}, {move_y}) [{move_x_percent}%, {move_y_percent}%]")
        print(f"Look: ({look_x}, {look_y}) [{look_x_percent}%, {look_y_percent}%]")
        print(f"Buttons: Move:{move_switch} Look:{look_switch} Emote:{emote_button} " 
              f"Payload:{payload_drop} Gait:{gait_button} Mode:{mode_button}")
        print("-" * 50)
        
        # Process the data for your robot here
        # Example: send_to_robot_controller(move_x, move_y, look_x, look_y, ...)
        
    except Exception as e:
        print(f"Error decoding message: {e}")

# Set up MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Connect to broker (adjust as needed)
client.connect("localhost", 1883, 60)

# Start the loop
print("Robot receiver is running...")
try:
    client.loop_forever()
except KeyboardInterrupt:
    print("Receiver stopped by user")
    client.disconnect()
