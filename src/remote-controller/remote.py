#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import struct
import time
import logging

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Constants
EXPECTED_PAYLOAD_SIZE = 17
TOPIC = "r"

class Controller:
    """Handles controller data processing and robot control"""
    
    def __init__(self):
        self.last_command_time = 0
        self.message_count = 0
        
    def process_command(self, payload):
        """Process binary controller data"""
        if len(payload) != EXPECTED_PAYLOAD_SIZE:
            logger.warning(f"Invalid payload size: {len(payload)} bytes, expected {EXPECTED_PAYLOAD_SIZE}")
            return
            
        try:
            # Track message receipt
            self.message_count += 1
            
            # Extract joystick values (2 bytes each)
            move_x = struct.unpack("<h", payload[0:2])[0]
            move_y = struct.unpack("<h", payload[2:4])[0]
            look_x = struct.unpack("<h", payload[4:6])[0]
            look_y = struct.unpack("<h", payload[6:8])[0]
            
            # Extract button states (1 byte total)
            button_byte = payload[8]
            move_switch = bool(button_byte & (1 << 0))
            look_switch = bool(button_byte & (1 << 1))
            emote_button = bool(button_byte & (1 << 2))
            payload_drop = bool(button_byte & (1 << 3))
            gait_button = bool(button_byte & (1 << 4))
            mode_button = bool(button_byte & (1 << 5))
            
            # Extract percentage values (1 byte each)
            move_x_percent = payload[9]
            move_y_percent = payload[10]
            look_x_percent = payload[11]
            look_y_percent = payload[12]
            
            # Extract timestamp (4 bytes)
            esp_timestamp = struct.unpack("<I", payload[13:17])[0]
            
            # Print decoded data (for debugging)
            print(f"Received at: {time.time():.3f}, ESP timestamp: {esp_timestamp/1000:.3f}")
            print(f"Move: ({move_x}, {move_y}) [{move_x_percent}%, {move_y_percent}%]")
            print(f"Look: ({look_x}, {look_y}) [{look_x_percent}%, {look_y_percent}%]")
            print(f"Buttons: Move:{move_switch} Look:{look_switch} Emote:{emote_button} "
                 f"Payload:{payload_drop} Gait:{gait_button} Mode:{mode_button}")
            print("-" * 50)
            
            # Process the data for your robot here
            # Example: send_to_robot_controller(move_x, move_y, look_x, look_y, ...)
            
        except Exception as e:
            logger.error(f"Error processing message: {e}", exc_info=True)


# Initialize controller
controller = Controller()

# Callback functions that use the controller through userdata parameter
def on_connect(client, userdata, flags, rc):
    """Callback when connected to MQTT broker"""
    if rc == 0:
        logger.info(f"Connected to MQTT broker")
        client.subscribe(TOPIC)
    else:
        logger.error(f"Failed to connect to MQTT broker with code {rc}")


def on_message(client, userdata, msg):
    """Callback when a message is received"""
    # The userdata parameter contains our controller
    # This was causing the error before when trying to access client.user_data
    userdata.process_command(msg.payload)


def on_disconnect(client, userdata, rc):
    """Callback when disconnected from MQTT broker"""
    if rc != 0:
        logger.warning(f"Unexpected disconnection. Reconnecting...")


def main():
    """Main function"""
    global controller
    
    # Set up MQTT client - PASS THE CONTROLLER AS USERDATA
    client = mqtt.Client(userdata=controller)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    
    # Connect to broker
    try:
        logger.info(f"Connecting to MQTT broker at localhost:1883")
        client.connect("localhost", 1883, 60)
        
        # Start the loop
        logger.info("Robot receiver is running... (Ctrl+C to exit)")
        client.loop_forever()
    except KeyboardInterrupt:
        logger.info("Receiver stopped by user")
        client.disconnect()
    except Exception as e:
        logger.error(f"Error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    main()
