import paho.mqtt.client as mqtt
import time
import json

# MQTT settings
MQTT_BROKER = "localhost"  # Since broker runs on the robot's Pi
MQTT_PORT = 1883
COMMAND_TOPIC = "robot/commands"

# Callback when connecting to MQTT broker
def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")
    print(f"Listening for commands on topic: {COMMAND_TOPIC}")
    client.subscribe(COMMAND_TOPIC)

# Callback when receiving command message
def on_message(client, userdata, msg):
    try:
        # Decode the message payload
        command = json.loads(msg.payload.decode())
        
        # Print out the raw command data
        print("-" * 50)
        print(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Received command from ESP32: {json.dumps(command, indent=2)}")
        print("-" * 50)
            
    except Exception as e:
        print(f"Error processing command: {e}")

# Create MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Connect to broker and start loop
try:
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    print("Robot listener started. Waiting for commands from ESP32...")
    client.loop_forever()
except KeyboardInterrupt:
    print("Stopping robot listener...")
    client.disconnect()
except Exception as e:
    print(f"Error: {e}")
    client.disconnect()
