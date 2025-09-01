'''
    This application will start all other needed processes for this project.
    - Hexapod
    - Zigbee Controller input
    - i2c to hexapod
    - camera input
    - etc.

'''
from src.pod import Pod
from src.hex_body import Body
from src.inversekinematics import solve_effector_IK
from src.gaits import new_Gait, Gait, GaitType
import numpy as np 
from src.coord import new_Coordinate
from src.config import COXA_ORIGIN_INDEX, FEMUR_ORIGIN_INDEX, TIBIA_ORIGIN_INDEX, EFFECTOR_ORIGIN_INDEX
import asyncio
from bleak import BleakClient, BleakScanner
import copy
import struct
import time

# BLE Configuration
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
# MAC
ESP32_MAC = "1C:69:20:8B:71:FA"

controller_data = {
    "move_x": 0,
    "move_y": 0,
    "look_x": 0,
    "look_y": 0,
    "move_switch": False,
    "look_switch": False,
    "emote_button": False,
    "payload_drop": False,
    "gait_button": False,
    "mode_button": False,
    "move_x_percent": 0,
    "move_y_percent": 0,
    "look_x_percent": 0,
    "look_y_percent": 0
}

received_count = 0
last_received_time = 0

# BLE notification handler - FIXED parameter name from 'payload' to 'data'
# BLE notification handler for comma-separated string format
def notification_handler(sender, data):
    global received_count, last_received_time, controller_data
    
    received_count += 1
    last_received_time = time.time()
    
    try:
        # Convert data to string if it's bytes
        # if isinstance(data, bytes):
        data = data.decode('utf-8')
        
        print(f"Raw data received: {data}")
        
        # Parse comma-separated values
        parts = data.split(',')
        
        if len(parts) < 10:
            print(f"Data has too few parts: {len(parts)}")
            return
            
        # Extract values from string parts
        move_x = int(parts[0])
        move_y = int(parts[1])
        look_x = int(parts[2]) 
        look_y = int(parts[3])
        move_switch = int(parts[4]) == 1
        look_switch = int(parts[5]) == 1
        emote_button = int(parts[6]) == 1
        payload_drop = int(parts[7]) == 1
        gait_button = int(parts[8]) == 1
        mode_button = int(parts[9]) == 1
        
        # Calculate percentage values (0-100)
        # Map from range -4095 to 4095 to 0-100
        def map_to_percent(value):
            abs_val = abs(value)
            if abs_val == 0:
                return 0
            percent = min(100, int((abs_val / 4095) * 100))
            return percent
            
        move_x_percent = map_to_percent(move_x)
        move_y_percent = map_to_percent(move_y)
        look_x_percent = map_to_percent(look_x)
        look_y_percent = map_to_percent(look_y)
        
        # Update global controller state
        controller_data.update({
            "move_x": move_x,
            "move_y": move_y,
            "look_x": look_x,
            "look_y": look_y,
            "move_switch": move_switch,
            "look_switch": look_switch,
            "emote_button": emote_button,
            "payload_drop": payload_drop,
            "gait_button": gait_button,
            "mode_button": mode_button,
            "move_x_percent": move_x_percent,
            "move_y_percent": move_y_percent,
            "look_x_percent": look_x_percent,
            "look_y_percent": look_y_percent
        })
        
        print(f"Controller: Move({move_x_percent}%, {move_y_percent}%) Switch:{move_switch}")
        
    except Exception as e:
        print(f"Error parsing data: {e}")
        print(f"Raw data: {data}")

# Connect and handle BLE
async def connect_and_listen(address):
    global received_count, last_received_time
    
    max_retries = 5
    retry_count = 0
    retry_delay = 5  # seconds
    
    while retry_count < max_retries:
        try:
            print(f"Connecting to BLE device {address}... (Attempt {retry_count+1}/{max_retries})")
            
            async with BleakClient(address) as client:
                print(f"Connected to {client.address}")
                
                # Reset stats
                received_count = 0
                last_received_time = time.time()
                
                # Start notifications
                await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
                print("Notifications activated")
                
                # Keep connection alive and monitor
                while True:
                    await asyncio.sleep(2.0)
                    time_since_last = time.time() - last_received_time
                    
                    # If no data received for 10 seconds, log a warning
                    if received_count > 0 and time_since_last > 10:
                        print(f"No data received for {time_since_last:.1f} seconds")
                    
                    # If we've received data, occasionally log status
                    if received_count > 0 and int(time_since_last) % 30 == 0:
                        print(f"BLE connection alive - Received {received_count} notifications")
                    
        except Exception as e:
            print(f"BLE connection error: {e}")
            retry_count += 1
            
            if retry_count < max_retries:
                print(f"Retrying in {retry_delay} seconds...")
                await asyncio.sleep(retry_delay)
            else:
                print("Max retries reached, giving up on BLE connection")
                break
        finally:
            print("BLE connection closed")
    
    print("BLE connection task ending")

# FIXED: Removed async from this function, as it needs to run in a thread/task
def hexapod_control(hexapod):
    
    
    while True:
        
        if hexapod.currentMode == "neutral":
            hexapod.start()
                
        if controller_data["move_y"] > 1500:
            hexapod.update()
          
        time.sleep(0.05)

# Main entry
async def main(hexapod):
    print("Starting main application")
    
    # First try to scan for device by name
    try:
        print("Scanning for BLE devices...")
        devices = await BleakScanner.discover(timeout=5.0)
        
        if devices:
            print(f"Found {len(devices)} BLE devices")
            for d in devices:
                print(f"  - {d.name or 'Unknown'}: {d.address}")
                
            target_device = next((d for d in devices if d.address.upper() == ESP32_MAC.upper() or 
                                 (d.name and "AB_BLE_ESP32" in d.name)), None)
                                 
            if target_device:
                print(f"Found target device: {target_device.name} ({target_device.address})")
                address = target_device.address
            else:
                print(f"Target device not found in scan, using hardcoded MAC: {ESP32_MAC}")
                address = ESP32_MAC
        else:
            print("No BLE devices found in scan, using hardcoded MAC")
            address = ESP32_MAC
            
    except Exception as e:
        print(f"Error during device scan: {e}")
        print(f"Falling back to hardcoded MAC: {ESP32_MAC}")
        address = ESP32_MAC
    
    # FIXED: Create a non-async function for hexapod control and run it in a separate thread
    import threading
    control_thread = threading.Thread(target=hexapod_control, args=(hexapod,), daemon=True)
    control_thread.start()
    
    # Run BLE connection in main async task
    await connect_and_listen(address)

if __name__ == "__main__":
    # setup controller stuff here
    tripod_gait = GaitType.TRIPOD
    wave_gait = GaitType.WAVE
    ripple_gait = GaitType.RIPPLE

    # Create the hexapod instance
    gait = new_Gait(tripod_gait, 1.0)  
    body = Body(6, Gait=gait)  
    body = body.load("src/hexapod_config.json")
    hexapod = Pod(body)
    # Ensure gaits are set the same
    body.set_gait(gait)
    hexapod.set_gait(gait)

    try:
        asyncio.run(main(hexapod))
    except KeyboardInterrupt:
        print("\nApplication terminated by user")
    except Exception as e:
        print(f"Unhandled exception: {e}")