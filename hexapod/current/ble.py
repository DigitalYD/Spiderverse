import asyncio
import struct
from bleak import BleakClient, BleakScanner
import time

# BLE Configuration
ESP32_MAC = "1C:69:20:8B:71:FA"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# Global controller data
controller_data = {
    "move_x": 0,
    "move_y": 0,
    "look_x": 0,
    "look_y": 0,
    "move_switch": False,
    "look_switch": False,
    "move_x_percent": 0,
    "move_y_percent": 0
}

# Notification counter
notification_count = 0

# BLE notification handler with print statements
def notification_handler(sender, data):
    global notification_count, controller_data
    notification_count += 1
    
    print(f"NOTIFICATION #{notification_count} RECEIVED")
    
    try:
        # Make sure data is bytes
        if not isinstance(data, bytes):
            print(f"Data is not bytes, it's {type(data)}")
            if isinstance(data, str):
                data = data.encode('utf-8')
            else:
                data = bytes(data)
        
        # Check data length
        if len(data) < 13:
            print(f"Data too short: {len(data)} bytes")
            return
            
        # Extract joystick values
        move_x = struct.unpack("<h", data[0:2])[0]
        move_y = struct.unpack("<h", data[2:4])[0]
        look_x = struct.unpack("<h", data[4:6])[0]
        look_y = struct.unpack("<h", data[6:8])[0]
        
        # Extract button states
        button_byte = data[8]
        move_switch = bool(button_byte & (1 << 0))
        look_switch = bool(button_byte & (1 << 1))
        
        # Extract percentages
        move_x_percent = data[9]
        move_y_percent = data[10]
        
        # Update global data
        controller_data.update({
            "move_x": move_x,
            "move_y": move_y,
            "look_x": look_x,
            "look_y": look_y,
            "move_switch": move_switch,
            "look_switch": look_switch,
            "move_x_percent": move_x_percent,
            "move_y_percent": move_y_percent
        })
        
        print(f"Updated controller data: Move({move_x_percent}%, {move_y_percent}%) Switch:{move_switch}")
        
    except Exception as e:
        print(f"Error parsing data: {e}")
        print(f"Raw data: {data}")

# Simulated hexapod control loop
async def hexapod_control():
    print("Starting hexapod control loop")
    print("This will check the controller data every second")
    
    while True:
        # Print the current controller data
        print("\n--- HEXAPOD CONTROL CHECK ---")
        print(f"Move: X={controller_data['move_x']}, Y={controller_data['move_y']}")
        print(f"Percentages: X={controller_data['move_x_percent']}%, Y={controller_data['move_y_percent']}%")
        print(f"Switches: Move={controller_data['move_switch']}, Look={controller_data['look_switch']}")
        
        # Check if we should move
        if controller_data["move_switch"] and abs(controller_data["move_y"]) > 1000:
            direction = "FORWARD" if controller_data["move_y"] < 0 else "BACKWARD"
            print(f"HEXAPOD WOULD MOVE {direction}")
        else:
            print("HEXAPOD WOULD BE IDLE")
            
        await asyncio.sleep(1)  # Check every second

async def main():
    print("=== SIMPLE BLE TEST WITH PRINT STATEMENTS ===")
    print(f"Target device: {ESP32_MAC}")
    print(f"Target characteristic: {CHARACTERISTIC_UUID}")
    
    # Start hexapod control in background
    control_task = asyncio.create_task(hexapod_control())
    
    # Connect to BLE device
    try:
        print(f"Connecting to {ESP32_MAC}...")
        async with BleakClient(ESP32_MAC) as client:
            print(f"Connected to {client.address}")
            
            # Start notifications
            print("Starting notifications...")
            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
            print("Notifications started. Move controller joysticks to see updates.")
            
            # Keep connection alive
            try:
                while True:
                    await asyncio.sleep(1)
            except asyncio.CancelledError:
                print("Connection task cancelled")
                
    except Exception as e:
        print(f"BLE Error: {e}")
    finally:
        print("BLE connection closed")
        
    # Wait for control task to complete
    control_task.cancel()
    try:
        await control_task
    except asyncio.CancelledError:
        pass

if __name__ == "__main__":
    asyncio.run(main())