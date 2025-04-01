import asyncio
from bleak import BleakClient, BleakScanner

#UUIDs
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#MAC
ESP32_MAC = "1C:69:20:8B:71:FA"

'''
moveHexaXValue
moveHexaYValue
lookHexaXValue
lookHexaYValue
moveSwitchState
lookSwitchState
b1 = false
b2 = false
b3 = false
b4 = false
'''

#Your robot loop logic goes here
async def robot_loop():
    while True:
        # Run background tasks or checks
        print("🔄 Running robot loop")
        # read_sensors()
        # decide_behavior()
        await asyncio.sleep(0.1)


#BLE notification handler
def notification_handler(sender, data):
    print("📥 Received:", data)
    
    decoded_data = data.decode('utf-8')
    return decoded_data
    # process BLE command
    # robot.move(), etc.

#Connect and handle BLE
async def connect_and_listen(address):
    async with BleakClient(address) as client:
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        print("🔗 Connected and listening")
        await robot_loop()  # Your robot logic runs here
        await client.stop_notify(CHARACTERISTIC_UUID)

#Main entry
async def main():
    device = await BleakScanner.find_device_byfilter(lambda d, : "ESP32" in d.name)
    if device:
        await connect_and_listen(device.address)
    else:
        print("❌ No ESP32 found")

#Start everything
if __name__ == "__main__":
    asyncio.run(main())
