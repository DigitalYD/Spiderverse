#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import struct
import time
import lgpio
import math

# I2C addresses from your output
LSM9DS1_ADDR_MAG = 0x3d        # Magnetometer address from your output
LSM9DS1_ADDR_ACCEL_GYRO = 0x68 # Accelerometer/Gyroscope from your output

# Initialize the lgpio library
h = lgpio.gpiochip_open(0)
# Open I2C device - using bus 1 for pins 3 and 5
i2c_bus = 1
i2c_flags = 0
i2c_mag_handle = lgpio.i2c_open(i2c_bus, LSM9DS1_ADDR_MAG, i2c_flags)
i2c_accel_gyro_handle = lgpio.i2c_open(i2c_bus, LSM9DS1_ADDR_ACCEL_GYRO, i2c_flags)

# LSM9DS1 register addresses
# Gyro and Accel registers (they're in the same chip at 0x68)
CTRL_REG1_G = 0x10    # Gyro control
CTRL_REG6_XL = 0x20   # Accel control
OUT_X_L_G = 0x18      # Gyro data
OUT_X_L_XL = 0x28     # Accel data

# Magnetometer registers (separate chip at 0x3d)
CTRL_REG1_M = 0x20    # Magnetometer control
CTRL_REG2_M = 0x21    # Magnetometer control 2
OUT_X_L_M = 0x28      # Magnetometer data

# Initialize the LSM9DS1 sensor
def init_lsm9ds1():
    # Initialize gyroscope (952 Hz ODR, 2000 dps full-scale)
    lgpio.i2c_write_byte_data(i2c_accel_gyro_handle, CTRL_REG1_G, 0xC0)
    
    # Initialize accelerometer (952 Hz ODR, +/-16g full-scale)
    lgpio.i2c_write_byte_data(i2c_accel_gyro_handle, CTRL_REG6_XL, 0xC0)
    
    # Initialize magnetometer (Medium performance mode)
    lgpio.i2c_write_byte_data(i2c_mag_handle, CTRL_REG1_M, 0x60)  # Temperature compensation, medium performance
    lgpio.i2c_write_byte_data(i2c_mag_handle, CTRL_REG2_M, 0x00)  # 4 Gauss scale
    
    time.sleep(0.1)  # Wait for initialization

# Read raw sensor data
def read_lsm9ds1():
    # Read accelerometer data (6 bytes starting from OUT_X_L_XL)
    accel_data = lgpio.i2c_read_i2c_block_data(i2c_accel_gyro_handle, OUT_X_L_XL, 6)
    accel_x = (accel_data[1] << 8) | accel_data[0]
    accel_y = (accel_data[3] << 8) | accel_data[2]
    accel_z = (accel_data[5] << 8) | accel_data[4]
    
    # Convert to signed values
    accel_x = accel_x - 65536 if accel_x > 32767 else accel_x
    accel_y = accel_y - 65536 if accel_y > 32767 else accel_y
    accel_z = accel_z - 65536 if accel_z > 32767 else accel_z
    
    # Convert to m/s² (±16g range)
    accel_x = accel_x * 0.000732 * 9.81
    accel_y = accel_y * 0.000732 * 9.81
    accel_z = accel_z * 0.000732 * 9.81
    
    # Read gyroscope data (6 bytes starting from OUT_X_L_G)
    gyro_data = lgpio.i2c_read_i2c_block_data(i2c_accel_gyro_handle, OUT_X_L_G, 6)
    gyro_x = (gyro_data[1] << 8) | gyro_data[0]
    gyro_y = (gyro_data[3] << 8) | gyro_data[2]
    gyro_z = (gyro_data[5] << 8) | gyro_data[4]
    
    # Convert to signed values
    gyro_x = gyro_x - 65536 if gyro_x > 32767 else gyro_x
    gyro_y = gyro_y - 65536 if gyro_y > 32767 else gyro_y
    gyro_z = gyro_z - 65536 if gyro_z > 32767 else gyro_z
    
    # Convert to rad/s (±2000 dps range)
    gyro_x = gyro_x * 0.000061 * math.pi / 180.0
    gyro_y = gyro_y * 0.000061 * math.pi / 180.0
    gyro_z = gyro_z * 0.000061 * math.pi / 180.0
    
    # Read magnetometer data (6 bytes starting from OUT_X_L_M)
    mag_data = lgpio.i2c_read_i2c_block_data(i2c_mag_handle, OUT_X_L_M, 6)
    mag_x = (mag_data[1] << 8) | mag_data[0]
    mag_y = (mag_data[3] << 8) | mag_data[2]
    mag_z = (mag_data[5] << 8) | mag_data[4]
    
    # Convert to signed values
    mag_x = mag_x - 65536 if mag_x > 32767 else mag_x
    mag_y = mag_y - 65536 if mag_y > 32767 else mag_y
    mag_z = mag_z - 65536 if mag_z > 32767 else mag_z
    
    # Convert to gauss (±4 gauss range)
    mag_x = mag_x * 0.00014
    mag_y = mag_y * 0.00014
    mag_z = mag_z * 0.00014
    
    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z

# Initialize the sensor
try:
    init_lsm9ds1()
    print("LSM9DS1 initialized successfully")
except Exception as e:
    print(f"Error initializing LSM9DS1: {e}")
    raise

# Set up MQTT client for publishing
client = mqtt.Client()

# Configure MQTT broker address - change to your laptop's IP address
broker_address = "192.168.1.100"  # Replace with your laptop's IP
broker_port = 1883

# Connect to the broker
print(f"Connecting to MQTT broker at {broker_address}...")
client.connect(broker_address, broker_port, 60)
client.loop_start()  # Start the MQTT processing loop in a separate thread

# MQTT topic for IMU data
imu_topic = "robot/sensors/imu"

try:
    print("IMU data publisher is running...")
    while True:
        # Read the IMU data
        try:
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = read_lsm9ds1()
        except Exception as e:
            print(f"Error reading sensor: {e}")
            time.sleep(1)  # Wait before retrying
            continue
        
        # Get current timestamp
        timestamp = int(time.time() * 1000)  # milliseconds since epoch
        
        # Pack binary data - 10 floats (4 bytes each) + 4 byte timestamp = 44 bytes
        # Format: 3 accel values, 3 mag values, 3 gyro values, temperature, timestamp
        binary_data = struct.pack("<fffffffffi",
                                  accel_x, accel_y, accel_z,   # 3 acceleration values (m/s²)
                                  mag_x, mag_y, mag_z,         # 3 magnetic field values (gauss)
                                  gyro_x, gyro_y, gyro_z,      # 3 gyroscope values (rad/s)
                                  timestamp)                    # Timestamp in ms
        
        # Publish the binary data
        client.publish(imu_topic, binary_data)
        
        # Print values (for debugging)
        print(f"Published IMU data at {time.time():.3f}:")
        print(f"Accel: ({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}) m/s²")
        print(f"Mag: ({mag_x:.2f}, {mag_y:.2f}, {mag_z:.2f}) gauss")
        print(f"Gyro: ({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}) rad/s")
        print("-" * 50)
        
        # Wait a bit before the next reading (50Hz)
        time.sleep(0.02)
        
except KeyboardInterrupt:
    print("Publisher stopped by user")
    client.loop_stop()
    client.disconnect()
except Exception as e:
    print(f"Error: {e}")
    client.loop_stop()
    client.disconnect()
finally:
    # Clean up I2C and GPIO resources
    lgpio.i2c_close(i2c_mag_handle)
    lgpio.i2c_close(i2c_accel_gyro_handle)
    lgpio.gpiochip_close(h)
    print("Resources cleaned up")