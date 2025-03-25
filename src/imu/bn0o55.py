#!/usr/bin/env python3
"""
BNO055 IMU Demo for Raspberry Pi 5 with Ubuntu Desktop
-----------------------------------------------------
A simple demo to read and display data from the BNO055 IMU sensor
connected to a Raspberry Pi 5 running Ubuntu Desktop.
"""

import time
import smbus2
import numpy as np
import os
from datetime import datetime

# BNO055 Register Map
BNO055_ADDRESS = 0x28           # Default I2C address
BNO055_CHIP_ID = 0xA0           # Expected chip ID
# Register addresses
REG_CHIP_ID = 0x00
REG_OPR_MODE = 0x3D
REG_SYS_TRIGGER = 0x3F
REG_PWR_MODE = 0x3E
REG_PAGE_ID = 0x07
REG_CALIB_STAT = 0x35
REG_TEMP = 0x34

# Sensor data registers (when in page 0)
REG_ACCEL_DATA = 0x08           # Acceleration data registers (6 bytes)
REG_MAG_DATA = 0x0E             # Magnetometer data registers (6 bytes)
REG_GYRO_DATA = 0x14            # Gyroscope data registers (6 bytes)
REG_EULER_DATA = 0x1A           # Euler angles data registers (6 bytes)
REG_QUATERNION_DATA = 0x20      # Quaternion data registers (8 bytes)
REG_LINEAR_ACCEL_DATA = 0x28    # Linear acceleration data registers (6 bytes)
REG_GRAVITY_DATA = 0x2E         # Gravity vector data registers (6 bytes)

# Operation modes
MODE_CONFIG = 0x00
MODE_NDOF = 0x0C                # Nine Degrees of Freedom mode with absolute orientation

class BNO055:
    """Class for interfacing with the BNO055 IMU sensor."""
    
    def __init__(self, address=BNO055_ADDRESS, bus_num=1):
        """Initialize the BNO055 sensor."""
        self.address = address
        self.bus = smbus2.SMBus(bus_num)
        
    def begin(self):
        """Initialize the sensor."""
        # Check if sensor is connected
        chip_id = self.read_byte(REG_CHIP_ID)
        if chip_id != BNO055_CHIP_ID:
            print(f"Error: Unexpected chip ID: 0x{chip_id:02X}, expected: 0x{BNO055_CHIP_ID:02X}")
            return False
            
        # Reset the device
        self.write_byte(REG_SYS_TRIGGER, 0x20)
        time.sleep(0.5)  # Wait for reset to complete
        
        # Wait for chip ID to be correctly read after reset
        timeout = time.time() + 5  # 5 second timeout
        while time.time() < timeout:
            chip_id = self.read_byte(REG_CHIP_ID)
            if chip_id == BNO055_CHIP_ID:
                break
            time.sleep(0.01)
        
        if chip_id != BNO055_CHIP_ID:
            print(f"Error: Unable to read correct chip ID after reset")
            return False
            
        # Set to CONFIG mode
        self.set_mode(MODE_CONFIG)
        
        # Set power mode to NORMAL
        self.write_byte(REG_PWR_MODE, 0x00)
        time.sleep(0.01)
        
        # Set PAGE_ID to 0
        self.write_byte(REG_PAGE_ID, 0x00)
        time.sleep(0.01)
        
        # Set mode to NDOF (fusion mode)
        self.set_mode(MODE_NDOF)
        time.sleep(0.02)
        
        return True
        
    def set_mode(self, mode):
        """Set the operation mode of the sensor."""
        self.write_byte(REG_OPR_MODE, mode)
        time.sleep(0.03)  # Wait for mode switch to complete
        
    def write_byte(self, register, value):
        """Write a byte to the specified register."""
        try:
            self.bus.write_byte_data(self.address, register, value)
        except IOError as e:
            print(f"I/O error writing to register 0x{register:02X}: {e}")
            
    def read_byte(self, register):
        """Read a byte from the specified register."""
        try:
            return self.bus.read_byte_data(self.address, register)
        except IOError as e:
            print(f"I/O error reading from register 0x{register:02X}: {e}")
            return 0
            
    def read_bytes(self, register, length):
        """Read multiple bytes starting from the specified register."""
        try:
            return self.bus.read_i2c_block_data(self.address, register, length)
        except IOError as e:
            print(f"I/O error reading from registers 0x{register:02X}-0x{register+length-1:02X}: {e}")
            return [0] * length
            
    def get_temperature(self):
        """Read temperature in degrees Celsius."""
        return self.read_byte(REG_TEMP)
        
    def get_calibration_status(self):
        """Get calibration status for system, gyro, accel, and mag.
        
        Returns:
            Tuple of (system, gyro, accel, mag) calibration status, each 0-3
        """
        calib_stat = self.read_byte(REG_CALIB_STAT)
        sys = (calib_stat >> 6) & 0x03
        gyro = (calib_stat >> 4) & 0x03
        accel = (calib_stat >> 2) & 0x03
        mag = calib_stat & 0x03
        return (sys, gyro, accel, mag)
        
    def get_acceleration(self):
        """Get accelerometer data in m/s^2."""
        data = self.read_bytes(REG_ACCEL_DATA, 6)
        x = self._convert_signed_int16(data[0], data[1]) / 100.0  # Scale to m/s^2
        y = self._convert_signed_int16(data[2], data[3]) / 100.0
        z = self._convert_signed_int16(data[4], data[5]) / 100.0
        return (x, y, z)
        
    def get_magnetometer(self):
        """Get magnetometer data in micro-Teslas."""
        data = self.read_bytes(REG_MAG_DATA, 6)
        x = self._convert_signed_int16(data[0], data[1]) / 16.0  # Scale to micro-Teslas
        y = self._convert_signed_int16(data[2], data[3]) / 16.0
        z = self._convert_signed_int16(data[4], data[5]) / 16.0
        return (x, y, z)
        
    def get_gyroscope(self):
        """Get gyroscope data in degrees per second."""
        data = self.read_bytes(REG_GYRO_DATA, 6)
        x = self._convert_signed_int16(data[0], data[1]) / 16.0  # Scale to degrees/s
        y = self._convert_signed_int16(data[2], data[3]) / 16.0
        z = self._convert_signed_int16(data[4], data[5]) / 16.0
        return (x, y, z)
        
    def get_euler(self):
        """Get Euler angles in degrees."""
        data = self.read_bytes(REG_EULER_DATA, 6)
        heading = self._convert_signed_int16(data[0], data[1]) / 16.0  # Heading (yaw)
        roll = self._convert_signed_int16(data[2], data[3]) / 16.0     # Roll
        pitch = self._convert_signed_int16(data[4], data[5]) / 16.0    # Pitch
        return (roll, pitch, heading)
        
    def get_quaternion(self):
        """Get quaternion data."""
        data = self.read_bytes(REG_QUATERNION_DATA, 8)
        w = self._convert_signed_int16(data[0], data[1]) / 16384.0  # Scale to unit quaternion
        x = self._convert_signed_int16(data[2], data[3]) / 16384.0
        y = self._convert_signed_int16(data[4], data[5]) / 16384.0
        z = self._convert_signed_int16(data[6], data[7]) / 16384.0
        return (w, x, y, z)
        
    def get_linear_acceleration(self):
        """Get linear acceleration in m/s^2 (without gravity)."""
        data = self.read_bytes(REG_LINEAR_ACCEL_DATA, 6)
        x = self._convert_signed_int16(data[0], data[1]) / 100.0  # Scale to m/s^2
        y = self._convert_signed_int16(data[2], data[3]) / 100.0
        z = self._convert_signed_int16(data[4], data[5]) / 100.0
        return (x, y, z)
        
    def get_gravity(self):
        """Get gravity vector in m/s^2."""
        data = self.read_bytes(REG_GRAVITY_DATA, 6)
        x = self._convert_signed_int16(data[0], data[1]) / 100.0  # Scale to m/s^2
        y = self._convert_signed_int16(data[2], data[3]) / 100.0
        z = self._convert_signed_int16(data[4], data[5]) / 100.0
        return (x, y, z)
    
    def _convert_signed_int16(self, lsb, msb):
        """Convert two bytes to signed 16-bit value."""
        value = (msb << 8) | lsb
        if value & 0x8000:  # Check if sign bit is set
            return value - 0x10000
        return value

def format_vector(vector):
    """Format a vector for display."""
    return f"X: {vector[0]:7.2f}  Y: {vector[1]:7.2f}  Z: {vector[2]:7.2f}"

def format_quaternion(quat):
    """Format a quaternion for display."""
    return f"W: {quat[0]:7.2f}  X: {quat[1]:7.2f}  Y: {quat[2]:7.2f}  Z: {quat[3]:7.2f}"

def format_euler(euler):
    """Format euler angles for display."""
    return f"Roll: {euler[0]:7.2f}°  Pitch: {euler[1]:7.2f}°  Yaw: {euler[2]:7.2f}°"

def print_calibration_status(sensor):
    """Print the calibration status of the IMU."""
    sys, gyro, accel, mag = sensor.get_calibration_status()
    print("Calibration Status:")
    print(f"  System: {sys}/3")
    print(f"  Gyroscope: {gyro}/3")
    print(f"  Accelerometer: {accel}/3")
    print(f"  Magnetometer: {mag}/3")

def display_imu_data(sensor):
    """Display all available IMU data."""
    os.system('clear')  # Clear the terminal
    
    # Get data from the sensor
    accel = sensor.get_acceleration()
    gyro = sensor.get_gyroscope()
    mag = sensor.get_magnetometer()
    euler = sensor.get_euler()
    quaternion = sensor.get_quaternion()
    linear_accel = sensor.get_linear_acceleration()
    gravity = sensor.get_gravity()
    temp = sensor.get_temperature()
    
    # Print header
    print("=" * 60)
    print(f"BNO055 IMU Data - {datetime.now().strftime('%H:%M:%S')}")
    print("=" * 60)
    
    # Print calibration status
    print_calibration_status(sensor)
    print("-" * 60)
    
    # Print sensor readings
    print(f"Temperature: {temp} °C")
    print("-" * 60)
    
    print("Orientation:")
    print(f"  Euler Angles: {format_euler(euler)}")
    print(f"  Quaternion: {format_quaternion(quaternion)}")
    print("-" * 60)
    
    print("Sensors:")
    print(f"  Accelerometer: {format_vector(accel)} m/s²")
    print(f"  Gyroscope: {format_vector(gyro)} deg/s")
    print(f"  Magnetometer: {format_vector(mag)} µT")
    print("-" * 60)
    
    print("Processed Data:")
    print(f"  Linear Acceleration: {format_vector(linear_accel)} m/s²")
    print(f"  Gravity Vector: {format_vector(gravity)} m/s²")
    print("=" * 60)
    
    print("Press Ctrl+C to exit")

def main():
    """Main function to run the BNO055 demo."""
    print("BNO055 IMU Demo for Raspberry Pi 5 with Ubuntu Desktop")
    print("----------------------------------------------------")
    
    # Create and initialize the IMU
    imu = BNO055()
    print("Connecting to BNO055 IMU...")
    
    if not imu.begin():
        print("Failed to initialize BNO055 IMU!")
        print("Check connections and try again.")
        return
    
    print("BNO055 IMU initialized successfully!")
    
    # Display calibration instructions
    print("\nCalibration Instructions:")
    print("1. Accelerometer: Move the sensor in different positions (6 directions)")
    print("2. Gyroscope: Place the sensor still on a flat surface")
    print("3. Magnetometer: Move in a figure-8 pattern in the air")
    print("\nPress Enter to start reading data...")
    input()
    
    try:
        # Main loop - continuously display IMU data
        while True:
            display_imu_data(imu)
            time.sleep(0.1)  # Update rate: 10 Hz
    
    except KeyboardInterrupt:
        print("\nDemo stopped by user")

if __name__ == "__main__":
    main()