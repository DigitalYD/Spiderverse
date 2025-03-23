#!/usr/bin/env python3
"""
Simple RPLIDAR A1M8 Demo
------------------------
Connects to an RPLIDAR A1M8 and prints the raw scan data.
For Raspberry Pi 5 running Ubuntu Desktop.
"""

import time
import serial

# RPLIDAR commands
STOP_COMMAND = b'\xA5\x25'
RESET_COMMAND = b'\xA5\x40'
SCAN_COMMAND = b'\xA5\x20'

def connect_to_lidar(port='/dev/ttyUSB0', baudrate=115200):
    """Connect to the RPLIDAR device."""
    try:
        lidar = serial.Serial(
            port,
            baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        return lidar
    except serial.SerialException as e:
        print(f"Error connecting to RPLIDAR: {e}")
        return None

def stop_lidar(lidar):
    """Stop the RPLIDAR scanning."""
    lidar.write(STOP_COMMAND)
    lidar.flush()

def start_scan(lidar):
    """Start the RPLIDAR scanning process."""
    lidar.write(SCAN_COMMAND)
    lidar.flush()
    
    # Clear any existing data
    time.sleep(0.1)
    lidar.reset_input_buffer()

def read_scan_point(lidar):
    """Read and parse a single scan point from the RPLIDAR."""
    try:
        # Each scan point is 5 bytes
        raw = lidar.read(5)
        
        if len(raw) != 5:
            return None
            
        # Parse the scan data according to RPLIDAR A1M8 protocol
        quality = raw[0] >> 2  # Quality is in the top 6 bits
        angle_q6 = ((raw[0] & 0x03) << 8) | raw[1]  # Bottom 2 bits + byte 1
        angle = angle_q6 / 64.0  # Convert from Q6 format to degrees
        distance = ((raw[3] << 8) | raw[2]) / 4.0  # Distance in mm
        
        return {
            'quality': quality,
            'angle': angle,
            'distance': distance
        }
    except:
        print("shit is not working")
        
        return None

def main():
    """Main function to run the RPLIDAR demo."""
    print("RPLIDAR A1M8 Simple Demo")
    print("------------------------")
    print("Connecting to RPLIDAR...")
    
    # Try to connect to the RPLIDAR
    lidar = connect_to_lidar()
    if not lidar:
        print("Failed to connect to RPLIDAR. Please check connections and try again.")
        return
    
    print("Connected to RPLIDAR!")
    
    try:
        # Stop any existing scan
        stop_lidar(lidar)
        time.sleep(0.5)
        
        # Start a new scan
        print("Starting scan...")
        start_scan(lidar)
        time.sleep(1)  # Give it time to start scanning
        
        print("\nReading scan data (press Ctrl+C to stop):")
        print("----------------------------------------")
        print("  Quality | Angle (deg) | Distance (mm)")
        print("----------------------------------------")
        
        # Continuously read and print scan points
        while True:
            point = read_scan_point(lidar)
            if point:
                print(f"  {point['quality']:7d} | {point['angle']:11.2f} | {point['distance']:12.2f}")
                time.sleep(0.01)  # Small delay to make output readable
    
    except KeyboardInterrupt:
        print("\nScan stopped by user")
    
    finally:
        # Always stop the lidar and close the connection
        print("Stopping RPLIDAR...")
        stop_lidar(lidar)
        lidar.close()
        print("RPLIDAR disconnected")

if __name__ == "__main__":
    main()