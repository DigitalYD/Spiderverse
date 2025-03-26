import socket
import struct

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 8089))  # Bind to all interfaces on port 8089
print("Listening for UDP packets on port 8089...")

try:
    while True:
        # Wait for data
        data, addr = sock.recvfrom(65536)
        print(f"Received {len(data)} bytes from {addr}")
        
        if len(data) >= 12:  # Minimum size for header
            # Parse header (timestamp + count)
            timestamp = struct.unpack('<Q', data[0:8])[0]
            count = struct.unpack('<I', data[8:12])[0]
            print(f"Timestamp: {timestamp}, Point count: {count}")
        else:
            print("Packet too small")
            
except KeyboardInterrupt:
    print("Shutting down")
finally:
    sock.close()