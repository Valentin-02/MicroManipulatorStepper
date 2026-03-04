"""
Simple HEX Sensor Debug Script
Just sends M60 and shows the raw response
"""

import time
import serial

PORT = 'COM8'
BAUD = 921600

print(f"Opening {PORT} at {BAUD} baud...")
ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(1)

print("Sending M60 (read HEX sensor)...\n")
ser.write(b"M60\n")
ser.flush()

print("Raw response:")
response = ""
timeout = time.time() + 5
while time.time() < timeout:
    if ser.in_waiting:
        char = ser.read(1).decode('ascii', errors='ignore')
        response += char
        print(repr(char), end='')
    else:
        time.sleep(0.01)

print(f"\n\nFull response:\n{response}")

ser.close()
