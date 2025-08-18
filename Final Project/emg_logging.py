"""
This script logs data from a serial port to a CSV file.
It reads lines from the serial port, decodes them, and writes them to a file.

Author: Eitan Gerber, Elad Siman Tov
Date: 2025-08-12

Usage:
1. Ensure the serial port is correctly set and running (e.g., 'COM8').
2. Run the script.
"""
import serial
from datetime import datetime

ser = serial.Serial('COM8', 115200, timeout=1)  # Add timeout for robustness
filename = f"gait_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

with open(filename, 'w') as f:
    try:
        while True:
            try:
                line = ser.readline()
                # Skip empty lines
                if not line:
                    continue
                # Try to decode, skip if decode fails
                try:
                    decoded = line.decode('utf-8').strip()
                except UnicodeDecodeError:
                    continue
                if decoded:
                    f.write(decoded + "\n")
                    print(decoded)
            except KeyboardInterrupt:
                print("Logging stopped by user.")
                break
    finally:
        ser.close()
        print("Serial port closed.")