#!/usr/bin/env python3
"""
Send a single waypoint to the boat over the SiK telemetry radio.

Usage:
    python waypoint.py 30.6914263 -88.1754964
    python waypoint.py "30.6914263, -88.1754964"
    python waypoint.py "(30.6914263, -88.1754964)"

To clear the active waypoint:
    python waypoint.py clear
"""

import sys
import platform
import serial
import re
import time

##Serial port — matches ground_station.py
if platform.system() == 'Windows':
    PORT = 'COM3'
else:
    PORT = '/dev/ttyUSB0'
BAUD = 57600

def send(cmd):
    try:
        s = serial.Serial(PORT, BAUD, timeout=2)
        time.sleep(0.1)  # let port settle
        s.write(cmd.encode('ascii'))
        s.flush()
        s.close()
        print(f'Sent: {cmd.strip()}')
    except serial.SerialException as e:
        print(f'ERROR: Could not open {PORT} — {e}')
        sys.exit(1)

def parse_coords(args):
    ##Join all args so formats like "(30.6, -88.1)" or "30.6, -88.1" work
    raw = ' '.join(args).replace('(', '').replace(')', '').replace(',', ' ')
    nums = re.findall(r'-?\d+\.?\d*', raw)
    if len(nums) < 2:
        print('ERROR: Could not parse coordinates.')
        print('Usage: python waypoint.py 30.6914263 -88.1754964')
        sys.exit(1)
    return float(nums[0]), float(nums[1])

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(0)

if sys.argv[1].lower() == 'clear':
    send('CLEAR\r')
else:
    lat, lon = parse_coords(sys.argv[1:])
    if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
        print(f'ERROR: Invalid coordinates: lat={lat}, lon={lon}')
        sys.exit(1)
    print(f'Sending waypoint: {lat:.6f}, {lon:.6f} -> {PORT}')
    send(f'W:{lat:.6f}:{lon:.6f}\r')
