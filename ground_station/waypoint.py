#!/usr/bin/env python3
"""
Send waypoints to the boat over the SiK telemetry radio.

Single waypoint:
    python waypoint.py 30.6914263 -88.1754964

Mission (multiple waypoints, boat returns to start when done):
    python waypoint.py mission 30.691 -88.175 30.692 -88.176 30.693 -88.177

Clear active waypoint/mission:
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
        time.sleep(0.1)
        s.write(cmd.encode('ascii'))
        s.flush()
        s.close()
        print(f'Sent: {cmd.strip()}')
    except serial.SerialException as e:
        print(f'ERROR: Could not open {PORT} — {e}')
        sys.exit(1)

def parse_all_coords(args):
    """Extract all lat/lon pairs from the argument list."""
    raw = ' '.join(args).replace('(', '').replace(')', '').replace(',', ' ')
    nums = re.findall(r'-?\d+\.?\d*', raw)
    if len(nums) < 2 or len(nums) % 2 != 0:
        print('ERROR: Need an even number of values (lat lon pairs).')
        sys.exit(1)
    pairs = []
    for i in range(0, len(nums), 2):
        lat, lon = float(nums[i]), float(nums[i+1])
        if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
            print(f'ERROR: Invalid coordinates: lat={lat}, lon={lon}')
            sys.exit(1)
        pairs.append((lat, lon))
    return pairs

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(0)

cmd = sys.argv[1].lower()

if cmd == 'clear':
    send('CLEAR\r')

elif cmd == 'mission':
    pairs = parse_all_coords(sys.argv[2:])
    if not pairs:
        print('ERROR: Provide at least one waypoint after "mission".')
        sys.exit(1)
    parts = ['MISSION']
    for lat, lon in pairs:
        parts.append(f'{lat:.6f}')
        parts.append(f'{lon:.6f}')
    mission_cmd = ':'.join(parts) + '\r'
    print(f'Sending mission: {len(pairs)} waypoint(s) + return to start')
    for i, (lat, lon) in enumerate(pairs, 1):
        print(f'  WP{i}: {lat:.6f}, {lon:.6f}')
    send(mission_cmd)

else:
    ##Single waypoint
    pairs = parse_all_coords(sys.argv[1:])
    lat, lon = pairs[0]
    print(f'Sending waypoint: {lat:.6f}, {lon:.6f} -> {PORT}')
    send(f'W:{lat:.6f}:{lon:.6f}\r')
