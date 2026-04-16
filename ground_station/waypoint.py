#!/usr/bin/env python3
"""
Send waypoints to the boat over the SiK telemetry radio.

Single waypoint:
    python waypoint.py 30.6914263 -88.1754964

Mission from a text file (one  lat, lon  per line):
    python waypoint.py mission waypoints.txt

Mission from inline coordinates:
    python waypoint.py mission 30.691 -88.175 30.692 -88.176

Clear active waypoint/mission:
    python waypoint.py clear

Text file format (blank lines and # comments are ignored):
    # my test route
    30.6914263, -88.1754964
    30.6914230, -88.1754970
    30.6914350, -88.1754970
"""

import sys
import os
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

def parse_line(line):
    """Parse a single 'lat, lon' line. Returns (lat, lon) or None."""
    line = line.split('#')[0].strip()   # strip inline comments
    if not line:
        return None
    nums = re.findall(r'-?\d+\.?\d*', line)
    if len(nums) < 2:
        return None
    return float(nums[0]), float(nums[1])

def load_file(path):
    """Load waypoints from a text file."""
    if not os.path.exists(path):
        print(f'ERROR: File not found: {path}')
        sys.exit(1)
    pairs = []
    with open(path) as f:
        for lineno, line in enumerate(f, 1):
            result = parse_line(line)
            if result is None:
                continue
            lat, lon = result
            if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                print(f'ERROR: Invalid coordinates on line {lineno}: {lat}, {lon}')
                sys.exit(1)
            pairs.append((lat, lon))
    if not pairs:
        print(f'ERROR: No valid waypoints found in {path}')
        sys.exit(1)
    return pairs

def parse_inline_coords(args):
    """Extract all lat/lon pairs from inline arguments."""
    raw = ' '.join(args).replace('(','').replace(')','').replace(',',' ')
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

def send_mission(pairs):
    parts = ['MISSION']
    for lat, lon in pairs:
        parts.append(f'{lat:.6f}')
        parts.append(f'{lon:.6f}')
    print(f'Sending mission: {len(pairs)} waypoint(s) + return to start')
    for i, (lat, lon) in enumerate(pairs, 1):
        print(f'  WP{i}: {lat:.6f}, {lon:.6f}')
    print(f'  --> Return to start')
    send(':'.join(parts) + '\r')

# ---------------------------------------------------------------------------

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(0)

cmd = sys.argv[1].lower()

if cmd == 'clear':
    send('CLEAR\r')

elif cmd == 'mission':
    if len(sys.argv) < 3:
        print('ERROR: Provide a file path or coordinates after "mission".')
        sys.exit(1)
    ##If next arg looks like a file, load it; otherwise parse inline coords
    if os.path.exists(sys.argv[2]) or sys.argv[2].endswith('.txt'):
        pairs = load_file(sys.argv[2])
    else:
        pairs = parse_inline_coords(sys.argv[2:])
    send_mission(pairs)

else:
    ##Single waypoint
    pairs = parse_inline_coords(sys.argv[1:])
    lat, lon = pairs[0]
    print(f'Sending waypoint: {lat:.6f}, {lon:.6f} -> {PORT}')
    send(f'W:{lat:.6f}:{lon:.6f}\r')
