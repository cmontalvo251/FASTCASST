#!/usr/bin/env python3
"""
test_gps_fix.py
----------------
Reads GPS from the Navio2 built-in NEO-M8N via ttyAMA0 (NMEA sentences).
Prints fix status and satellite count continuously.

Requirements:
    sudo pip3 install pyserial

Usage:
    sudo python3 src/test_gps_fix.py

Take the antenna outdoors with clear sky view.
You need fix_quality >= 1 and sats >= 6 to navigate.
Press Ctrl+C to stop.
"""

import time
import serial

GPS_PORT = '/dev/ttyAMA0'
GPS_BAUD = 9600

def parse_latlon(value_str, direction, is_lat):
    val = float(value_str)
    deg = int(val / 100)
    minutes = val - deg * 100
    decimal = deg + minutes / 60.0
    if direction in ('S', 'W'):
        decimal = -decimal
    return decimal

print(f'Opening Navio2 GPS on {GPS_PORT} at {GPS_BAUD} baud...')
try:
    ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=2)
    print('Port opened.\n')
except Exception as e:
    print(f'ERROR: Could not open {GPS_PORT}: {e}')
    print('Make sure ttyAMA0 is enabled and the SiK radio has been moved to Pixhawk TELEM1.')
    raise SystemExit(1)

FIX_NAMES = {0: 'NO FIX', 1: 'GPS', 2: 'DGPS', 3: 'PPS', 4: 'RTK Fixed',
             5: 'RTK Float', 6: 'Dead Reckoning'}

print('Waiting for GPS data — make sure antenna has clear sky view...')
print('You need fix_quality >= 1 and sats >= 6 to navigate.\n')
print(f'{"Time":>8}  {"Fix":<12} {"Sats":>5}  {"Lat":>12} {"Lon":>13}  {"Alt(m)":>8}  {"HDOP":>6}')
print('-' * 80)

start   = time.time()
buffer  = ''
lat     = 0.0
lon     = 0.0
alt     = 0.0
sats    = 0
fix     = 0
hdop    = 99.99

try:
    while True:
        raw = ser.read(ser.in_waiting or 1)
        if raw:
            buffer += raw.decode('ascii', errors='ignore')

        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            line = line.strip()
            if not line.startswith('$'):
                continue

            parts = line.split(',')
            msg = parts[0]

            ##GGA — fix quality, sats, lat, lon, alt, hdop
            if msg in ('$GPGGA', '$GNGGA', '$GLGGA'):
                try:
                    if len(parts) >= 10:
                        fix  = int(parts[6]) if parts[6] else 0
                        sats = int(parts[7]) if parts[7] else 0
                        hdop = float(parts[8]) if parts[8] else 99.99
                        if fix > 0 and parts[2] and parts[4]:
                            lat = parse_latlon(parts[2], parts[3], is_lat=True)
                            lon = parse_latlon(parts[4], parts[5], is_lat=False)
                            alt = float(parts[9]) if parts[9] else 0.0

                        elapsed  = time.time() - start
                        fix_name = FIX_NAMES.get(fix, f'TYPE={fix}')
                        bar = '#' * sats + '.' * max(0, 12 - sats)
                        print(f'{elapsed:8.1f}  {fix_name:<12} [{bar}] {sats:>2}  '
                              f'{lat:>12.6f} {lon:>13.6f}  {alt:>8.1f}  {hdop:>6.2f}')

                        if fix >= 1 and sats >= 6:
                            print(f'          *** GPS FIX — {sats} satellites ***')
                except (ValueError, IndexError):
                    pass

except KeyboardInterrupt:
    print('\nStopped.')
finally:
    ser.close()
