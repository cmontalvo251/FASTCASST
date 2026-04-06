#!/usr/bin/env python3
"""
test_gps_fix.py
----------------
Connects to Pixhawk and continuously prints GPS fix status and satellite count.
Run this outside with the antenna in clear sky view.

Usage:
    sudo python3 src/test_gps_fix.py

Press Ctrl+C to stop.
"""

import time
from pymavlink import mavutil

PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 115200

FIX_NAMES = {
    0: 'NO FIX',
    1: 'NO FIX',
    2: '2D FIX',
    3: '3D FIX',
    4: 'DGPS',
    5: 'RTK FLOAT',
    6: 'RTK FIXED',
}

print(f'Connecting to Pixhawk on {PIXHAWK_PORT}...')
px = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
px.wait_heartbeat()
print(f'Pixhawk connected.\n')

##Request GPS at 2 Hz
px.mav.request_data_stream_send(
    px.target_system, px.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1)

print('Waiting for GPS data — make sure antenna has clear sky view...')
print('You need fix_type=3 and sats >= 6 to navigate.\n')
print(f'{"Time":>8}  {"Fix":<12} {"Sats":>5}  {"Lat":>12} {"Lon":>13}  {"Alt(m)":>8}  {"HDOP":>6}')
print('-' * 80)

start = time.time()
try:
    while True:
        msg = px.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if msg is None:
            print('  No GPS message received — check M8N is plugged into Pixhawk GPS port')
            continue

        fix    = msg.fix_type
        sats   = msg.satellites_visible
        lat    = msg.lat  / 1e7
        lon    = msg.lon  / 1e7
        alt    = msg.alt  / 1000.0
        hdop   = msg.eph  / 100.0   # horizontal dilution of precision
        elapsed = time.time() - start

        fix_name = FIX_NAMES.get(fix, f'TYPE={fix}')

        ##Visual satellite bar
        bar = '#' * sats + '.' * max(0, 12 - sats)

        print(f'{elapsed:8.1f}  {fix_name:<12} [{bar}] {sats:>2}  '
              f'{lat:>12.6f} {lon:>13.6f}  {alt:>8.1f}  {hdop:>6.2f}')

        if fix >= 3:
            print(f'          *** 3D FIX ACQUIRED — {sats} satellites ***')

except KeyboardInterrupt:
    print('\nStopped.')
