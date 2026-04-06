#!/usr/bin/env python3
"""
test_gps_compass_radio.py
--------------------------
Continuously reads GPS and compass from Pixhawk and prints to console.
Also confirms the ground radio link is active.

Usage:
    sudo python3 src/test_gps_compass_radio.py

Press Ctrl+C to stop.
"""

import time
import math
from pymavlink import mavutil

PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 115200

RADIO_PORT   = '/dev/ttyAMA0'
RADIO_BAUD   = 57600

# -------------------------------------------------------------------------
# Connect to Pixhawk
# -------------------------------------------------------------------------
print(f'Connecting to Pixhawk on {PIXHAWK_PORT}...')
px = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
px.wait_heartbeat()
print(f'Pixhawk connected. System: {px.target_system}\n')

##Request GPS at 5 Hz
px.mav.request_data_stream_send(
    px.target_system, px.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1)

##Request attitude (compass) at 10 Hz
px.mav.request_data_stream_send(
    px.target_system, px.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)

# -------------------------------------------------------------------------
# Connect to ground radio
# -------------------------------------------------------------------------
print(f'Connecting to ground radio on {RADIO_PORT}...')
try:
    radio = mavutil.mavlink_connection(RADIO_PORT, baud=RADIO_BAUD)
    hb = radio.wait_heartbeat(timeout=8)
    if hb:
        print(f'Radio link active.\n')
    else:
        print(f'WARNING: No heartbeat from radio. Is Mission Planner open on laptop?\n')
        radio = None
except Exception as e:
    print(f'WARNING: Could not open radio port: {e}\n')
    radio = None

# -------------------------------------------------------------------------
# State
# -------------------------------------------------------------------------
lat        = 0.0
lon        = 0.0
alt        = 0.0
fix_type   = 0
sats       = 0
heading    = 0.0
roll       = 0.0
pitch      = 0.0

FIX_NAMES  = {0: 'NO FIX', 1: 'NO FIX', 2: '2D FIX', 3: '3D FIX',
              4: 'DGPS',   5: 'RTK Float', 6: 'RTK Fixed'}

print('Reading GPS and compass — take antenna outside for fix.')
print('Press Ctrl+C to stop.\n')
print(f'{"Time":>8}  {"Fix":<10} {"Sats":>4}  {"Lat":>12} {"Lon":>13}  '
      f'{"Alt(m)":>8}  {"Heading":>8}  {"Roll":>7}  {"Pitch":>7}')
print('-' * 95)

# -------------------------------------------------------------------------
# Main loop
# -------------------------------------------------------------------------
start = time.time()
try:
    while True:
        msg = px.recv_match(blocking=True, timeout=0.1)
        if msg is None:
            continue

        mtype = msg.get_type()

        if mtype == 'GPS_RAW_INT':
            fix_type = msg.fix_type
            sats     = msg.satellites_visible
            if fix_type >= 3:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0

        elif mtype == 'ATTITUDE':
            heading = (math.degrees(msg.yaw) + 360) % 360
            roll    = math.degrees(msg.roll)
            pitch   = math.degrees(msg.pitch)

            elapsed  = time.time() - start
            fix_name = FIX_NAMES.get(fix_type, str(fix_type))
            print(f'{elapsed:8.1f}  {fix_name:<10} {sats:>4}  '
                  f'{lat:>12.6f} {lon:>13.6f}  '
                  f'{alt:>8.1f}  '
                  f'{heading:>7.1f}°  '
                  f'{roll:>6.1f}°  '
                  f'{pitch:>6.1f}°')

except KeyboardInterrupt:
    print('\nStopped.')
