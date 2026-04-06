#!/usr/bin/env python3
"""
test_connections.py
-------------------
Run this on the Pi to verify Pixhawk (USB) and telemetry radio (UART) are working
before launching fast.py.

Usage:
    sudo python3 test_connections.py

Expected output:
    [PIXHAWK] Heartbeat received!  System: 1  Component: 1
    [RADIO]   MAVLink link active on /dev/ttyAMA0
    All connections OK.
"""

import sys
import time

PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 115200

RADIO_PORT   = '/dev/ttyAMA0'
RADIO_BAUD   = 57600

# -------------------------------------------------------------------------
# Check pymavlink is installed
# -------------------------------------------------------------------------
try:
    from pymavlink import mavutil
except ImportError:
    print('ERROR: pymavlink not installed.')
    print('Run: sudo pip3 install pymavlink')
    sys.exit(1)

# -------------------------------------------------------------------------
# Test 1 — Pixhawk via USB
# -------------------------------------------------------------------------
print(f'\n[PIXHAWK] Connecting on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud...')
try:
    px = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
    hb = px.wait_heartbeat(timeout=10)
    if hb:
        print(f'[PIXHAWK] Heartbeat received!  '
              f'System: {px.target_system}  Component: {px.target_component}')
    else:
        print('[PIXHAWK] ERROR: No heartbeat within 10 seconds.')
        print('  - Is the Pixhawk powered on?')
        print('  - Is the USB cable plugged in?')
        sys.exit(1)
except Exception as e:
    print(f'[PIXHAWK] ERROR: Could not connect: {e}')
    print(f'  - Check {PIXHAWK_PORT} exists: ls /dev/ttyACM*')
    sys.exit(1)

# -------------------------------------------------------------------------
# Test 2 — Request GPS data from Pixhawk
# -------------------------------------------------------------------------
print('\n[PIXHAWK] Requesting GPS data...')
px.mav.request_data_stream_send(
    px.target_system,
    px.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    2,   # 2 Hz
    1    # start
)
gps_ok = False
for _ in range(20):
    msg = px.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
    if msg:
        lat  = msg.lat  / 1e7
        lon  = msg.lon  / 1e7
        fix  = msg.fix_type   # 0=no fix, 3=3D fix
        sats = msg.satellites_visible
        print(f'[PIXHAWK] GPS fix_type={fix}  sats={sats}  '
              f'lat={lat:.6f}  lon={lon:.6f}')
        gps_ok = True
        break
if not gps_ok:
    print('[PIXHAWK] WARNING: No GPS message received. '
          'Check M8N is plugged into Pixhawk GPS port and has antenna attached.')

# -------------------------------------------------------------------------
# Test 3 — Request attitude from Pixhawk
# -------------------------------------------------------------------------
print('\n[PIXHAWK] Requesting attitude data...')
px.mav.request_data_stream_send(
    px.target_system,
    px.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    10,  # 10 Hz
    1
)
att_ok = False
for _ in range(20):
    msg = px.recv_match(type='ATTITUDE', blocking=True, timeout=1)
    if msg:
        import math
        roll  = math.degrees(msg.roll)
        pitch = math.degrees(msg.pitch)
        yaw   = math.degrees(msg.yaw)
        print(f'[PIXHAWK] Attitude  roll={roll:.1f}°  pitch={pitch:.1f}°  yaw={yaw:.1f}°')
        att_ok = True
        break
if not att_ok:
    print('[PIXHAWK] WARNING: No ATTITUDE message received.')

# -------------------------------------------------------------------------
# Test 4 — Telemetry radio via UART
# -------------------------------------------------------------------------
print(f'\n[RADIO]   Connecting on {RADIO_PORT} at {RADIO_BAUD} baud...')
try:
    radio = mavutil.mavlink_connection(RADIO_PORT, baud=RADIO_BAUD)
    hb = radio.wait_heartbeat(timeout=10)
    if hb:
        print(f'[RADIO]   MAVLink link active on {RADIO_PORT}')
    else:
        print('[RADIO]   WARNING: No heartbeat on radio within 10 seconds.')
        print('  - Is the air-side radio powered and plugged into UART?')
        print('  - Is the ground-side radio plugged into the laptop?')
        print('  - Check both radios have the same baud/net ID in Mission Planner')
except Exception as e:
    print(f'[RADIO]   WARNING: Could not open {RADIO_PORT}: {e}')
    print('  - Enable UART: sudo raspi-config → Interface Options → Serial')
    print('  - Wiring: Radio TX → Pi Pin 10 (RXD), Radio RX → Pi Pin 8 (TXD)')

# -------------------------------------------------------------------------
# Summary
# -------------------------------------------------------------------------
print('\n' + '-'*40)
if gps_ok and att_ok:
    print('Pixhawk OK — ready to run fast.py')
else:
    print('Pixhawk connected but GPS or attitude not streaming yet.')
    print('If GPS shows fix_type < 3, move antenna outdoors and wait for lock.')
print('-'*40 + '\n')
