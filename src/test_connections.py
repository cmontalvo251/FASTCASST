#!/usr/bin/env python3
"""
test_connections.py
-------------------
Verifies all connections for the new hardware architecture:

  1. Navio2 GPS    — ttyAMA0 (NMEA sentences)
  2. Pixhawk USB   — ttyACM0 (MAVLink heartbeat + compass)
  3. Radio TX test — sends a test packet via Pixhawk TELEM1 SERIAL_CONTROL

Run this before launching fast.py to confirm everything is wired up.

Usage:
    sudo python3 src/test_connections.py

Press Ctrl+C to stop.
"""

import sys
import time
import math
import struct
import serial
from pymavlink import mavutil

GPS_PORT     = '/dev/ttyAMA0'
GPS_BAUD     = 9600
PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 115200

ok_count = 0

# -------------------------------------------------------------------------
# Test 1 — Navio2 GPS via ttyAMA0
# -------------------------------------------------------------------------
print(f'\n[GPS] Opening Navio2 GPS on {GPS_PORT} at {GPS_BAUD} baud...')
gps_ok = False
gps_ser = None
try:
    gps_ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=2)
    print(f'[GPS] Port open. Waiting for NMEA sentences (up to 5 seconds)...')
    buf = ''
    deadline = time.time() + 5
    while time.time() < deadline:
        raw = gps_ser.read(gps_ser.in_waiting or 1)
        if raw:
            buf += raw.decode('ascii', errors='ignore')
        if '\n' in buf:
            line, buf = buf.split('\n', 1)
            if line.startswith('$GP') or line.startswith('$GN'):
                print(f'[GPS] NMEA sentence received: {line.strip()[:60]}')
                gps_ok = True
                ok_count += 1
                break
    if not gps_ok:
        print('[GPS] WARNING: No NMEA sentences received.')
        print('  - Is the antenna connected to the Navio2 ANT port?')
        print('  - Is ttyAMA0 free (SiK radio moved to Pixhawk TELEM1)?')
except Exception as e:
    print(f'[GPS] ERROR: Could not open {GPS_PORT}: {e}')

# -------------------------------------------------------------------------
# Test 2 — Pixhawk USB heartbeat
# -------------------------------------------------------------------------
print(f'\n[PIXHAWK] Connecting on {PIXHAWK_PORT} at {PIXHAWK_BAUD} baud...')
px = None
px_ok = False
try:
    px = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
    hb = px.wait_heartbeat(timeout=10)
    if hb:
        print(f'[PIXHAWK] Heartbeat received! '
              f'System: {px.target_system}  Component: {px.target_component}')
        px_ok = True
        ok_count += 1
    else:
        print('[PIXHAWK] ERROR: No heartbeat within 10 seconds.')
        print('  - Is Pixhawk powered on and USB cable plugged in?')
except Exception as e:
    print(f'[PIXHAWK] ERROR: {e}')

# -------------------------------------------------------------------------
# Test 3 — Pixhawk compass (ATTITUDE)
# -------------------------------------------------------------------------
compass_ok = False
if px_ok:
    print('\n[COMPASS] Requesting ATTITUDE from Pixhawk...')
    px.mav.request_data_stream_send(
        px.target_system, px.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
    for _ in range(20):
        msg = px.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if msg:
            heading = (math.degrees(msg.yaw) + 360) % 360
            roll    = math.degrees(msg.roll)
            pitch   = math.degrees(msg.pitch)
            print(f'[COMPASS] heading={heading:.1f}°  roll={roll:.1f}°  pitch={pitch:.1f}°')
            compass_ok = True
            ok_count += 1
            break
    if not compass_ok:
        print('[COMPASS] WARNING: No ATTITUDE message received.')

# -------------------------------------------------------------------------
# Test 4 — Radio TX via Pixhawk TELEM1 SERIAL_CONTROL
# -------------------------------------------------------------------------
radio_ok = False
if px_ok:
    print('\n[RADIO] Sending test packet via Pixhawk TELEM1 SERIAL_CONTROL...')
    try:
        ##Build a minimal FASTCASST test packet (3 slots)
        def float_to_hex(num):
            bin_str = ''.join('{:0>8b}'.format(c) for c in struct.pack('!f', num))
            int_val = int(bin_str, 2)
            if int_val == 0:
                return '00000000'
            return hex(int_val).replace('0x', '')

        test_vals = [time.time(), 1.23, 4.56]
        packet_str = ''.join(f'{i}:{float_to_hex(v)}\r' for i, v in enumerate(test_vals))
        data_bytes = packet_str.encode('ascii')
        padded = list(data_bytes[:70]) + [0] * (70 - min(len(data_bytes), 70))

        px.mav.serial_control_send(
            device=0, flags=0, timeout=0,
            baudrate=57600, count=min(len(data_bytes), 70), data=padded)

        print(f'[RADIO] Test packet sent ({len(data_bytes)} bytes).')
        print('[RADIO] If ground station shows data — radio link is working.')
        radio_ok = True
        ok_count += 1
    except Exception as e:
        print(f'[RADIO] ERROR: {e}')
        print('  - Is SiK radio wired to Pixhawk TELEM1 port?')
        print('  - Is the ground radio plugged into the laptop (COM3)?')

# -------------------------------------------------------------------------
# Summary
# -------------------------------------------------------------------------
print('\n' + '=' * 50)
print(f'Results: {ok_count}/4 checks passed')
print(f'  GPS (Navio2 ttyAMA0):     {"OK" if gps_ok     else "FAIL"}')
print(f'  Pixhawk USB heartbeat:    {"OK" if px_ok      else "FAIL"}')
print(f'  Compass (ATTITUDE):       {"OK" if compass_ok else "FAIL"}')
print(f'  Radio TX (SERIAL_CONTROL):{"OK" if radio_ok   else "FAIL"}')
if ok_count == 4:
    print('\nAll connections OK — ready to run fast.py')
else:
    print('\nFix the failing connections before running fast.py')
print('=' * 50 + '\n')

if gps_ser:
    gps_ser.close()
