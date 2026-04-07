#!/usr/bin/env python3
"""
test_radio_output.py
---------------------
Reads GPS (Navio2 ttyAMA0) and compass (Pixhawk USB) then streams
a 12-slot FASTCASST telemetry packet to the ground station via the
SiK radio on Pixhawk TELEM1 (MAVLink SERIAL_CONTROL injection).

Packet layout (12 slots):
  0  - Time (s)
  1  - Roll (deg)
  2  - Pitch (deg)
  3  - Compass heading / Yaw (deg)
  4  - Latitude (deg)
  5  - Longitude (deg)
  6  - Baro altitude (m)  [0.0 — baro not available in this test]
  7  - GPS speed (m/s)
  8  - Motor 1 command    [0.0 — placeholder]
  9  - Motor 2 command    [0.0 — placeholder]
  10 - Rudder command     [0.0 — placeholder]
  11 - GPS fix quality

Usage:
    sudo python3 src/test_radio_output.py

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
SEND_RATE    = 1.0   # seconds between telemetry packets

# -------------------------------------------------------------------------
# Connect Navio2 GPS
# -------------------------------------------------------------------------
print(f'Opening Navio2 GPS on {GPS_PORT}...')
try:
    gps_ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
    print('GPS port open.\n')
except Exception as e:
    print(f'ERROR: Could not open {GPS_PORT}: {e}')
    sys.exit(1)

# -------------------------------------------------------------------------
# Connect Pixhawk
# -------------------------------------------------------------------------
print(f'Connecting to Pixhawk on {PIXHAWK_PORT}...')
px = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
px.wait_heartbeat()
print(f'Pixhawk connected.\n')

##Request attitude at 20 Hz
px.mav.request_data_stream_send(
    px.target_system, px.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20, 1)

# -------------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------------
def parse_latlon(value_str, direction, is_lat):
    val = float(value_str)
    deg = int(val / 100)
    minutes = val - deg * 100
    decimal = deg + minutes / 60.0
    if direction in ('S', 'W'):
        decimal = -decimal
    return decimal

def float_to_hex(num):
    bin_str = ''.join('{:0>8b}'.format(c) for c in struct.pack('!f', num))
    int_val = int(bin_str, 2)
    if int_val == 0:
        return '00000000'
    return hex(int_val).replace('0x', '')

def send_packet(px_conn, packet_array, baud=57600):
    """Inject FASTCASST packet bytes to SiK radio via Pixhawk TELEM1."""
    packet_str = ''
    for i, n in enumerate(packet_array):
        packet_str += f'{i}:{float_to_hex(n)}\r'
    data_bytes = packet_str.encode('ascii')
    for offset in range(0, len(data_bytes), 70):
        chunk = data_bytes[offset:offset + 70]
        padded = list(chunk) + [0] * (70 - len(chunk))
        px_conn.mav.serial_control_send(
            device=0, flags=0, timeout=0,
            baudrate=baud, count=len(chunk), data=padded)

# -------------------------------------------------------------------------
# State
# -------------------------------------------------------------------------
lat     = 0.0
lon     = 0.0
speed   = 0.0
fix     = 0
heading = 0.0
roll    = 0.0
pitch   = 0.0
gps_buf = ''

FIX_NAMES = {0:'NO FIX', 1:'GPS', 2:'DGPS', 3:'PPS', 4:'RTK Fixed', 5:'RTK Float'}

start     = time.time()
last_send = -SEND_RATE

print('Streaming telemetry to ground station via Pixhawk TELEM1. Press Ctrl+C to stop.\n')
print(f'{"Time":>8}  {"Fix":<8} {"Lat":>12} {"Lon":>13}  {"Hdg":>7}  '
      f'{"Sats":>5}  {"Speed":>6}')
print('-' * 75)

# -------------------------------------------------------------------------
# Main loop
# -------------------------------------------------------------------------
try:
    while True:
        ##--- GPS from Navio2 ttyAMA0 ---
        raw = gps_ser.read(gps_ser.in_waiting or 1)
        if raw:
            gps_buf += raw.decode('ascii', errors='ignore')

        sats = 0
        while '\n' in gps_buf:
            line, gps_buf = gps_buf.split('\n', 1)
            line = line.strip()
            if not line.startswith('$'):
                continue
            parts = line.split(',')
            ##GGA: fix, sats, lat, lon, alt
            if parts[0] in ('$GPGGA', '$GNGGA', '$GLGGA'):
                try:
                    if len(parts) >= 10:
                        fix  = int(parts[6]) if parts[6] else 0
                        sats = int(parts[7]) if parts[7] else 0
                        if fix > 0 and parts[2] and parts[4]:
                            lat = parse_latlon(parts[2], parts[3], is_lat=True)
                            lon = parse_latlon(parts[4], parts[5], is_lat=False)
                except (ValueError, IndexError):
                    pass
            ##RMC: speed
            elif parts[0] in ('$GPRMC', '$GNRMC'):
                try:
                    if len(parts) >= 8 and parts[2] == 'A' and parts[7]:
                        speed = float(parts[7]) * 0.514444  # knots → m/s
                except (ValueError, IndexError):
                    pass

        ##--- Compass from Pixhawk ---
        msg = px.recv_match(blocking=False)
        if msg and msg.get_type() == 'ATTITUDE':
            heading = (math.degrees(msg.yaw) + 360) % 360
            roll    = math.degrees(msg.roll)
            pitch   = math.degrees(msg.pitch)

        ##--- Send telemetry packet ---
        now     = time.time()
        elapsed = now - start
        if (now - last_send) >= SEND_RATE:
            last_send = now

            packet = [0.0] * 12
            packet[0]  = elapsed
            packet[1]  = roll
            packet[2]  = pitch
            packet[3]  = heading
            packet[4]  = lat
            packet[5]  = lon
            packet[6]  = 0.0          # baro alt — not available in this test
            packet[7]  = speed
            packet[8]  = 0.0          # motor 1 placeholder
            packet[9]  = 0.0          # motor 2 placeholder
            packet[10] = 0.0          # rudder placeholder
            packet[11] = float(fix)

            send_packet(px, packet)

            fix_name = FIX_NAMES.get(fix, str(fix))
            print(f'{elapsed:8.1f}  {fix_name:<8} {lat:>12.6f} {lon:>13.6f}  '
                  f'{heading:>6.1f}°  {sats:>5}  {speed:>6.2f}')

except KeyboardInterrupt:
    print('\nStopped.')
finally:
    gps_ser.close()
