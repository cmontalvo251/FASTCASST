#!/usr/bin/env python3
"""
test_gps_compass_radio.py
--------------------------
Tests the full new sensor architecture:
  - GPS from Navio2 NEO-M8N via ttyAMA0 (NMEA)
  - Compass heading from Pixhawk via USB (MAVLink ATTITUDE)
  - Sends a test telemetry packet via Pixhawk TELEM1 → SiK radio

Requirements:
    sudo pip3 install pyserial pymavlink

Usage:
    sudo python3 src/test_gps_compass_radio.py

Press Ctrl+C to stop.
"""

import sys
import time
import math
import serial
import struct
from pymavlink import mavutil

GPS_PORT     = '/dev/ttyAMA0'
GPS_BAUD     = 9600
PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 115200

# -------------------------------------------------------------------------
# Connect Navio2 GPS (ttyAMA0)
# -------------------------------------------------------------------------
print(f'Opening Navio2 GPS on {GPS_PORT}...')
try:
    gps_ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
    print('GPS port open.\n')
except Exception as e:
    print(f'ERROR: Could not open GPS port {GPS_PORT}: {e}')
    sys.exit(1)

# -------------------------------------------------------------------------
# Connect Pixhawk via USB (compass + radio relay)
# -------------------------------------------------------------------------
print(f'Connecting to Pixhawk on {PIXHAWK_PORT}...')
px = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
px.wait_heartbeat()
print(f'Pixhawk connected. System: {px.target_system}\n')

##Request attitude at 10 Hz
px.mav.request_data_stream_send(
    px.target_system, px.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)

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

def send_test_packet(px_conn, packet_array, baud=57600):
    """Inject FASTCASST packet to SiK radio via Pixhawk TELEM1 SERIAL_CONTROL."""
    packet = ''
    for i, n in enumerate(packet_array):
        packet += f'{i}:{float_to_hex(n)}\r'
    data_bytes = packet.encode('ascii')
    chunk_size = 70
    sent = 0
    for offset in range(0, len(data_bytes), chunk_size):
        chunk = data_bytes[offset:offset + chunk_size]
        padded = list(chunk) + [0] * (70 - len(chunk))
        px_conn.mav.serial_control_send(
            device=0, flags=0, timeout=0,
            baudrate=baud, count=len(chunk), data=padded)
        sent += len(chunk)
    return sent

# -------------------------------------------------------------------------
# State
# -------------------------------------------------------------------------
lat       = 0.0
lon       = 0.0
alt       = 0.0
sats      = 0
fix       = 0
heading   = 0.0
roll      = 0.0
pitch     = 0.0
gps_buf   = ''

FIX_NAMES = {0: 'NO FIX', 1: 'GPS', 2: 'DGPS', 3: 'PPS',
             4: 'RTK Fixed', 5: 'RTK Float'}

print('Reading GPS (Navio2) and compass (Pixhawk). Take antenna outdoors for fix.')
print('Press Ctrl+C to stop.\n')
print(f'{"Time":>8}  {"Fix":<10} {"Sats":>4}  {"Lat":>12} {"Lon":>13}  '
      f'{"Alt(m)":>8}  {"Heading":>8}  {"Roll":>7}  {"Pitch":>7}')
print('-' * 95)

start     = time.time()
last_send = -5.0

try:
    while True:
        ##--- Read Navio2 GPS NMEA from ttyAMA0 ---
        raw = gps_ser.read(gps_ser.in_waiting or 1)
        if raw:
            gps_buf += raw.decode('ascii', errors='ignore')

        while '\n' in gps_buf:
            line, gps_buf = gps_buf.split('\n', 1)
            line = line.strip()
            if not line.startswith('$'):
                continue
            parts = line.split(',')
            if parts[0] in ('$GPGGA', '$GNGGA', '$GLGGA'):
                try:
                    if len(parts) >= 10:
                        fix  = int(parts[6]) if parts[6] else 0
                        sats = int(parts[7]) if parts[7] else 0
                        if fix > 0 and parts[2] and parts[4]:
                            lat = parse_latlon(parts[2], parts[3], is_lat=True)
                            lon = parse_latlon(parts[4], parts[5], is_lat=False)
                            alt = float(parts[9]) if parts[9] else 0.0
                except (ValueError, IndexError):
                    pass

        ##--- Read Pixhawk compass (ATTITUDE) ---
        msg = px.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            heading = (math.degrees(msg.yaw) + 360) % 360
            roll    = math.degrees(msg.roll)
            pitch   = math.degrees(msg.pitch)

            elapsed  = time.time() - start
            fix_name = FIX_NAMES.get(fix, str(fix))
            print(f'{elapsed:8.1f}  {fix_name:<10} {sats:>4}  '
                  f'{lat:>12.6f} {lon:>13.6f}  '
                  f'{alt:>8.1f}  '
                  f'{heading:>7.1f}°  '
                  f'{roll:>6.1f}°  '
                  f'{pitch:>6.1f}°')

        ##--- Send test telemetry packet every 5 seconds ---
        now = time.time()
        if (now - last_send) >= 5.0:
            last_send = now
            packet = [now - start, roll, pitch, heading, lat, lon, alt, 0.0,
                      0.0, 0.0, 0.0, float(fix)]
            bytes_sent = send_test_packet(px, packet)
            print(f'          [RADIO] Sent {bytes_sent}-byte test packet via Pixhawk TELEM1')

except KeyboardInterrupt:
    print('\nStopped.')
finally:
    gps_ser.close()
