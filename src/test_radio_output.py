#!/usr/bin/env python3
"""
test_radio_output.py
---------------------
Reads GPS (Navio2 SPI) and compass (Pixhawk USB) then streams
a 12-slot FASTCASST telemetry packet to the ground station via the
SiK radio on Pixhawk TELEM1 (MAVLink SERIAL_CONTROL injection).

Packet layout (12 slots):
  0  - Time (s)
  1  - Roll (deg)
  2  - Pitch (deg)
  3  - Compass heading / Yaw (deg)
  4  - Latitude (deg)
  5  - Longitude (deg)
  6  - Baro altitude (m)  [0.0 — baro not in this test]
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
sys.path.append('/home/pi/FASTCASST/libraries/Ublox')
import ublox
from pymavlink import mavutil

GPS_PORT     = 'spi:0.0'
GPS_BAUD     = 5000000
PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 115200
SEND_RATE    = 1.0

# -------------------------------------------------------------------------
# Connect Navio2 GPS via SPI
# -------------------------------------------------------------------------
print(f'Opening Navio2 GPS via {GPS_PORT}...')
try:
    ubl = ublox.UBlox(GPS_PORT, baudrate=GPS_BAUD, timeout=2)
    ubl.configure_poll_port()
    ubl.configure_port(port=ublox.PORT_SERIAL1, inMask=1, outMask=0)
    ubl.configure_port(port=ublox.PORT_USB,     inMask=1, outMask=1)
    ubl.configure_port(port=ublox.PORT_SERIAL2, inMask=1, outMask=0)
    ubl.configure_solution_rate(rate_ms=200)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_PVT, 1)
    print('GPS initialized.\n')
except Exception as e:
    print(f'ERROR: Could not open GPS: {e}')
    sys.exit(1)

# -------------------------------------------------------------------------
# Connect Pixhawk
# -------------------------------------------------------------------------
print(f'Connecting to Pixhawk on {PIXHAWK_PORT}...')
px = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
px.wait_heartbeat()
print(f'Pixhawk connected.\n')

px.mav.request_data_stream_send(
    px.target_system, px.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20, 1)

# -------------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------------
def float_to_hex(num):
    bin_str = ''.join('{:0>8b}'.format(c) for c in struct.pack('!f', num))
    int_val = int(bin_str, 2)
    if int_val == 0:
        return '00000000'
    return hex(int_val).replace('0x', '')

def send_packet(px_conn, packet_array, baud=57600):
    packet_str = ''.join(f'{i}:{float_to_hex(n)}\r' for i, n in enumerate(packet_array))
    data_bytes  = packet_str.encode('ascii')
    for offset in range(0, len(data_bytes), 70):
        chunk  = data_bytes[offset:offset + 70]
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
sats    = 0
fix     = 0
heading = 0.0
roll    = 0.0
pitch   = 0.0

FIX_NAMES = {0:'NO FIX', 1:'GPS', 2:'DGPS', 3:'3D FIX', 4:'RTK Fixed', 5:'RTK Float'}

start     = time.time()
last_send = -SEND_RATE

print('Streaming telemetry to ground station via Pixhawk TELEM1. Press Ctrl+C to stop.\n')
print(f'{"Time":>8}  {"Fix":<8} {"Lat":>12} {"Lon":>13}  {"Hdg":>7}  {"Sats":>5}  {"Speed":>6}')
print('-' * 75)

# -------------------------------------------------------------------------
# Main loop
# -------------------------------------------------------------------------
try:
    while True:
        ##--- GPS from Navio2 SPI ---
        try:
            gps_msg = ubl.receive_message()
            if gps_msg and gps_msg.name() == 'NAV_PVT':
                gps_msg.unpack()
                fix   = gps_msg._fields.get('fixType', 0)
                sats  = gps_msg._fields.get('numSV',   0)
                if fix >= 3:
                    lat   = gps_msg._fields.get('lat',    0) * 1e-7
                    lon   = gps_msg._fields.get('lon',    0) * 1e-7
                    speed = gps_msg._fields.get('gSpeed', 0) * 1e-3
        except Exception:
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

            packet = [elapsed, roll, pitch, heading, lat, lon,
                      0.0, speed, 0.0, 0.0, 0.0, float(fix)]
            send_packet(px, packet)

            fix_name = FIX_NAMES.get(fix, str(fix))
            print(f'{elapsed:8.1f}  {fix_name:<8} {lat:>12.6f} {lon:>13.6f}  '
                  f'{heading:>6.1f}°  {sats:>5}  {speed:>6.2f}')

except KeyboardInterrupt:
    print('\nStopped.')
finally:
    ubl.close()
