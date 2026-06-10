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
import struct
sys.path.append('/home/pi/FASTCASST/libraries/Ublox')
import ublox
from pymavlink import mavutil

GPS_PORT     = 'spi:0.0'
GPS_BAUD     = 5000000
PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 115200

# -------------------------------------------------------------------------
# Connect Navio2 GPS (SPI)
# -------------------------------------------------------------------------
print(f'Opening Navio2 GPS via {GPS_PORT}...')
try:
    ubl = ublox.UBlox(GPS_PORT, baudrate=GPS_BAUD, timeout=2)
    ubl.configure_poll_port()
    ubl.configure_port(port=ublox.PORT_SERIAL1, inMask=1, outMask=0)
    ubl.configure_port(port=ublox.PORT_USB,     inMask=1, outMask=1)
    ubl.configure_port(port=ublox.PORT_SERIAL2, inMask=1, outMask=0)
    ubl.configure_solution_rate(rate_ms=1000)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_PVT, 1)
    print('GPS initialized.\n')
except Exception as e:
    print(f'ERROR: Could not open GPS: {e}')
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
        ##--- Read Navio2 GPS via SPI ---
        try:
            gps_msg = ubl.receive_message()
            if gps_msg and gps_msg.name() == 'NAV_PVT':
                gps_msg.unpack()
                fix  = gps_msg._fields.get('fixType', 0)
                sats = gps_msg._fields.get('numSV',   0)
                if fix >= 3:
                    lat = gps_msg._fields.get('lat', 0) * 1e-7
                    lon = gps_msg._fields.get('lon', 0) * 1e-7
                    alt = gps_msg._fields.get('height', 0) * 1e-3
        except Exception:
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
    ubl.close()
