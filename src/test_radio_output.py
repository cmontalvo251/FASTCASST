#!/usr/bin/env python3
"""
test_radio_output.py
---------------------
Reads GPS, compass, motor, and servo data from Pixhawk and streams
it to the ground station over the telemetry radio.

Packet layout (12 slots):
  0  - Time (s)
  1  - Roll (deg)
  2  - Pitch (deg)
  3  - Compass heading / Yaw (deg)
  4  - Latitude (deg)
  5  - Longitude (deg)
  6  - Baro altitude (m)
  7  - GPS speed (m/s)
  8  - Motor 1 command
  9  - Motor 2 command
  10 - Rudder / Servo command
  11 - GPS fix quality (0=none, 3=3D fix)

Usage:
    sudo python3 src/test_radio_output.py

Press Ctrl+C to stop.
"""

import sys
import time
import math

sys.path.append('../libraries/')
sys.path.append('../libraries/Util')
from Comms.Comms import Comms
import util

from pymavlink import mavutil

PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 115200
RADIO_PORT   = '/dev/ttyAMA0'
RADIO_BAUD   = 57600
SEND_RATE    = 1.0   # seconds between telemetry packets

# -------------------------------------------------------------------------
# Connect to Pixhawk
# -------------------------------------------------------------------------
print(f'Connecting to Pixhawk on {PIXHAWK_PORT}...')
px = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
px.wait_heartbeat()
print(f'Pixhawk connected.\n')

##Request GPS at 5 Hz
px.mav.request_data_stream_send(
    px.target_system, px.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1)

##Request attitude at 20 Hz
px.mav.request_data_stream_send(
    px.target_system, px.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20, 1)

##Request VFR_HUD (speed, baro alt) at 10 Hz
px.mav.request_data_stream_send(
    px.target_system, px.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10, 1)

# -------------------------------------------------------------------------
# Connect radio
# -------------------------------------------------------------------------
ser = Comms(packet_size=12)
ser.SerialInit(RADIO_BAUD, RADIO_PORT, period=SEND_RATE)
if ser.hComm is not None:
    print(f'Radio open on {RADIO_PORT} at {RADIO_BAUD} baud.\n')
else:
    print(f'WARNING: Radio not connected. Data will print to console only.\n')

# -------------------------------------------------------------------------
# State
# -------------------------------------------------------------------------
lat       = 0.0
lon       = 0.0
baro_alt  = 0.0
speed     = 0.0
fix_type  = 0
roll      = 0.0
pitch     = 0.0
heading   = 0.0

##Fake motor/servo values for testing (replace with real RC commands when integrated)
motor1  = 0.0
motor2  = 0.0
rudder  = 0.0

start     = time.time()
last_send = -SEND_RATE

FIX_NAMES = {0:'NO FIX', 1:'NO FIX', 2:'2D', 3:'3D', 4:'DGPS', 5:'RTK Float', 6:'RTK Fixed'}

print('Streaming telemetry to ground station. Press Ctrl+C to stop.\n')
print(f'{"Time":>8}  {"Fix":<8} {"Lat":>12} {"Lon":>13}  {"Hdg":>7}  '
      f'{"M1":>6} {"M2":>6} {"RUD":>6}')
print('-' * 80)

# -------------------------------------------------------------------------
# Main loop
# -------------------------------------------------------------------------
try:
    while True:
        ##Drain Pixhawk messages
        msg = px.recv_match(blocking=True, timeout=0.05)
        if msg:
            mtype = msg.get_type()
            if mtype == 'GPS_RAW_INT':
                fix_type = msg.fix_type
                if fix_type >= 3:
                    lat   = msg.lat / 1e7
                    lon   = msg.lon / 1e7
            elif mtype == 'ATTITUDE':
                roll    = math.degrees(msg.roll)
                pitch   = math.degrees(msg.pitch)
                heading = (math.degrees(msg.yaw) + 360) % 360
            elif mtype == 'VFR_HUD':
                speed    = msg.groundspeed
                baro_alt = msg.alt

        ##Send telemetry at SEND_RATE
        now = time.time()
        elapsed = now - start
        if (now - last_send) >= SEND_RATE:
            last_send = now

            ser.fast_packet[0]  = elapsed
            ser.fast_packet[1]  = roll
            ser.fast_packet[2]  = pitch
            ser.fast_packet[3]  = heading
            ser.fast_packet[4]  = lat
            ser.fast_packet[5]  = lon
            ser.fast_packet[6]  = baro_alt
            ser.fast_packet[7]  = speed
            ser.fast_packet[8]  = motor1
            ser.fast_packet[9]  = motor2
            ser.fast_packet[10] = rudder
            ser.fast_packet[11] = float(fix_type)

            if ser.hComm is not None:
                ser.SerialSend(0)

            fix_name = FIX_NAMES.get(fix_type, str(fix_type))
            print(f'{elapsed:8.1f}  {fix_name:<8} {lat:>12.6f} {lon:>13.6f}  '
                  f'{heading:>6.1f}°  '
                  f'{motor1:>6.2f} {motor2:>6.2f} {rudder:>6.2f}')

except KeyboardInterrupt:
    print('\nStopped.')
