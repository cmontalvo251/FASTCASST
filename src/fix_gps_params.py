#!/usr/bin/env python3
"""
fix_gps_params.py
------------------
Reads and (optionally) fixes the ArduPilot parameters required for the
NEO-M8N GPS to be detected by the Pixhawk.

Required parameters:
  GPS_TYPE          = 1   (auto-detect)
  SERIAL3_PROTOCOL  = 5   (GPS)
  SERIAL3_BAUD      = 38  (38 = 38400 baud — M8N default)

Usage:
    sudo python3 src/fix_gps_params.py           # check only
    sudo python3 src/fix_gps_params.py --fix     # check and apply fixes

Press Ctrl+C to stop.
"""

import sys
import time
from pymavlink import mavutil

PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 115200

REQUIRED = {
    'GPS_TYPE':         1,
    'SERIAL3_PROTOCOL': 5,
    'SERIAL3_BAUD':     38,
}

FIX_MODE = '--fix' in sys.argv

# -------------------------------------------------------------------------
print(f'Connecting to Pixhawk on {PIXHAWK_PORT}...')
px = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
px.wait_heartbeat()
print(f'Connected. System {px.target_system}  Component {px.target_component}\n')

# -------------------------------------------------------------------------
def get_param(name, timeout=5):
    """Request and return a single parameter value, or None on timeout."""
    px.mav.param_request_read_send(
        px.target_system, px.target_component,
        name.encode('utf-8'), -1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = px.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg and msg.param_id.rstrip('\x00') == name:
            return msg.param_value
    return None

def set_param(name, value):
    """Set a parameter and wait for confirmation."""
    px.mav.param_set_send(
        px.target_system, px.target_component,
        name.encode('utf-8'),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    deadline = time.time() + 5
    while time.time() < deadline:
        msg = px.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg and msg.param_id.rstrip('\x00') == name:
            return msg.param_value
    return None

# -------------------------------------------------------------------------
print(f'{"Parameter":<22} {"Current":>10}  {"Required":>10}  {"Status"}')
print('-' * 60)

needs_reboot = False

for param, required in REQUIRED.items():
    current = get_param(param)
    if current is None:
        print(f'{param:<22} {"TIMEOUT":>10}  {required:>10}  !! could not read')
        continue

    ok = int(current) == required
    status = 'OK' if ok else f'WRONG — should be {required}'
    print(f'{param:<22} {int(current):>10}  {required:>10}  {status}')

    if not ok:
        if FIX_MODE:
            result = set_param(param, required)
            if result is not None:
                print(f'  --> Set to {int(result)}  {"OK" if int(result)==required else "FAILED"}')
                needs_reboot = True
            else:
                print(f'  --> Set FAILED (no confirmation)')
        else:
            print(f'  --> Run with --fix to correct this')

print()

if FIX_MODE and needs_reboot:
    print('Parameters updated. You MUST reboot the Pixhawk for GPS changes to take effect.')
    print('Unplug the USB cable, wait 5 seconds, replug it.')
elif not FIX_MODE:
    print('Run with --fix to apply any corrections:')
    print('  sudo python3 src/fix_gps_params.py --fix')
else:
    print('All parameters already correct — no changes needed.')
    print('If GPS still shows sats=0, check the physical GPS connector on the Pixhawk.')
