#!/usr/bin/env python3
"""
test_gps_fix.py
----------------
Reads GPS from the Navio2 built-in NEO-M8N via SPI using the ublox library.
Prints fix status and satellite count continuously.

The Navio2 GPS chip is connected via SPI (spi:0.0), NOT UART.

Usage:
    sudo python3 src/test_gps_fix.py

Take the antenna outdoors with clear sky view.
You need fix_type >= 3 and sats >= 6 to navigate.
Press Ctrl+C to stop.
"""

import sys
import time

sys.path.append('/home/pi/FASTCASST/libraries/Ublox')
sys.path.append('/home/pi/FASTCASST/libraries/Util')

try:
    import ublox
except ImportError as e:
    print(f'ERROR: Could not import ublox library: {e}')
    print('Make sure you are running from the Pi with FASTCASST at /home/pi/FASTCASST')
    sys.exit(1)

FIX_NAMES = {
    0: 'NO FIX',
    1: 'DEAD REC',
    2: '2D FIX',
    3: '3D FIX',
    4: 'GPS+DR',
    5: 'TIME ONLY',
}

print('Opening Navio2 GPS via SPI (spi:0.0)...')
try:
    ubl = ublox.UBlox('spi:0.0', baudrate=5000000, timeout=2)
except Exception as e:
    print(f'ERROR: Could not open SPI GPS: {e}')
    print('Make sure SPI is enabled: sudo raspi-config -> Interface Options -> SPI')
    sys.exit(1)

print('Configuring GPS...')
ubl.configure_poll_port()
ubl.configure_port(port=ublox.PORT_SERIAL1, inMask=1, outMask=0)
ubl.configure_port(port=ublox.PORT_USB,     inMask=1, outMask=1)
ubl.configure_port(port=ublox.PORT_SERIAL2, inMask=1, outMask=0)
ubl.configure_solution_rate(rate_ms=1000)

##Request NAV_PVT — has lat, lon, fix type, sats, speed, altitude all in one
ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_PVT, 1)
##Request NAV_SVINFO — per-satellite signal strength (shows if antenna is receiving RF)
ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SVINFO, 1)
print('GPS configured.\n')

print('Waiting for GPS messages — take antenna outdoors with clear sky view...')
print('You need fix_type=3 and sats >= 6 to navigate.')
print('Signal bars show RF received — if all 0 dBHz, antenna is not connected.\n')
print(f'{"Time":>8}  {"Fix":<10} {"Sats":>5}  {"Lat":>12} {"Lon":>13}  {"Alt(m)":>8}')
print('-' * 80)

start = time.time()
last_svinfo_time = 0

try:
    while True:
        msg = ubl.receive_message()
        if msg is None:
            continue

        name = msg.name()

        if name == 'NAV_PVT':
            msg.unpack()
            fix   = msg._fields.get('fixType', 0)
            sats  = msg._fields.get('numSV', 0)
            lat   = msg._fields.get('lat',   0) * 1e-7
            lon   = msg._fields.get('lon',   0) * 1e-7
            alt   = msg._fields.get('height',0) * 1e-3

            elapsed  = time.time() - start
            fix_name = FIX_NAMES.get(fix, f'TYPE={fix}')
            bar = '#' * sats + '.' * max(0, 12 - sats)

            print(f'{elapsed:8.1f}  {fix_name:<10} [{bar}] {sats:>2}  '
                  f'{lat:>12.6f} {lon:>13.6f}  {alt:>8.1f}')

            if fix >= 3 and sats >= 6:
                print(f'          *** 3D FIX — {sats} satellites ***')

        elif name == 'NAV_SVINFO':
            ##Print signal strength every 5 seconds to diagnose antenna
            now = time.time()
            if (now - last_svinfo_time) >= 5.0:
                last_svinfo_time = now
                try:
                    msg.unpack()
                    recs = msg._recs
                    if not recs:
                        print('  [SVINFO] No satellite records — antenna may not be connected')
                    else:
                        signals = [(r.get('svid', 0), r.get('cno', 0)) for r in recs if r.get('cno', 0) > 0]
                        if signals:
                            sig_str = '  '.join(f'SV{s}:{c}dBHz' for s, c in sorted(signals, key=lambda x: -x[1])[:8])
                            print(f'  [SIGNAL] {sig_str}')
                        else:
                            print('  [SIGNAL] All satellites 0 dBHz — check antenna snap on Navio2 ANT port')
                except Exception:
                    pass

except KeyboardInterrupt:
    print('\nStopped.')
finally:
    ubl.close()
