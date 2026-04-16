#############################################
#
#  FASTCASSTPy - Facility for Aerial Systems and Technology
#  Configurable Autopilot Simulation and Software Tool in Python
#
#  Initially Created: Spring 2022
#  Primary Author: Julia Nelson Spring 2022
#  Secondary Author: Maxwell Cobar Spring 2023
#  Tertiary Authors: Aramis Hoffmann (car.py)
#  Kate Doiron (plane.py) Spring 2025
#  Quaternary Author: Carlos Montalvo Fall 2025
#
################################################

#####################PARAMETERS#################
NUMOUTPUTS = 21  #Number of data outputs (20 for car, 21 for boat, 22 for airplane)
NUMPWM = 3       #Number of PWM signals (2 for car, 3 for boat, 4 for airplane)
VEHICLE = 'boat' #Options are 'car', 'boat', or 'airplane'

##Waypoints file — loaded on startup. Path is relative to the src/ directory.
##Format: one  lat, lon  per line. Blank lines and # comments are ignored.
##If the file is missing or empty, no default mission is set.
WAYPOINTS_FILE = '../waypoints.txt'

##Pixhawk connected to Pi via USB — provides roll, pitch, and yaw via EKF
PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 115200

##Navio2 built-in GPS — SPI (GPS chip on HAT, not UART)
GPS_PORT = 'spi:0.0'
GPS_BAUD = 5000000

##SiK telemetry radio — Navio2 expansion header UART (ttyAMA0 is free now that GPS is SPI)
RADIO_PORT = '/dev/ttyAMA0'
RADIO_BAUD = 57600
################################################

import numpy as np
import sys
import time
import re
import os

##Load waypoints from file
def load_waypoints(path):
    if not os.path.exists(path):
        print(f'[WAYPOINTS] File not found: {path} — no default mission.')
        return []
    pairs = []
    with open(path) as f:
        for line in f:
            line = line.split('#')[0].strip()
            nums = re.findall(r'-?\d+\.?\d*', line)
            if len(nums) >= 2:
                pairs.append((float(nums[0]), float(nums[1])))
    print(f'[WAYPOINTS] Loaded {len(pairs)} waypoint(s) from {path}')
    return pairs

_waypoints_from_file = load_waypoints(WAYPOINTS_FILE)
_mission_loaded      = False   # set True once GPS fix arms the mission

##Vehicle controller
sys.path.insert(0, '../libraries/V_'+VEHICLE)
import controller
vehicle = controller.CONTROLLER()

##Make sure Ardupilot is off
sys.path.append('../libraries/Util')
import util
util.check_apm()

##GPS (Navio2 SPI)
sys.path.append('../libraries')
from GPS.gps import GPS
gps_sensor = GPS(port=GPS_PORT, baud=GPS_BAUD)

##IMU (Navio2 MPU9250 via SPI)
sys.path.append('../libraries/MPU9250')
from mpu9250 import MPU9250
imu = MPU9250()

##Barometer (Navio2 MS5611)
sys.path.append('../libraries/MS5611')
from ms5611 import MS5611
baro = MS5611()
baro.calibrate()

##Pixhawk — compass/yaw only via USB MAVLink
sys.path.append('../libraries/Pixhawk')
import pixhawk
px = pixhawk.PIXHAWK(port=PIXHAWK_PORT, baud=PIXHAWK_BAUD)

##Datalogger
sys.path.append('../libraries/Datalogger')
import datalogger
logger = datalogger.Datalogger(NUMOUTPUTS)

##LED
sys.path.append('../libraries/LED')
import leds
led = leds.Led()

##RCIO
sys.path.append('../libraries/RCIO/Python')
import rcio
rc = rcio.RCIO(NUMPWM)

##Telemetry radio — direct serial on ttyAMA0
from Comms.Comms import Comms as U
ser = U(packet_size=12)
ser.SerialInit(RADIO_BAUD, RADIO_PORT, period=1.0)

print('Sleep for 1 second.....')
time.sleep(1)

print('Setting up Time')
StartTime = time.time()
RunTime = 0.0
logTime = RunTime
telemetryTime = RunTime
printTime = RunTime

print('Running main loop....')

while (True):

  try:

    ##Time
    LastTime = RunTime
    RunTime  = time.time() - StartTime
    elapsedTime = RunTime - LastTime

    ##RC input
    ARMED, safety_color = rc.rcin.readALL()

    ##IMU (Navio2) — only used for raw sensor logging; attitude comes from Pixhawk EKF
    a, gdegs, m, rpy, rpy_ahrs, temp = imu.getALL(elapsedTime)

    ##Barometer
    baro.poll(RunTime)

    ##GPS (Navio2 SPI)
    gps_sensor.poll(RunTime)

    ##Load waypoint file mission once we have a valid GPS fix (sets correct return-to-home)
    if not _mission_loaded and _waypoints_from_file and gps_sensor.fix_quality > 0:
        _mission_loaded = True
        start_lat = gps_sensor.latitude
        start_lon = gps_sensor.longitude
        if len(_waypoints_from_file) == 1:
            vehicle.set_waypoint(_waypoints_from_file[0][0], _waypoints_from_file[0][1])
        else:
            vehicle.set_mission(_waypoints_from_file, start_lat, start_lon)

    ##Attitude from Pixhawk EKF (roll, pitch, yaw) — replaces Navio2 AHRS
    ##Offsets zero roll/pitch when boat is on flat ground.
    px.poll(RunTime)
    rpy_ahrs[0] = px.roll  - 5.6
    rpy_ahrs[1] = px.pitch - 0.8
    rpy_ahrs[2] = px.yaw

    ##Autopilot / manual control
    controls, defaults, control_color = vehicle.loop(
        RunTime,
        rc.rcin,
        gps_lat=gps_sensor.latitude,
        gps_lon=gps_sensor.longitude,
        heading_deg=px.yaw
    )

    ##Armed check — failsafe overrides disarm so the boat can finish its mission
    ##if the RC transmitter goes out of range.
    if ARMED or vehicle.failsafe_active:
        led.setColor(control_color)
        pwm_commands = controls
    else:
        led.setColor(safety_color)
        pwm_commands = defaults

    ##PWM output
    rc.set_commands(pwm_commands)

    ##Console print
    str_pwm = [f"{pwm:1.3f}" for pwm in pwm_commands]
    str_rpy = [f"{ang:3.3f}" for ang in rpy_ahrs]
    if vehicle._mission and vehicle.waypoint_active:
        total_wps = len(vehicle._mission)
        wp_str = (f'WP={vehicle._wp_index+1}/{total_wps} '
                  f'({vehicle.target_lat:.5f},{vehicle.target_lon:.5f})')
    elif vehicle._mission and vehicle._mission_complete:
        wp_str = 'MISSION_COMPLETE'
    elif vehicle.waypoint_active and vehicle.target_lat is not None:
        wp_str = f'WP={vehicle.target_lat:.5f},{vehicle.target_lon:.5f}'
    else:
        wp_str = 'WP=none'
    ##Console print at 1 Hz to prevent stdout blocking the loop
    if (RunTime - printTime) > 1.0:
        printTime = RunTime
        print(f"{RunTime:4.4f}", f"{elapsedTime:1.4f}",
              f"GPS={gps_sensor.latitude:.6f},{gps_sensor.longitude:.6f}",
              f"FIX={gps_sensor.fix_quality} SATS={gps_sensor.num_satellites}",
              f"HDG={px.yaw:.1f}", wp_str, str_pwm, str_rpy,
              f"BARO={baro.ALT:.3f}")

    ##Receive waypoint commands from ground station
    ##  Ground station sends: "W:LAT:LON\r"
    if ser.hComm is not None:
        try:
            if ser.hComm.in_waiting > 0:
                raw  = ser.hComm.readline()
                line = raw.decode('ascii', errors='ignore').strip()
                if line.startswith('W:'):
                    parts = line.split(':')
                    if len(parts) == 3:
                        wp_lat = float(parts[1])
                        wp_lon = float(parts[2])
                        vehicle.set_waypoint(wp_lat, wp_lon)
                        print(f'[AUTOPILOT] Waypoint received from ground station: {wp_lat:.6f}, {wp_lon:.6f}')
                elif line.startswith('MISSION:'):
                    ##Format: MISSION:lat1:lon1:lat2:lon2:...\r
                    parts = line.split(':')
                    coords = parts[1:]
                    if len(coords) >= 2 and len(coords) % 2 == 0:
                        waypoints = []
                        for i in range(0, len(coords), 2):
                            waypoints.append((float(coords[i]), float(coords[i+1])))
                        ##Use current GPS position as the return-to-start coordinate
                        start_lat = gps_sensor.latitude
                        start_lon = gps_sensor.longitude
                        vehicle.set_mission(waypoints, start_lat, start_lon)
                        print(f'[AUTOPILOT] Mission received: {len(waypoints)} waypoint(s), '
                              f'return to {start_lat:.6f},{start_lon:.6f}')
                    else:
                        print(f'[AUTOPILOT] Invalid MISSION command: {line}')
                elif line == 'CLEAR':
                    vehicle.clear_waypoint()
                    print('[AUTOPILOT] Waypoint/mission cleared by ground station')
        except Exception as e:
            print(f'Waypoint receive error: {e}')

    ##Telemetry to ground station (every 1 second)
    if (RunTime - telemetryTime) > 1.0:
        telemetryTime = RunTime
        ser.fast_packet[0]  = RunTime
        ser.fast_packet[1]  = rpy_ahrs[0]             # roll
        ser.fast_packet[2]  = rpy_ahrs[1]             # pitch
        ser.fast_packet[3]  = px.yaw                  # compass heading
        ser.fast_packet[4]  = gps_sensor.latitude
        ser.fast_packet[5]  = gps_sensor.longitude
        ser.fast_packet[6]  = baro.ALT
        ser.fast_packet[7]  = gps_sensor.speed
        ser.fast_packet[8]  = pwm_commands[0]         # motor 1
        ser.fast_packet[9]  = pwm_commands[1]         # motor 2
        ser.fast_packet[10] = pwm_commands[2] if len(pwm_commands) > 2 else 0.0  # rudder
        ser.fast_packet[11] = float(gps_sensor.fix_quality)
        print('Sending telemetry...', RunTime)
        ser.SerialSend(0)

    ##Log data
    if (RunTime - logTime) > 0.1:
        logTime = RunTime
        logger.outdata[0]  = np.round(RunTime, 5)
        logger.outdata[1]  = rc.rcin.rcsignals[0]
        logger.outdata[2]  = rc.rcin.rcsignals[1]
        logger.outdata[3]  = rc.rcin.rcsignals[2]
        logger.outdata[4]  = rc.rcin.rcsignals[3]
        logger.outdata[5]  = rc.rcin.rcsignals[4]
        logger.outdata[6]  = rc.rcin.rcsignals[5]
        logger.outdata[7]  = gps_sensor.latitude
        logger.outdata[8]  = gps_sensor.longitude
        logger.outdata[9]  = gps_sensor.altitude
        logger.outdata[10] = baro.PRES
        logger.outdata[11] = rpy_ahrs[0]
        logger.outdata[12] = rpy_ahrs[1]
        logger.outdata[13] = rpy_ahrs[2]
        logger.outdata[14] = gps_sensor.speed
        logger.outdata[15] = gdegs[0]
        logger.outdata[16] = gdegs[1]
        logger.outdata[17] = gdegs[2]
        logger.outdata[18] = pwm_commands[0]
        logger.outdata[19] = pwm_commands[1]
        if len(pwm_commands) > 2:
            logger.outdata[20] = pwm_commands[2]
        logger.println()

  except KeyboardInterrupt:
      raise
  except Exception as e:
      print(f'[LOOP ERROR] {type(e).__name__}: {e}')
