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

##Pixhawk connected to Pi via USB — compass only
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

##Vehicle controller
sys.path.append('../libraries/V_'+VEHICLE)
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

##TEST: set a waypoint to verify Blue LED and autopilot engage
##Replace with coordinates ~50m from your current position
##Remove this when using ground station radio to send waypoints
vehicle.set_waypoint(30.6920, -88.1760)

print('Running main loop....')

while (True):

    ##Time
    LastTime = RunTime
    RunTime  = time.time() - StartTime
    elapsedTime = RunTime - LastTime

    ##RC input
    ARMED, safety_color = rc.rcin.readALL()

    ##IMU
    a, gdegs, m, rpy, rpy_ahrs, temp = imu.getALL(elapsedTime)

    ##Barometer
    baro.poll(RunTime)

    ##GPS (Navio2 SPI)
    gps_sensor.poll(RunTime)

    ##Compass from Pixhawk
    px.poll(RunTime)
    rpy_ahrs[2] = px.yaw

    ##Autopilot / manual control
    controls, defaults, control_color = vehicle.loop(
        RunTime,
        rc.rcin,
        gps_lat=gps_sensor.latitude,
        gps_lon=gps_sensor.longitude,
        heading_deg=px.yaw
    )

    ##Armed check
    if ARMED:
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
    wp_str  = (f"WP={vehicle.target_lat:.5f},{vehicle.target_lon:.5f}"
               if vehicle.waypoint_active else "WP=none")
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
                elif line == 'CLEAR':
                    vehicle.clear_waypoint()
                    print('[AUTOPILOT] Waypoint cleared by ground station')
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
