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

##Pixhawk connected to Pi via USB
PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 115200

##Telemetry radio connected to Pi via UART
RADIO_PORT = '/dev/ttyAMA0'
RADIO_BAUD = 57600
################################################

##Import basic utilities
import numpy as np
import sys
import time

##Import the vehicle controller based on your selection
sys.path.append('../libraries/V_'+VEHICLE)
import controller
vehicle = controller.CONTROLLER()

##Make sure Ardupilot is off
sys.path.append('../libraries/Util')
import util
util.check_apm()

##Setup Pixhawk (GPS + IMU + Baro all come from Pixhawk via MAVLink)
sys.path.append('../libraries/Pixhawk')
import pixhawk
px = pixhawk.PIXHAWK(port=PIXHAWK_PORT, baud=PIXHAWK_BAUD)

##Setup datalogger
sys.path.append('../libraries/Datalogger')
import datalogger
logger = datalogger.Datalogger(NUMOUTPUTS)

##Setup LED
sys.path.append('../libraries/LED')
import leds
led = leds.Led()

##Setup RCIO (receiver signals and output pwm signals)
sys.path.append('../libraries/RCIO/Python')
import rcio
rc = rcio.RCIO(NUMPWM)

##Setup Telemetry (radio serial link to ground station)
sys.path.append('../libraries/')
from Comms.Comms import Comms as U
ser = U()
ser.SerialInit(RADIO_BAUD, RADIO_PORT, period=1.0)

##Short break to build suspense
print('Sleep for 1 second.....')
time.sleep(1)

##Create a time for elapsed time
print('Setting up Time')
StartTime = time.time()
RunTime = 0.0
logTime = RunTime
telemetryTime = RunTime

##This runs on repeat until code is killed
print('Running main loop....')

while (True):

    ##Get Time
    LastTime = RunTime
    RunTime = time.time() - StartTime
    elapsedTime = RunTime - LastTime

    ##Read in receiver commands
    ARMED, safety_color = rc.rcin.readALL()

    ##Get attitude, GPS, and baro from Pixhawk via MAVLink
    a, gdegs, m, rpy, rpy_ahrs, temp = px.getALL(elapsedTime)

    ##Poll GPS update from Pixhawk
    px.poll(RunTime)

    ##Run the control loop
    controls, defaults, control_color = vehicle.loop(
        RunTime,
        rc.rcin,
        gps_lat=px.latitude,
        gps_lon=px.longitude,
        heading_deg=rpy_ahrs[2]
    )

    ##Check if we are armed or not
    if ARMED:
        led.setColor(control_color)
        pwm_commands = controls
    else:
        led.setColor(safety_color)
        pwm_commands = defaults

    ##Send PWM signals to rcio
    rc.set_commands(pwm_commands)

    ##Print to console
    str_pwm = [f"{pwm:1.3f}" for pwm in pwm_commands]
    str_rpy = [f"{ang:3.3f}" for ang in rpy_ahrs]
    str_g   = [f"{gi:2.3f}" for gi in gdegs]
    gps_fix_str = f"FIX={px.fix_quality} SATS={px.num_satellites}"
    print(f"{RunTime:4.4f}", f"{elapsedTime:1.4f}",
          f"GPS={px.latitude:.6f},{px.longitude:.6f}", gps_fix_str,
          str_pwm, str_rpy, str_g,
          f"BARO={px.ALT:.3f}")

    ##Check for incoming waypoint commands from ground station
    ##  Format: "W:LAT:LON\r"  (e.g. "W:32.694500:-88.100000\r")
    if ser.hComm is not None:
        try:
            if ser.hComm.in_waiting > 0:
                raw = ser.hComm.readline()
                line = raw.decode('ascii', errors='ignore').strip()
                if line.startswith('W:'):
                    parts = line.split(':')
                    if len(parts) == 3:
                        wp_lat = float(parts[1])
                        wp_lon = float(parts[2])
                        vehicle.set_waypoint(wp_lat, wp_lon)
        except Exception as e:
            print(f'Waypoint receive error: {e}')

    ##Send Telemetry to ground station (every 1 second)
    if (RunTime - telemetryTime) > 1.0:
        telemetryTime = RunTime
        print('Sending telemetry packet...', RunTime)
        ser.fast_packet[0] = RunTime          # 0 - Time
        ser.fast_packet[1] = rpy_ahrs[0]      # 1 - roll
        ser.fast_packet[2] = rpy_ahrs[1]      # 2 - pitch
        ser.fast_packet[3] = rpy_ahrs[2]      # 3 - yaw (compass heading)
        ser.fast_packet[4] = px.latitude       # 4 - latitude
        ser.fast_packet[5] = px.longitude      # 5 - longitude
        ser.fast_packet[6] = px.ALT            # 6 - altitude (barometer)
        ser.fast_packet[7] = px.speed          # 7 - speed (GPS, m/s)
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
        logger.outdata[7]  = px.latitude
        logger.outdata[8]  = px.longitude
        logger.outdata[9]  = px.altitude
        logger.outdata[10] = px.PRES
        logger.outdata[11] = rpy_ahrs[0]
        logger.outdata[12] = rpy_ahrs[1]
        logger.outdata[13] = rpy_ahrs[2]
        logger.outdata[14] = px.speed
        logger.outdata[15] = gdegs[0]
        logger.outdata[16] = gdegs[1]
        logger.outdata[17] = gdegs[2]
        logger.outdata[18] = pwm_commands[0]
        logger.outdata[19] = pwm_commands[1]
        if len(pwm_commands) > 2:
            logger.outdata[20] = pwm_commands[2]
        if len(pwm_commands) > 3:
            logger.outdata[21] = pwm_commands[3]
        logger.println()
