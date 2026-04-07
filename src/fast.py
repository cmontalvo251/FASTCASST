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

##Pixhawk connected to Pi via USB — used for compass heading only
##  SiK telemetry radio is connected to Pixhawk TELEM1 port
PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 115200

##Navio2 built-in GPS — SPI bus 0, chip select 0
GPS_PORT = 'spi:0.0'
GPS_BAUD = 5000000
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

##Setup GPS (Navio2 built-in NEO-M8N via ttyAMA0)
sys.path.append('../libraries')
from GPS.gps import GPS
gps_sensor = GPS(port=GPS_PORT, baud=GPS_BAUD)

##Setup IMU (Navio2 MPU9250 via SPI)
sys.path.append('../libraries/MPU9250')
from mpu9250 import MPU9250
imu = MPU9250()

##Setup Barometer (Navio2 MS5611)
sys.path.append('../libraries/MS5611')
from ms5611 import MS5611
baro = MS5611()
baro.calibrate()

##Setup Pixhawk (compass/yaw only + telemetry radio injection via TELEM1)
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

    ##Get IMU data from Navio2 MPU9250 (accel, gyro, mag, attitude)
    a, gdegs, m, rpy, rpy_ahrs, temp = imu.getALL(elapsedTime)

    ##Poll barometer
    baro.poll(RunTime)

    ##Poll GPS (Navio2 ttyAMA0)
    gps_sensor.poll(RunTime)

    ##Poll Pixhawk for compass heading (yaw)
    px.poll(RunTime)

    ##Use Pixhawk compass heading as the authoritative yaw for navigation
    rpy_ahrs[2] = px.yaw

    ##Run the control loop
    controls, defaults, control_color = vehicle.loop(
        RunTime,
        rc.rcin,
        gps_lat=gps_sensor.latitude,
        gps_lon=gps_sensor.longitude,
        heading_deg=px.yaw
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
    gps_fix_str = f"FIX={gps_sensor.fix_quality} SATS={gps_sensor.num_satellites}"
    print(f"{RunTime:4.4f}", f"{elapsedTime:1.4f}",
          f"GPS={gps_sensor.latitude:.6f},{gps_sensor.longitude:.6f}", gps_fix_str,
          str_pwm, str_rpy, str_g,
          f"BARO={baro.ALT:.3f}")

    ##Send Telemetry to ground station via Pixhawk TELEM1 → SiK radio (every 1 second)
    ##NOTE: Waypoint receive from ground station is not supported in this config —
    ##      the SiK radio is bridged through Pixhawk TELEM1 (MAVLink port).
    if (RunTime - telemetryTime) > 1.0:
        telemetryTime = RunTime
        packet = np.zeros(12)
        packet[0]  = RunTime                   # 0 - Time
        packet[1]  = rpy_ahrs[0]               # 1 - roll
        packet[2]  = rpy_ahrs[1]               # 2 - pitch
        packet[3]  = px.yaw                    # 3 - compass heading
        packet[4]  = gps_sensor.latitude       # 4 - latitude
        packet[5]  = gps_sensor.longitude      # 5 - longitude
        packet[6]  = baro.ALT                  # 6 - baro altitude (m)
        packet[7]  = gps_sensor.speed          # 7 - GPS speed (m/s)
        packet[8]  = pwm_commands[0]           # 8 - motor 1
        packet[9]  = pwm_commands[1]           # 9 - motor 2
        packet[10] = pwm_commands[2] if len(pwm_commands) > 2 else 0.0  # 10 - rudder
        packet[11] = float(gps_sensor.fix_quality)  # 11 - GPS fix quality
        print('Sending telemetry packet...', RunTime)
        px.send_to_radio(packet)

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
        if len(pwm_commands) > 3:
            logger.outdata[21] = pwm_commands[3]
        logger.println()
