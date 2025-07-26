#############################################
#
#  FASTCASSTPy - Facility for Aerial Systems and Technology
#  Configurable Autopilot Simulation and Software Tool in Python
#
#  Initially Created: Spring 2022
#  Primary Author: Julia Nelson Spring 2022
#  Secondary Author: Maxwell Cobar Spring 2023
#  Tertiary Author : Aramis Hoffmann (car.py) and Kate Doiron (plane.py) Spring 2025
#
################################################

#####################PARAMETERS#################
NUMOUTPUTS = 20  #Number of data outputs (20 for car and boat, 22 for airplane)
NUMPWM = 2 #Number of PWM signals for car and boat
#NUMPWM = 4 #number of PWM signals for airplane

##Pick the vehicle you want to use
import sys
#sys.path.append('../libraries/V_car')
#sys.path.append('../libraries/V_airplane')
sys.path.append('../libraries/V_boat')
import controller
vehicle = controller.CONTROLLER()

#Make sure Ardupilot is off
sys.path.append('../libraries/Util')
import util
util.check_apm()

#Setup GPS
sys.path.append('../libraries/GPS')
import gps
gps_llh = gps.GPS()

#Setup IMU
sys.path.append('../libraries/MPU9250')
import mpu9250
imu = mpu9250.MPU9250()

#Setup datalogger
sys.path.append('../libraries/Datalogger')
import datalogger
logger = datalogger.Datalogger(NUMOUTPUTS)

#Setup LED
sys.path.append('../libraries/LED')
import leds
led = leds.Led()

#Setup the Barometer
sys.path.append('../libraries/MS5611/')
import ms5611
baro = ms5611.MS5611()

#Setup RCIO (receiver signals and output pwmsignals)
sys.path.append('../libraries/RCIO/Python')
import rcio
rc = rcio.RCIO(NUMPWM)

#Short break to build suspense
print('Sleep for 1 second.....')
import time
time.sleep(1)

#Create a time for elapsed time
print('Setting up Time')
StartTime = time.time()

#This runs on repeat until code is killed
print('Running main loop....')
import numpy as np
while (True):

    #Get Time
    RunTime = time.time() - StartTime
    
    #Read in receiver commands
    rc.rcin.readALL()

    #Get acceleration, gyroscope, magnetometer & temperature data
    a,g,m,rpy,temp = imu.getALL()

    #Get GPS update if it's ready
    gps_llh.poll(RunTime)

    #Get pressure for altitude
    baro.poll(RunTime)

    #Run your control loop
    controls = vehicle.loop(RunTime,rc.rcin,gps_llh,rpy,g,baro)

    #Independent Safety precautions
    if(rc.rcin.armswitch < 0):
        led.setColor('Red')
        rc.set_defaults()
    elif(rc.rcin.armswitch > 0):
        led.setColor('Green')
        rc.set_commands(controls)

    #Print to Home
    print(f"{RunTime:4.2f}",rc.rcin.rcsignals,gps_llh.latitude,gps_llh.longitude,gps_llh.altitude,baro.ALT,rpy,gps_llh.speed,g,controls)

    #Log data
    logger.outdata[0] = np.round(RunTime,5)
    logger.outdata[1] = rc.rcin.rcsignals[0]
    logger.outdata[2] = rc.rcin.rcsignals[1]
    logger.outdata[3] = rc.rcin.rcsignals[2]
    logger.outdata[4] = rc.rcin.rcsignals[3]
    logger.outdata[5] = rc.rcin.rcsignals[4]
    logger.outdata[6] = rc.rcin.rcsignals[5]
    logger.outdata[7] = gps_llh.latitude
    logger.outdata[8] = gps_llh.longitude
    logger.outdata[9] = gps_llh.altitude
    logger.outdata[10] = baro.ALT
    logger.outdata[11] = rpy[0]
    logger.outdata[12] = rpy[1]
    logger.outdata[13] = rpy[2]
    logger.outdata[14] = gps_llh.speed
    logger.outdata[15] = g[0]
    logger.outdata[16] = g[1]
    logger.outdata[17] = g[2]
    logger.outdata[18] = controls[0]
    logger.outdata[19] = controls[1]
    if len(controls) > 2:
        logger.outdata[20] = controls[2]
        logger.outdata[21] = controls[3]
    logger.println()

    #sleep so we don't spontaneously explode
    time.sleep(0.01)
