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
NUMOUTPUTS = 13  #Number of data outputs
NUMRX = 9  #Receiver signals
NUMPWM = 2 #Number of PWM signals

##Pick the vehicle you want to use
import sys
sys.path.append('../libraries/V_car')
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
rc = rcio.RCIO(NUMRX,NUMPWM)

#Short break to build suspense
print('Sleep for 1 second.....')
import time
time.sleep(1)

#Create a time for elapsed time
print('Setting up Time')
StartTime = time.time()

#This runs on repeat until code is killed
print('Running main loop....')
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
    elif(rcin.armswitch > 0):
        led.setColor('Green')
        rc.set_commands(controls)

    #Print to Home
    print(np.round(RunTime,2))
    #print(np.round(RunTime,2), throttlerc, rollrc, autopilot,gps_llh.longitude,gps_llh.latitude, d, rpy[0], throttle_command,roll_command)
    #print(np.round(RunTime, 2), np.round(m[0], 2), np.round(m[1], 2), np.round(m[2], 2))

    #Log data
    logger.outdata[0] = np.round(RunTime,5)
    logger.outdata[1] = a[0]
    logger.outdata[2] = a[1]
    logger.outdata[3] = a[2]
    logger.outdata[4] = g[0]
    logger.outdata[5] = g[1]
    logger.outdata[6] = g[2]
    logger.outdata[7] = m[0]
    logger.outdata[8] = m[1]
    logger.outdata[9] = m[2]
    logger.outdata[10] = rpy[0]
    logger.outdata[11] = rpy[1]
    logger.outdata[12] = rpy[2]
    logger.println()

    #sleep so we don't spontaneously explode
    time.sleep(0.01)
