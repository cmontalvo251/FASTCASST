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

#####################PARAMETERS##########################
NUMOUTPUTS = 13  #Number of data outputs
NUMRX = 9  #Receiver signals
NUMPWM = 2 #Number of PWM signals

#Add in all needed libraries and modules
import sys, time
import numpy as np

sys.path.append('../libraries/Util')
import util

sys.path.append('../libraries/GPS')
import gps

sys.path.append('../libraries/MPU9250')
import mpu9250

sys.path.append('../libraries/Datalogger')
import datalogger

sys.path.append('../libraries/LED')
import leds

sys.path.append('../libraries/MS5611/')
import ms5611

sys.path.append('../libraries/RCIO/Python')
import rcio

#Make sure Ardupilot is off
util.check_apm()

#Setup datalogger
logger = datalogger.Datalogger(NUMOUTPUTS)

#Setup GPS
gps_llh = gps.GPS()

#Setup IMU
imu = mpu9250.MPU9250()

#Setup RCIO (receiver signals and output pwmsignals)
rc = rcio.RCIO(NUMRX,NUMPWM)

#Setup LED
led = leds.Led()

#Setup the Barometer
#print('Setting up the barometer....')
#BARONEXT = 1.0
#BAROWAIT = 0.01
#baro = ms5611.MS5611()
#baro.initialize()
#baro.refreshPressure()
#time.sleep(BAROWAIT)
#baro.readPressure()
#baro.calculatePressureAndTemperature()
#pressure = baro.PRES
#time.sleep(BARONEXT)
#BAROMODE = 0

#Waypoints
wp = np.array([[-88.1755, 30.6906], [-88.1750, 30.6902], [-88.1745, 30.6906]])
wp_index = 0 #Keeps track of which waypoint is being used
wp_index_max = len(wp)
R = 6371*10**3 #Radius of the Earth in m

#Short break to build suspense
print('Sleep for 1 second')
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

    #Barometer not needed
    #if BAROMODE == 2:
        ##first we grab prassure
        #pressure = baro.PRES
        ##in here we want to make sure we wait 1 second before we set
        ##baromode back to zero
        #if (RunTime - BAROTime) > BARONEXT:
            #BAROTime = RunTime
            #BAROMODE = 0
    #if BAROMODE == 1:
        ##If baromode is 1 we read and calculate but only after 0.01 seconds has passed
        #if (RunTime - BAROTime) > BAROWAIT:
            #baro.readPressure()
            #baro.calculatePressureAndTemperature()
            #BAROTime = RunTime
            ##and set the baromode to 2
            #BAROMODE = 2
    #if BAROMODE == 0:
        ##initially the mode is zero
        ##so we refresh the register
        #baro.refreshPressure()
        #BAROTime = RunTime
        ##then we set the mode to 1
        #BAROMODE = 1

    #Find the distance and angle from the car to the next waypoint
    d_long = gps_llh.longitude - wp[wp_index, 0]
    d_lat = gps_llh.latitude - wp[wp_index, 1]
    dx = d_long * np.pi/180 * R
    dy = d_lat * np.pi/180 * R
    d = np.sqrt(dx**2 + dy**2) #Distance from car to waypoint

    #Compute the controller values
    if (rc.rcin.autopilot < 2):
        throttle_command = rc.rcin.throttlerc
        roll_command = rc.rcin.rollrc
    else: #Remember to test this part of code!
        throttle_command = 0
        if wp_index >= wp_index_max:
            break
        if (d<=20): #If the distance is less than or equal to 20 m...
            pwm2.set_duty_cycle(SERVO_MAX) #Turns the front wheels left
            time.sleep(1) #Stops the car to signal it's reached its waypoint
            wp_index += 1
        else: #If the distance is greater than 20 m...
            roll_command = SERVO_MID #... keep the wheels straight

    ##Saturation blocks
    if(throttle_command < rc.SERVO_MIN):
        throttle_command = rc.SERVO_MIN
    if(throttle_command > rc.SERVO_MAX):
        throttle_command = rc.SERVO_MAX
    if(roll_command < rc.SERVO_MIN):
        roll_command = rc.SERVO_MIN
    if(roll_command > rc.SERVO_MAX):
        roll_command = rc.SERVO_MAX

    #Set arm switch up for safety reasons
    if(rc.rcin.armswitch < 1.495):
        led.setColor('Red')
        rc.pwms[0].set_duty_cycle(rc.SERVO_MIN)
        rc.pwms[1].set_duty_cycle(rc.SERVO_MID)
        #print('Disarmed for safety')
    elif(1.495 < rcin.armswitch < 1.995):
        led.setColor('Green')
        rc.pwms[0].set_duty_cycle(throttle_command)
        rc.pwms[1].set_duty_cycle(roll_command)
        #print('Open Loop Control')
    elif(rcin.armswitch > 1.995):
        led.setColor('Blue')
        rc.pwms[0].set_duty_cycle(rc.SERVO_MIN)
        rc.pwms[1].set_duty_cycle(rc.SERVO_MID)
        #print('Autonomous Control')
    #print(armswitch,throttlerc,rollrc)

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
