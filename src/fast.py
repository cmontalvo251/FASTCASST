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


#Add in all needed libraries and modules
import sys, time

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
import rcinput
import pwm

import numpy as np

#Make sure Ardupilot is off
print('Checking to make sure APM is off')
util.check_apm()

#Setup datalogger
logger = datalogger.Datalogger()
#Get Directoy
print('Input arguments = ',sys.argv)
if len(sys.argv) > 1:
    print('Using Directory = ',sys.argv[1])
else:
    sys.exit('No input argument given for datalogging directory')
logger.findfile(sys.argv[1])
logger.open()
#create an array for data
outdata = np.zeros(13)

#Setup GPS
print('Initializing GPS...')
gps_llh = gps.GPS()
gps_llh.initialize()

#Setup IMU
print('Initializing IMU...')
imu = mpu9250.MPU9250()
imu.initialize()

#Setup RCIO
print('Initializing RCInput...')
rcin = rcinput.RCInput()
i = 0
num_channels = 9

#Setup LED
print('Setting up LEDs.....')
led = leds.Led()
led.setColor('Yellow')
print('LED is yellow now')

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

#Setup Servos
SERVO_MIN = 0.995 #ms
SERVO_MID = 1.504 #ms
SERVO_MAX = 2.010 #ms
PWM_OUTPUT = [0,1] #Servo Rail Spots
print('PWM Channels: ',PWM_OUTPUT)

#Throttle - PWM Channel 1
print('Initializing PMW channels')
pwm1 = pwm.PWM(PWM_OUTPUT[0])
pwm1.initialize()
pwm1.set_period(50)
pwm1.enable()
#Steering - PWM Channel 2
pwm2 = pwm.PWM(PWM_OUTPUT[1])
pwm2.initialize()
pwm2.set_period(50)
pwm2.enable()

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
BAROTime = time.time()-StartTime
GPScount = 0 #Counts how many time the GPS has been read per 0.25 sec
GPStime = 0 #Increments every 0.25 sec

#This runs on repeat until code is killed
while (True):
    RunTime = time.time() - StartTime
    
    #Read in receiver commands
    period = []
    for i in range(num_channels):
        value = rcin.read(i)
        period.append(value)
    #print period

    #Turn receiver commands to floats
    rollrc = float(period[0])/1000.
    pitchrc = float(period[1])/1000.
    throttlerc = float(period[2])/1000.
    yawrc = float(period[3])/1000.
    armswitch = float(period[4])/1000.
    autopilot = float(period[6])/1000.
    #print(throttlerc,rollrc,pitchrc,yawrc,armswitch)

    #Get GPS update every 0.25 s
    if GPStime <= RunTime:
        GPScount = 0
        GPStime += 0.25
    if(RunTime > 1.0) & (GPScount == 0):
        gps_llh.update()
        GPScount = 1

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
    if (autopilot < 2):
        throttle_command = throttlerc
        roll_command = rollrc
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
    if(throttle_command < SERVO_MIN):
        throttle_command = SERVO_MIN
    if(throttle_command > SERVO_MAX):
        throttle_command = SERVO_MAX
    if(roll_command < SERVO_MIN):
        roll_command = SERVO_MIN
    if(roll_command > SERVO_MAX):
        roll_command = SERVO_MAX

    #Set arm switch up for safety reasons
    if(armswitch < 1.495):
        led.setColor('Red')
        pwm1.set_duty_cycle(SERVO_MIN)
        pwm2.set_duty_cycle(SERVO_MID)
        #print('Disarmed for safety')
    elif(1.495 < armswitch < 1.995):
        led.setColor('Green')
        pwm1.set_duty_cycle(throttle_command)
        pwm2.set_duty_cycle(roll_command)
        #print('Open Loop Control')
    elif(armswitch > 1.995):
        led.setColor('Blue')
        pwm1.set_duty_cycle(SERVO_MIN)
        pwm2.set_duty_cycle(SERVO_MID)
        #print('Autonomous Control')
    #print(armswitch,throttlerc,rollrc)

    #Print to Home
    #print(np.round(RunTime,2), throttlerc, rollrc, autopilot,gps_llh.longitude,gps_llh.latitude, d, phi, throttle_command,roll_command)
    #Get acceleration, gyroscope, magnetometer & temperature data
    a,g,m,rpy,temp = imu.getALL()
    print(np.round(RunTime, 2), np.round(m[0], 2), np.round(m[1], 2), np.round(m[2], 2))

    #Log data
    outdata[0] = np.round(RunTime,5)
    outdata[1] = a[0]
    outdata[2] = a[1]
    outdata[3] = a[2]
    outdata[4] = g[0]
    outdata[5] = g[1]
    outdata[6] = g[2]
    outdata[7] = m[0]
    outdata[8] = m[1]
    outdata[9] = m[2]
    outdata[10] = rpy[0]
    outdata[11] = rpy[1]
    outdata[12] = rpy[2]
    #outdata[]
    logger.println(outdata)
    time.sleep(0.01)
