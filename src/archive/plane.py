
#############################################
#
#  FASTPy - Facility for Aerial Systems and Technology
#  Kit in Python
#
#  Standard 4 Channel Airplane (Derived from Car.py)
#  Created: Spring 2025
#  Primary Author: Kate Doiron
#
################################################ 


#Add in all needed libraries and modules
import sys, time

sys.path.append('/home/pi/FASTCASST/libraries/Util')
import util

sys.path.append('/home/pi/FASTCASST/libraries/GPS')
import gps

sys.path.append('/home/pi/FASTCASST/libraries/MPU9250')
import mpu9250

sys.path.append('/home/pi/FASTCASST/libraries/Datalogger')
import datalogger

sys.path.append('/home/pi/FASTCASST/libraries/LED')
import leds

sys.path.append('/home/pi/FASTCASST/libraries/MS5611/')
import ms5611

sys.path.append('/home/pi/FASTCASST/libraries/RCIO/Python')
import rcinput
import pwm

import numpy as np

#Make sure Ardupilot is off
print('Checking to make sure APM is off')
util.check_apm()

#Setup datalogger
logger = datalogger.Datalogger()
print(sys.argv)
logger.findfile(sys.argv[1])
logger.open()
#create an array for data
outdata = np.zeros(20)

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
print('Setting up the barometer....')
BARONEXT = 1.0
BAROWAIT = 0.01
baro = ms5611.MS5611()
baro.initialize()
baro.refreshPressure()
time.sleep(BAROWAIT)
baro.readPressure()
baro.calculatePressureAndTemperature()
pressure = baro.PRES
time.sleep(BARONEXT)
BAROMODE = 0

#Setup Servos
SERVO_MIN = 0.995 #ms
SERVO_MID = 1.504 #ms
SERVO_MAX = 2.010 #ms
PWM_OUTPUT = [0,1,2,3] #Servo Rail Spots
print('PWM Channels: ',PWM_OUTPUT)

#Throttle - PWM Channel 1
print('Initializing PMW channels')
pwm1 = pwm.PWM(PWM_OUTPUT[0])
pwm1.initialize()
pwm1.set_period(50)
pwm1.enable()
#Roll (Aileron) - PWM Channel 2
pwm2 = pwm.PWM(PWM_OUTPUT[1])
pwm2.initialize()
pwm2.set_period(50)
pwm2.enable()
#Pitch (Elevator) - PWM Channel 3
pwm3 = pwm.PWM(PWM_OUTPUT[2])
pwm3.initialize()
pwm3.set_period(50)
pwm3.enable()
#Yaw (Rudder) - PWM Channel 4
pwm4 = pwm.PWM(PWM_OUTPUT[3])
pwm4.initialize()
pwm4.set_period(50)
pwm4.enable()

#Short break to build suspense
print('Sleep for 1 second')
time.sleep(1)

#Create a time for elapsed time
print('Setting up Time')
StartTime = time.time()
GPSTime = 0
GPScount = 0
BAROTime = time.time()-StartTime
printRATE = 0.2
printNEXT = 0.0

#This runs on repeat until code is killed
while (True):
    RunTime = time.time() - StartTime
    
    #Read in receiver commands
    period = []
    for i in range(num_channels):
        value = rcin.read(i)
        period.append(value)
    #print period

    #Get IMU
    a,g,rpy,temp = imu.getALL()
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]
    roll_rate = -g[1]
    pitch_rate = -g[0]
    yaw_rate = -g[2]

    #Turn receiver commands to floats
    throttlerc = float(period[0])/1000.
    rollrc = float(period[1])/1000.
    pitchrc = float(period[2])/1000.
    yawrc = float(period[3])/1000.
    armswitch = float(period[4])/1000.  ##CHANGE ARMSWITCH TO BE AUTOPILOT AND JUST HAVE PLANE ARMED AT ALL TIMES
    autopilot = float(period[5])/1000. ##This is not working on the current transmitter
    #print(throttlerc,rollrc,pitchrc,yawrc,armswitch)

    #Get GPS update (Commented out gps and barometer for now while debugging servos
    """
    if(RunTime > 1.0):
        GPSTime = time.time()
        gps_llh.update()
        #print(gps_llh.longitude,gps_llh.latitude,gps_llh.altitude)
    """
    
    if BAROMODE == 2:
        #first we grab prassure
        pressure = baro.PRES
        #in here we want to make sure we wait 1 second before we set
        #baromode back to zero
        if (RunTime - BAROTime) > BARONEXT:
            BAROTime = RunTime
            BAROMODE = 0
    if BAROMODE == 1:
        #If baromode is 1 we read and calculate but only after 0.01 seconds has passed
        if (RunTime - BAROTime) > BAROWAIT:
            baro.readPressure()
            baro.calculatePressureAndTemperature()
            BAROTime = RunTime
            #and set the baromode to 2
            BAROMODE = 2
    if BAROMODE == 0:
        #initially the mode is zero
        #so we refresh the register
        baro.refreshPressure()
        BAROTime = RunTime
        #then we set the mode to 1
        BAROMODE = 1

    #Compute the controller values

    #Set arm switch up for safety reasons
    if(armswitch < 1.495):
        led.setColor('Red')
        throttle_command = SERVO_MIN
        roll_command = SERVO_MID
        pitch_command = SERVO_MID
        yaw_command = SERVO_MID
        #print('Disarmed for safety')
    elif(1.495 < armswitch < 1.995):
        led.setColor('Green')
        throttle_command = throttlerc
        roll_command = rollrc
        pitch_command = SERVO_MID-(pitchrc-SERVO_MID) #switch pitch and yaw
        yaw_command = SERVO_MID-(yawrc-SERVO_MID) 
        #print('Open Loop Control')
    elif(armswitch > 1.995):
        ##PROGRAM PD CONTROLLER
        led.setColor('Blue')
	#throttle_command unchanged
        throttle_command = throttlerc
        kp = 0.01
        kd = 0.001
        kr = 1.0
        roll_error = kp*(roll - 0) + kd*(roll_rate - 0)
        roll_command = SERVO_MID - roll_error
        pitch_command = SERVO_MID - kp*(pitch - 0) - kd*(pitch_rate - 0)
        yaw_command = SERVO_MID + kr*roll_error
        #print('Autonomous Control')

    #print(armswitch,throttlerc,yawrc)

    ##Saturation blocks
    if(throttle_command < SERVO_MIN):
        throttle_command = SERVO_MIN
    if(throttle_command > SERVO_MAX):
        throttle_command = SERVO_MAX

    if(roll_command < SERVO_MIN):
        roll_command = SERVO_MIN
    if(roll_command > SERVO_MAX):
        roll_command = SERVO_MAX

    if(pitch_command < SERVO_MIN):
        pitch_command = SERVO_MIN
    if(pitch_command > SERVO_MAX):
        pitch_command = SERVO_MAX

    if(yaw_command < SERVO_MIN):
        yaw_command = SERVO_MIN
    if(yaw_command > SERVO_MAX):
        yaw_command = SERVO_MAX

    pwm1.set_duty_cycle(throttle_command)
    pwm2.set_duty_cycle(roll_command)
    pwm3.set_duty_cycle(pitch_command)
    pwm4.set_duty_cycle(yaw_command)

    #Print to Home
    if RunTime > printNEXT:
        print(np.round(RunTime,2), throttlerc, rollrc, pitchrc, yawrc, armswitch,autopilot,gps_llh.longitude,gps_llh.latitude,gps_llh.altitude,np.round(pressure,2),np.round(roll,2),np.round(pitch,2),np.round(yaw,2),np.round(roll_rate,2),np.round(pitch_rate,2),np.round(yaw_rate,2))
        printNEXT += printRATE

    #Log data
    outdata[0] = np.round(RunTime,5)
    outdata[1] = throttlerc
    outdata[2] = rollrc
    outdata[3] = pitchrc
    outdata[4] = yawrc
    outdata[5] = autopilot
    outdata[6] = gps_llh.longitude
    outdata[7] = gps_llh.latitude
    outdata[8] = gps_llh.altitude
    outdata[9] = throttle_command
    outdata[10] = roll_command
    outdata[11] = pitch_command
    outdata[12] = yaw_command
    outdata[13] = pressure
    outdata[14] = roll
    outdata[15] = pitch
    outdata[16] = yaw
    outdata[17] = roll_rate
    outdata[18] = pitch_rate
    outdata[19] = yaw_rate
    #outdata[]
    logger.println(outdata)

    time.sleep(0.01)

