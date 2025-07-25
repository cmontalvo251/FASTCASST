
#############################################
#
#  FASTPy - Facility for Aerial Systems and Technology
#  Kit in Python
#
#  Front Wheel Drive Car Manual Control Software
#  Created: Spring 2022
#  Primary Author: Julia Nelson
#  Secondary Author: Maxwell Cobar
#  Tertiary Author : Aramis Hoffmann
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
outdata = np.zeros(10)

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
rcin = rcinput.RCInput(9)

#Setup LED
print('Setting up LEDs.....')
led = leds.Led()
led.setColor('Yellow')
print('LED is yellow now')

#Setup the Barometer
print('Setting up the barometer....')
baro = ms5611.MS5611()
baro.start()

#SETUP SERVOS
#Throttle - PWM Channel 1
print('Initializing PMW channels')
pwms = pwm.CHANNELS(2) ##Create 2 PWM objects

pwm1 = pwm.PWM(0)
pwm1.initialize()
pwm1.set_period(50)
pwm1.enable()
#Steering - PWM Channel 2
pwm2 = pwm.PWM(1)
pwm2.initialize()
pwm2.set_period(50)
pwm2.enable()

#Short break to build suspense
print('Sleep for 1 second')
time.sleep(1)

#Create a time for elapsed time
print('Setting up Time')
StartTime = time.time()

#This runs on repeat until code is killed
while (True):
    RunTime = time.time() - StartTime
    #print(elapsedTime)
    
    #Read in receiver commands
    for i in range(num_channels):
        value = rcin.read(i)
        period[i] = value
    #print period

    #Turn receiver commands to floats
    rollrc = float(period[0])/1000.
    pitchrc = float(period[1])/1000.
    throttlerc = float(period[2])/1000.
    yawrc = float(period[3])/1000.
    armswitch = float(period[4])/1000.
    autopilot = float(period[6])/1000.
    #print(throttlerc,rollrc,pitchrc,yawrc,armswitch)

    #Get GPS update
    gps_llh.poll(RunTime)

    #Poll barometer
    baro.poll(RunTime)

    #Compute the controller values
    throttle_command = throttlerc
    roll_command = rollrc

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
    print(np.round(RunTime,2), throttlerc, rollrc, autopilot,gps_llh.longitude,gps_llh.latitude,gps_llh.altitude,np.round(baro.pressure,2))

    #Log data
    outdata[0] = np.round(RunTime,5)
    outdata[1] = throttlerc
    outdata[2] = rollrc
    outdata[3] = autopilot
    outdata[4] = gps_llh.longitude
    outdata[5] = gps_llh.latitude
    outdata[6] = gps_llh.altitude
    outdata[7] = throttle_command
    outdata[8] = roll_command
    outdata[9] = np.round(baro.pressure,5)
    #outdata[]
    logger.println(outdata)
