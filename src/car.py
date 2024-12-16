
#############################################
#
#  FASTPy - Facility for Aerial Systems and Technology
#  Kit in Python
#
#  Front Wheel Drive Car Manual Control Software
#  Created: Spring 2022
#  Primary Author: Julia Nelson
#  Secondary Author: Maxwell Cobar
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

sys.path.append('/home/pi/FASTCASST/libraries/RCIO/Python')
import rcinput
import pwm

import numpy as np
#Create a time for elapsed time
StartTime = time.time()

#Make sure Ardupilot is off
util.check_apm()

#Setup datalogger
logger = datalogger.Datalogger()
print(sys.argv)
logger.findfile(sys.argv[1])
logger.open()
#create an array for data
#arm,throttlerc,yawrc,lat,lon,alt,velocity,roll,pitch,heading
outdata = np.zeros(10)

#Setup GPS
gps_llh = gps.GPS()
gps_llh.initialize()

#Setup IMU
imu = mpu9250.MPU9250()
imu.initialize()

#Setup RCIO
rcin = rcinput.RCInput()
i = 0
num_channels = 9

#Setup LED
led = leds.Led()
led.setColor('Yellow')

#Setup Servos
SERVO_MIN = 0.995 #ms
SERVO_MID = 1.504 #ms
SERVO_MAX = 2.010 #ms
PWM_OUTPUT = [0,1] #Servo Rail Spots
print('PWM Channels: ',PWM_OUTPUT)

#Throttle - PWM Channel 1
pwm1 = pwm.PWM(PWM_OUTPUT[0])
pwm1.initialize()
pwm1.set_period(50)
pwm1.enable()
#Steering - PWM Channel 2
pwm2 = pwm.PWM(PWM_OUTPUT[1])
pwm2.initialize()
pwm2.set_period(50)
pwm2.enable()

#Short break to build suspense
time.sleep(1)

#This runs on repeat until code is killed
while (True):
    RunTime = time.time()
    elapsedTime = RunTime - StartTime
    #print(elapsedTime)
    
    #Read in receiver commands
    period = []
    for i in range(num_channels):
        value = rcin.read(i)
        period.append(value)
    #print period

    #Turn receiver commands to floats
    throttlerc = float(period[0])/1000.
    rollrc = float(period[1])/1000.
    pitchrc = float(period[2])/1000.
    yawrc = float(period[3])/1000.
    armswitch = float(period[4])/1000.
    #print(throttlerc,rollrc,pitchrc,yawrc,armswitch)

    #Get GPS update
    if(elapsedTime > 1.0):
        StartTime = time.time()
        gps_llh.update()
        #print(gps_llh.longitude,gps_llh.latitude,gps_llh.altitude)

    #Set arm switch up for safety reasons
    if(armswitch < 1.495):
        led.setColor('Red')
        pwm1.set_duty_cycle(SERVO_MIN)
        pwm2.set_duty_cycle(SERVO_MID)
        #print('Disarmed for safety')
    elif(1.495 < armswitch < 1.995):
        led.setColor('Green')
        #Add Saturation Block to ensure servo safety
        if(throttlerc < SERVO_MIN):
            throttlerc = SERVO_MIN
        if(throttlerc > SERVO_MAX):
            throttlerc = SERVO_MAX
        #Send throttlerc to servo
        pwm1.set_duty_cycle(throttlerc)
        #Add Saturation Block to ensure servo safety
        if(yawrc < SERVO_MIN):
            yawrc = SERVO_MIN
        if(yawrc > SERVO_MAX):
            yawrc = SERVO_MAX
        #Send yawrc to servo
        pwm2.set_duty_cycle(yawrc)
        #print('Open Loop Control')
    elif(armswitch > 1.995):
        led.setColor('Blue')
        pwm1.set_duty_cycle(SERVO_MIN)
        pwm2.set_duty_cycle(SERVO_MID)
        #print('Autonomous Control')
    #print(armswitch,throttlerc,yawrc)

    #Log data
    outdata[0] = armswitch
    outdata[1] = throttlerc
    outdata[2] = yawrc
    #outdata[]
    logger.println(outdata)
