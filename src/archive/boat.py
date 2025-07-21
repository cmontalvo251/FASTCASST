#############################################
#
#  FASTPy - Facility for Aerial Systems and Technology
#  Kit in Python for ROBOBOAT
#
#  SKID STEER BOAT
#  Created: Spring 2022
#  Primary Author: Julia Nelson
#  Secondary Author: Maxwell Cobar
#  TERTIARY AUTHOR: Carlos Montalvo
#
################################################


#Add in all needed libraries and modules
import sys, time

sys.path.append('../libraries/Util')
import util

sys.path.append('../libraries/GPS/')
import gps

sys.path.append('../libraries/IMU/')
import mpu9250

sys.path.append('../libraries/Datalogger')
import datalogger

sys.path.append('../libraries/LED')
import leds

sys.path.append('../libraries/RCIO/Python')
import rcinput
import pwm

import numpy as np
#Create a time for elapsed time
LastTime = time.time()
GPSTime = time.time()
StartTime = time.time()

#Make sure Ardupilot is off
util.check_apm()

#Setup datalogger
logger = datalogger.Datalogger()
if len(sys.argv) < 2:
    logger.findfile('./')
else:
    logger.findfile(sys.argv[1])
logger.open()
#create an array for data
outdata = np.zeros(11)

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

def SATURATE(pwm_in):
    #Add Saturation Block to ensure servo safety
    if(pwm_in < SERVO_MIN):
        pwm_in = SERVO_MIN
    if(pwm_in > SERVO_MAX):
        pwm_in = SERVO_MAX
    return pwm_in

#This runs on repeat until code is killed
while (True):
    ##GET TIME
    RunTime = time.time()

    ##GET ELAPSED GPS TIME
    elapsedGPSTime = RunTime - GPSTime

    ##GET LOOP ELAPSED TIME
    elapsedTime = RunTime - LastTime
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

    #Get GPS update
    if(elapsedTime > 1.0):
        GPSTime = time.time()
        gps_llh.update()

    #Get IMU
    a,g,rpy,temp = imu.getALL()
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]

    #Set arm switch up for safety reasons
    MOTOR1 = SERVO_MIN
    MOTOR2 = SERVO_MIN
    if(armswitch < 1.495):
        led.setColor('Red')
        #print('Disarmed for safety')
        MOTOR1 = SERVO_MIN
        MOTOR2 = SERVO_MIN
    elif(1.495 < armswitch < 1.995):
        led.setColor('Green')

        ##MIX THE CHANNELS
        throttle_left = throttlerc + 1.0*(rollrc - SERVO_MID)
        throttle_right = throttlerc - 1.0*(rollrc - SERVO_MID)

        MOTOR1 = SATURATE(throttle_left)
        MOTOR2 = SATURATE(throttle_right)

    elif(armswitch > 1.995):
        led.setColor('Blue')
        MOTOR1 = SERVO_MIN
        MOTOR2 = SERVO_MAX

    pwm1.set_duty_cycle(MOTOR1)
    pwm2.set_duty_cycle(MOTOR2)
    #print(armswitch,throttlerc,yawrc)

    ##Print Stuff
    if elapsedTime > 0.1:
        LastTime = RunTime
        print(RunTime-StartTime,throttlerc,rollrc,armswitch,gps_llh.latitude,gps_llh.longitude,MOTOR1,MOTOR2,roll,pitch,yaw)

        #Log data
        outdata[0] = RunTime-StartTime
        outdata[1] = throttlerc
        outdata[2] = rollrc
        outdata[3] = armswitch
        outdata[4] = gps_llh.latitude
        outdata[5] = gps_llh.longitude
        outdata[6] = MOTOR1
        outdata[7] = MOTOR2
        outdata[8] = roll
        outdata[9] = pitch
        outdata[10] = yaw
        #outdata[]
        logger.println(outdata)

    
