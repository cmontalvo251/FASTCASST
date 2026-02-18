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
NUMOUTPUTS = 20  #Number of data outputs (20 for car and boat, 22 for airplane)
NUMPWM = 2 #Number of PWM signals (2 for car and boat, 4 for airplane)
VEHICLE = 'boat'  #Options are 'car', 'boat', or 'airplane'
################################################

##Import basic utilities
import numpy as np
import sys

##Import the vehicle controller based on your selection
sys.path.append('../libraries/V_'+VEHICLE)
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
#Calibrate the barometer
baro.calibrate() #if you don't calibrate sea level defaults to 1013.25

#Setup RCIO (receiver signals and output pwmsignals)
sys.path.append('../libraries/RCIO/Python')
import rcio
rc = rcio.RCIO(NUMPWM)

##Setup Telemetry
sys.path.append('../libraries/')
from Comms.Comms import Comms as U
ser = U()
ser.SerialInit(57600,"/dev/ttyAMA0",period=1.0)

#Short break to build suspense
print('Sleep for 1 second.....')
import time
time.sleep(1)

#Create a time for elapsed time
print('Setting up Time')
StartTime = time.time()
RunTime = 0.0
logTime = RunTime
telemetryTime = RunTime

#This runs on repeat until code is killed
print('Running main loop....')

while (True):

    #Get Time
    LastTime = RunTime
    RunTime = time.time() - StartTime
    elapsedTime = RunTime - LastTime
    
    #Read in receiver commands
    ARMED,safety_color = rc.rcin.readALL()

    #Get acceleration,gyroscope, magnetometer & temperature data
    #Note I do not recommend using rpy since that is solely using trigonometry
    #in addition the yaw angle does not work at all
    #rpy_ahrs works really well for obtaining the yaw angle
    a,gdegs,m,rpy,rpy_ahrs,temp = imu.getALL(elapsedTime) 

    #Get GPS update if it's ready
    gps_llh.poll(RunTime)

    #Get pressure which also converts to altitude (temperature is not currently working to speed up simulation)
    #If you really want temperature check the baro.poll() function and uncomment the temperature routine
    #Also keep in mind that the MPU9250 returns temperature automatically
    baro.poll(RunTime)

    #Run your control loop
    controls,defaults,control_color = vehicle.loop(RunTime,rc.rcin)

    #Check if we are armed or not
    if ARMED:
        led.setColor(control_color)
        pwm_commands = controls
    else:
        led.setColor(safety_color)
        pwm_commands = defaults

    ##Send PWM signals to rcio
    rc.set_commands(pwm_commands)

    #Print to Home
    str_pwm = [f"{pwm:1.3f}" for pwm in pwm_commands] #convert pwm commands to 3 sig figs
    str_rpy = [f"{ang:3.3f}" for ang in rpy_ahrs] #convert rpy to 3 sig figs
    str_g = [f"{gi:2.3f}" for gi in gdegs] #convert ang vel to 3 sig figs
    #print(f"{RunTime:4.4f}",f"{elapsedTime:1.4f}",gps_llh.latitude,gps_llh.longitude,gps_llh.altitude)
    print(f"{RunTime:4.4f}",f"{elapsedTime:1.4f}",rc.rcin.rcsignals,str_pwm,str_rpy,str_g,f"{baro.ALT:.3f}",gps_llh.altitude)

    ##Send Telemetry
    if (RunTime - telemetryTime) > 1.0:
        telemetryTime = RunTime	
        print('Sending telemtry packet...',RunTime)
        ser.fast_packet[0] = RunTime #//1 - Time
        ser.fast_packet[1] = rpy_ahrs[0] #//2 - roll
        ser.fast_packet[2] = rpy_ahrs[1] #//3 - pitch
        ser.fast_packet[3] = rpy_ahrs[2] #//4 - yaw (compass)
        ser.fast_packet[4] = gps_llh.latitude #//5 - latitude
        ser.fast_packet[5] = gps_llh.longitude #//6 - longitude
        ser.fast_packet[6] = baro.ALT #//7 - altitude (barometer)
        ser.fast_packet[7] = gps_llh.speed #//8 - speed (GPS)
        ser.SerialSend(0)
    #Log data
    if (RunTime - logTime) > 0.1:
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
        logger.outdata[10] = baro.PRES
        logger.outdata[11] = rpy_ahrs[0]
        logger.outdata[12] = rpy_ahrs[1]
        logger.outdata[13] = rpy_ahrs[2]
        logger.outdata[14] = gps_llh.speed #Does not work right now. Need to revisit gps.py to fix
        logger.outdata[15] = gdegs[0]
        logger.outdata[16] = gdegs[1]
        logger.outdata[17] = gdegs[2]
        logger.outdata[18] = pwm_commands[0]
        logger.outdata[19] = pwm_commands[1]
        if len(pwm_commands) > 2:
            logger.outdata[20] = pwm_commands[2]
            logger.outdata[21] = pwm_commands[3]
        logger.println()
        logTime = RunTime

    #sleep so we don't spontaneously explode
    #time.sleep(0.01) Since there are sleeps in the barometer you don't need this anymore.
    #Also all the different sensor updates and calculations take so much time that the system won't spontaneously
    #explode. However, if you start debugging and turning things off it easily could....
