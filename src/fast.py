
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
#  Quaternary Author: Carlos Montalvo Fall 2025/Spring 2026/Fall 2026
#  Quinary Author: Vinicius da Luz (mostly an undisclosed AI tool) Summer 2026 (car waypoint navigation)
#
################################################

#####################PARAMETERS#################
VEHICLE = 'car'  #Options are 'car', 'boat', or 'airplane'
TELEMETRYTIME = 1.0 #time between telemetry sends in seconds
LOGTIME = 0.1 #time between logging time in seconds
MODE = 'SIMONLY' #options are 'SIMONLY', 'SIL' 'HIL' and 'AUTO'
TIMESTEP = 0.01 #Timestep of modeling if SIMONLY selected
TFINAL = 5.0 #final time of simulation if SIMONLY selected
#Initial Conditions for SIMONLY
ICs = [0,0,0,0,0,0,0,0,0,0,0,0] #x (m),y (m),z (m),phi (deg),theta (deg),psi (deg),u (m/s),v (m/s),w (m/s),p (deg/s),q (deg/s), r (deg/s)
LATITUDE_ORIGIN = 30.69 #Set origin for SIMONLY / SIL / HIL
LONGITUDE_ORIGIN = -88.16 #set origin for SIMONLY / SIL / HIL
################################################

##Import basic utilities
import numpy as np
import time
import sys
import os
sys.path.append('../libraries/')
sys.path.append('libraries/')

#Make sure Ardupilot is off
import Util.util as U
U.check_apm()

##Import the vehicle controller based on your selection
sys.path.append('../libraries/V_'+VEHICLE)
sys.path.append('libraries/V_'+VEHICLE)
import controller
vehicle = controller.CONTROLLER()
#Initialize commands
commands = vehicle.defaults
NUMOUTPUTS = 36+vehicle.NUMCONTROLS

##Import modeling if MODE == SIMONLY
if MODE == 'SIMONLY':
    import modeling.modeling as M
    model = M.MODEL(TIMESTEP,ICs,VEHICLE,NUMOUTPUTS,LATITUDE_ORIGIN,LONGITUDE_ORIGIN)

#Setup GPS
import GPS.gps as G
gps_llh = G.GPS(mode=MODE)
gps_llh.setOrigin(LATITUDE_ORIGIN,LONGITUDE_ORIGIN)

#Setup IMU
import MPU9250.mpu9250 as MPU
imu = MPU.MPU9250(mode=MODE)

#Setup datalogger
import Datalogger.datalogger as D
print('Input arguments = ',sys.argv)
if len(sys.argv) > 1:
    print('Using Directory = ',sys.argv[1])
else:
	sys.exit('No input argument given for datalogging directory')
logger = D.Datalogger(sys.argv[1],NUMOUTPUTS)
headers = 'Time (sec) ,Sense X(m) ,Sense Y(m) ,Sense Z(m) ,Sense Roll (deg) ,Sense Pitch (deg) ,Sense Compass (deg) ,Sense U(m/s) ,Sense V(m/s) ,Sense W(m/s) ,Sense P(rad/s) ,Sense Q(rad/s) ,Sense R(rad/s) ,Sense Mx(Gauss) ,Sense My(Gauss) ,Sense Mz(Gauss) ,Sense GPS Latitude (deg) ,Sense GPS Longitude (deg) ,Sense GPS Altitude (m) ,Sense GPS Heading (deg) ,Sense IMU Heading (deg) ,Sense Analog 1 (V) ,Sense Analog 2 (V) ,Sense Analog 3 (V) ,Sense Analog 4 (V) ,Sense Analog 5 (V) ,Sense Analog 6 (V) ,Sense Pressure (Pa) ,Sense Pressure Altitude (m) ,Sense Temperature (C) ,RC Channel #1 (ms) ,RC Channel #2 (ms) ,RC Channel #3 (ms) ,RC Channel #4 (ms),RC Channel #5 (ms), RC Channel #6 (ms)'
logger.writeheader(headers,'Hardware')

#Setup LED
import LED.leds as L
led = L.Led(mode=MODE)

#Setup RCIO (receiver signals and output pwmsignals)
import RCIO.Python.rcio as R
rc = R.RCIO(vehicle.NUMCONTROLS,MODE)
#This creates the pwm_commands vector and sends default values to the pwm channels
rc.set_commands(commands)

#Setup the Barometer
import MS5611.ms5611 as MS
baro = MS.MS5611(mode=MODE)
#Calibrate the barometer but only if you're not in SIMONLY mode
if MODE != 'SIMONLY':
    baro.calibrate() #if you don't calibrate sea level defaults to 1013.25

##Setup Telemetry
from Comms.Comms import Comms as S
ser = S(13) #otherwise this defaults to 12
ser.SerialInit(57600,"/dev/ttyAMA0",period=1.0)

#Short break to build suspense
if MODE != 'SIMONLY':
    print('Sleep for 1 second.....')
    time.sleep(1)
    TFINAL = 1e20 #Make the end time absurdly long that we would never hit in our lifetime

#Create a time for elapsed time
print('Setting up Time')
StartTime = time.time()
RunTime = 0.0
logTime = -LOGTIME
telemetryTime = 0.0

#This runs on repeat until code is killed
print('Running main loop....')

while (RunTime <= TFINAL):

    #Get Time
    LastTime = RunTime
    if MODE == 'SIMONLY':
        #Send model states to sensors
        gps_llh.send(model.state,model.statedot,VEHICLE)
        imu.send(model.state) #Just send the entire state vector
        baro.send(model.state) #again send the entire state vector
        #Integrate one timestep
        RunTime = LastTime + model.timestep
        model.loop(RunTime,rc.rcin.rcsignals,commands)
    else:
        RunTime = time.time() - StartTime
    elapsedTime = RunTime - LastTime
    
    #Read in receiver commands
    ARMED,safety_color = rc.rcin.readALL()

    #Get GPS update if it's ready
    gps_llh.poll(RunTime) 

    #Get acceleration,gyroscope, magnetometer & temperature data
    #Note I do not recommend using rpy since that is solely using trigonometry
    #in addition the yaw angle does not work at all
    #rpy_ahrs works really well for obtaining the yaw angle
    #compass is the ahrs magnetometer heading + the gps heading
    a,gdegs,m,rpy,rpy_ahrs,temp,compass = imu.getALL(elapsedTime,gps_llh.heading) 

    #Get pressure which also converts to altitude (temperature is not currently working to speed up simulation)
    #If you really want temperature check the baro.poll() function and uncomment the temperature routine
    #Also keep in mind that the MPU9250 returns temperature automatically
    baro.poll(RunTime)

    #Run your control loop
    controls,defaults,control_color = vehicle.loop(RunTime,rc.rcin,gps_llh,rpy_ahrs,gdegs,baro)

    #Check if we are armed or not
    if ARMED:
        led.setColor(control_color)
        commands = controls
    else:
        led.setColor(safety_color)
        commands = defaults

    ##Send PWM signals to rcio
    rc.set_commands(commands)

    #Print to Home
    str_pwm = [f"{pwm:1.3f}" for pwm in rc.pwm_commands] #convert pwm commands to 3 sig figs
    str_rpy = [f"{ang:3.3f}" for ang in rpy_ahrs] #convert rpy to 3 sig figs
    str_g = [f"{gi:2.3f}" for gi in gdegs] #convert ang vel to 3 sig figs
    #print(f"{RunTime:4.4f}",f"{elapsedTime:1.4f}",gps_llh.latitude,gps_llh.longitude,gps_llh.altitude)
    print(f"{RunTime:4.4f}",f"{elapsedTime:1.4f}",rc.rcin.rcsignals,str_pwm,str_rpy,str_g,f"{baro.ALT:.3f}",gps_llh.altitude)

    ##Send Telemetry
    if (RunTime - telemetryTime) >= TELEMETRYTIME and MODE != 'SIMONLY':
        telemetryTime += TELEMETRYTIME	
        print('Sending telemtry packet...',RunTime)
        ser.fast_packet[0] = RunTime #//1 - Time
        ser.fast_packet[1] = rpy_ahrs[0] #//2 - roll
        ser.fast_packet[2] = rpy_ahrs[1] #//3 - pitch
        ser.fast_packet[3] = rpy_ahrs[2] #//4 - yaw 
        ser.fast_packet[4] = gps_llh.latitude #//5 - latitude
        ser.fast_packet[5] = gps_llh.longitude #//6 - longitude
        ser.fast_packet[6] = baro.ALT #//7 - altitude (barometer)
        ser.fast_packet[7] = gps_llh.speed #//8 - speed (GPS)
        ser.fast_packet[8] = gps_llh.altitude #//9 - altitude (GPS)
        ser.fast_packet[9] = rc.rcin.throttle #//10 - throttle
        ser.fast_packet[10] = rc.rcin.roll #//11 - aileron
        ser.fast_packet[11] = rc.rcin.pitch #//12 - elevator
        ser.fast_packet[12] = rc.rcin.yaw #//13 - rudder
        ser.SerialSend(0)
    #Log data
    if (RunTime - logTime) >= LOGTIME:
        #Time (sec)
        logger.outdata[0] = np.round(RunTime,5)
        #X(m) Y(m) Z(m)
        logger.outdata[1] = gps_llh.X
        logger.outdata[2] = gps_llh.Y
        if VEHICLE == 'satellite':
            Z = gps_llh.Z
        else:
            Z = -baro.ALT
        logger.outdata[3] = Z
        #Roll (deg) ,Pitch (deg) , Compass (deg)
        logger.outdata[4] = rpy_ahrs[0]
        logger.outdata[5] = rpy_ahrs[1]
        if compass == -999:
            yaw = rpy_ahrs[2]
        else:
            yaw = compass
        logger.outdata[6] = yaw
        #U(m/s) ,V(m/s) ,W(m/s)
        if gps_llh.speed == -99:
            speed = 0.0
        else:
            speed = gps_llh.speed
        logger.outdata[7] = speed 
        logger.outdata[8] = 0
        logger.outdata[9] = 0
        #P(deg/s) ,Q(deg/s) ,R(deg/s)
        logger.outdata[10] = gdegs[0]
        logger.outdata[11] = gdegs[1]
        logger.outdata[12] = gdegs[2]
        #Mx(Gauss) ,My(Gauss) ,Mz(Gauss)
        logger.outdata[13] = m[0]
        logger.outdata[14] = m[1]
        logger.outdata[15] = m[2]
        #GPS Latitude (deg) ,GPS Longitude (deg) ,GPS Altitude (m)
        logger.outdata[16] = gps_llh.latitude
        logger.outdata[17] = gps_llh.longitude
        logger.outdata[18] = gps_llh.altitude
        #GPS Heading (deg) ,IMU Heading (deg)
        if gps_llh.heading == -999:
            heading = 0.0
        else:
            heading = gps_llh.heading
        logger.outdata[19] = heading
        logger.outdata[20] = rpy_ahrs[2]
        #Analog 1-6 (V)
        logger.outdata[21] = 0
        logger.outdata[22] = 0
        logger.outdata[23] = 0
        logger.outdata[24] = 0
        logger.outdata[25] = 0
        logger.outdata[26] = 0
        #Pressure (Pa) #Pressure Altitude (m) #Temperature (C)
        logger.outdata[27] = baro.PRES
        logger.outdata[28] = baro.ALT
        logger.outdata[29] = baro.TEMP
        #RC Channel #1-5
        logger.outdata[30] = rc.rcin.rcsignals[0]
        logger.outdata[31] = rc.rcin.rcsignals[1]
        logger.outdata[32] = rc.rcin.rcsignals[2]
        logger.outdata[33] = rc.rcin.rcsignals[3]
        logger.outdata[34] = rc.rcin.rcsignals[4]
        logger.outdata[35] = rc.rcin.rcsignals[5]
        #PWM Hardware Out 1-len(pwm_commands)
        for i in range(0,len(commands)):
            logger.outdata[36+i] = commands[i]
        logger.println()
        logTime += LOGTIME
        if MODE == 'SIMONLY':
            model.log(RunTime)

    #sleep so we don't spontaneously explode
    #time.sleep(0.01) Since there are sleeps in the barometer you don't need this anymore.
    #Also all the different sensor updates and calculations take so much time that the system won't spontaneously
    #explode. However, if you start debugging and turning things off it easily could....

#If the program ends it means we're running in modeling mode
#we need to copy a file
logger.close()
model.logger.close()
command = 'cp ' + str(logger.filename) + ' ' + str(logger.directory) + '0.csv'
os.system(command)
command = 'cp ' + str(model.logger.filename) + ' ' + str(model.logger.directory) + '0.csv'
os.system(command)
