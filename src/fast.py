
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

##Import basic utilities
import numpy as np
import time
import math
import sys
import os
sys.path.append('../libraries/')
sys.path.append('libraries/')

if len(sys.argv) > 2:
    MODE = sys.argv[1]
    VEHICLE = sys.argv[2]
else:
    print('You must run python3 src/fast.py MODE MODEL')
    print('Where MODE = SIMONLY, SIL, HIL or AUTO')
    print('MODEL = x8, airplane, boat or car')
    sys.exit()
#Check for JSON FILE NAME
if MODE == 'SIL':
    if len(sys.argv) > 3:
        USB_NAME = sys.argv[3]
        JSON_FILE_NAME = USB_NAME + '_config.json'
    else:
        print('If running in SIL mode you also need the USB Controller name')
        sys.exit()
else:
    JSON_FILE_NAME = '' #empty because it isn't needed

#Read the Simulation and Config Input Files
sim_file_name = 'libraries/V_' + VEHICLE + '/Input_Files/Simulation.txt'
try:
    lines = []
    with open(sim_file_name, "r") as f:
        for line in f:
            part = line.split("!")[0].strip()
            if part:
                lines.append(part)
except:
    print("Sim File Not Found:",sim_file_name)
TFINAL = float(lines[0])
TIMESTEP = float(lines[1])
ICs = np.zeros(13)
for i in range(0,13):
    ICs[i] = float(lines[2+i])
config_file_name = 'libraries/V_' + VEHICLE + '/Input_Files/Config.txt'
try:
    lines = []
    with open(config_file_name, "r") as f:
        for line in f:
            part = line.split("!")[0].strip()
            if part:
                lines.append(part)
except:
    print("Config File Not Found:",config_file_name)
CONTROLMODE = float(lines[10])
TELEMETRYTIME = float(lines[3])
PRINTTIME = float(lines[0])
LOGTIME = float(lines[1])
NUMWAYPOINTS = int(lines[11])
WAYPOINTSX = []
WAYPOINTSY = []
for i in range(0,NUMWAYPOINTS):
    WAYPOINTSX.append(float(lines[12+2*i]))
    WAYPOINTSY.append(float(lines[12+2*i+1]))
################################################

#Make sure Ardupilot is off
import Util.util as U
U.check_apm()

#Setup GPS
import GPS.gps as G
gps_llh = G.GPS(mode=MODE)
WAYPOINTSLAT = []
WAYPOINTSLON = []
if "WAYPOINTS" not in locals():
    #This means that WAYPOINTSXY has been set rather than WAYPOINTS in GPS coordinates
    for i in range(0,len(WAYPOINTSX)):
        lat,lon,alt = gps_llh.convertXYZ2LATLON(WAYPOINTSX[i],WAYPOINTSY[i],0)
        WAYPOINTSLAT.append(lat)
        WAYPOINTSLON.append(lon)
    WAYPOINTS = [WAYPOINTSLAT,WAYPOINTSLON]

##Check to see if system is always armed and user has no control
if CONTROLMODE >= 0 or MODE == 'SIMONLY':
    ALWAYSARMED = True
else:
    ALWAYSARMED = False

##Import the vehicle controller based on your selection
sys.path.append('../libraries/V_'+VEHICLE)
sys.path.append('libraries/V_'+VEHICLE)
import controller
vehicle = controller.CONTROLLER(WAYPOINTS,abs(CONTROLMODE))
#Initialize commands
commands = vehicle.defaults
NUMOUTPUTS = 36+vehicle.NUMCONTROLS

##Import modeling if MODE == SIMONLY
if MODE == 'SIMONLY' or MODE == 'SIL':
    import modeling.modeling as M
    model = M.MODEL(TIMESTEP,ICs,VEHICLE,NUMOUTPUTS,gps_llh.latO,gps_llh.lonO)

#Setup IMU
import MPU9250.mpu9250 as MPU
imu = MPU.MPU9250(mode=MODE)

#Setup datalogger
import Datalogger.datalogger as D
logger = D.Datalogger('data/',NUMOUTPUTS)
headers = 'Time (sec) ,Sense X(m) ,Sense Y(m) ,Sense Z(m) ,Sense Roll (deg) ,Sense Pitch (deg) ,Sense Compass (deg) ,Sense U(m/s) ,Sense V(m/s) ,Sense W(m/s) ,Sense P(deg/s) ,Sense Q(deg/s) ,Sense R(deg/s) ,Sense Mx(Gauss) ,Sense My(Gauss) ,Sense Mz(Gauss) ,Sense GPS Latitude (deg) ,Sense GPS Longitude (deg) ,Sense GPS Altitude (m) ,Sense GPS Heading (deg) ,Sense IMU Heading (deg) ,Sense Analog 1 (V) ,Sense Analog 2 (V) ,Sense Analog 3 (V) ,Sense Analog 4 (V) ,Sense Analog 5 (V) ,Sense Analog 6 (V) ,Sense Pressure (Pa) ,Sense Pressure Altitude (m) ,Sense Temperature (C) ,Sense RC Channel #1 (ms) ,Sense RC Channel #2 (ms) ,Sense RC Channel #3 (ms) ,Sense RC Channel #4 (ms) ,Sense RC Channel #5 (ms) ,Sense RC Channel #6 (ms)'
logger.writeheader(headers,'Hardware')

#Setup LED
import LED.leds as L
led = L.Led(mode=MODE)

#Setup RCIO (receiver signals and output pwmsignals)
import RCIO.Python.rcio as R
rc = R.RCIO(vehicle.NUMCONTROLS,MODE,JSON_FILE_NAME) #This is the number of PWM channels you want to output to
#This creates the pwm_commands vector and sends default values to the pwm channels
rc.set_commands(commands)
#Need a check to see if we read the joystick properly
if MODE == 'SIL' and rc.rcin.joy_fd == -1:
    ALWAYSARMED = True

#Setup the Barometer
import MS5611.ms5611 as MS
baro = MS.MS5611(mode=MODE)
#Calibrate the barometer but only if you're not in SIMONLY mode
if MODE != 'SIMONLY' and MODE != 'SIL':
    baro.calibrate() #if you don't calibrate sea level defaults to 1013.25

##Setup Telemetry
from Comms.Comms import Comms as S
ser = S(13) #otherwise this defaults to 12
ser.SerialInit(57600,"/dev/ttyAMA0",period=1.0)

#Short break to build suspense
if MODE != 'SIMONLY':
    TFINAL = 1e20 #Make the end time absurdly long that we would never hit in our lifetime

#Create a time for elapsed time
print('Setting up Time')
StartTime = time.time()
RunTime = 0.0
logTime = -LOGTIME
printTime = -PRINTTIME
telemetryTime = 0.0

##Kick off all the PyOpenGL routines
if MODE == 'SIL':
    import threading
    from opengl.Python.opengl import glhandle_g, MATLABMatrix,NUMCONTROLS
    # Setup local matrices for pushing updates to the renderer
    cgin = MATLABMatrix(3,1)
    ptpin = MATLABMatrix(3,1)
    keyboard_out = [0.0] * NUMCONTROLS

def run_physics_simulation():
    global RunTime,commands,logTime,printTime,telemetryTime
    #This runs on repeat until code is killed
    print('Running main loop....')

    ##If running in SIL mode we need to run in real time so we need a 
    #real start time to compare to the simulation time
    if MODE == 'SIL':
        while not glhandle_g.ready:
            realStartTime = time.time()
            print('Sleep to wait for OpenGL initialization...')
            time.sleep(1)

    prev_armed = None
    prev_control_color = None

    while (RunTime <= TFINAL):

        #Get Time
        LastTime = RunTime
        if MODE == 'SIMONLY' or MODE == 'SIL':
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

        if ALWAYSARMED:
            #System is always armed
            ARMED = True
            safety_color = 'Green'
            rc.rcin.armswitch = rc.rcin.STICK_MAX
            rc.rcin.autopilot = rc.rcin.STICK_MAX

        #Announce arming state changes in stdout
        if ARMED != prev_armed:
            prev_armed = ARMED
            status_text = "ARMED" if ARMED else f"DISARMED ({safety_color})"
            print(f"\n{'='*40}\n>>> RC STATE: {status_text} <<<\n{'='*40}\n")

        #Get GPS update if it's ready
        gps_llh.poll(RunTime) 

        #Get acceleration,gyroscope, magnetometer & temperature data
        #Note I do not recommend using rpy since that is solely using trigonometry
        #in addition the yaw angle does not work at all
        #rpy_ahrs works really well for obtaining the yaw angle
        #It's possible though that the trigonometry routine for yaw angle doesn't work
        #because it assumes the magnetic field is bhat = [1,0,0] and thus you need to calibrate said sensor
        #during boot up. Might be a good thing to test once you start hardware testing.
        #compass is the ahrs magnetometer heading + the gps heading
        a,gdegs,m,rpy,rpy_ahrs,temp,compass = imu.getALL(elapsedTime,gps_llh.heading) 

        #Get pressure which also converts to altitude (temperature is not currently working to speed up simulation)
        #If you really want temperature check the baro.poll() function and uncomment the temperature routine
        #Also keep in mind that the MPU9250 returns temperature automatically
        baro.poll(RunTime)

        #Run your control loop
        controls,defaults,control_color = vehicle.loop(RunTime,rc.rcin,gps_llh,rpy_ahrs,gdegs,baro)

        #Announce autopilot state changes in stdout
        if ARMED:
            if control_color != prev_control_color:
                prev_control_color = control_color
                if control_color == 'Blue':
                    print(f"\n{'='*40}\n>>> AUTOPILOT ENGAGED <<<\n{'='*40}\n")
                else:
                    print(f"\n{'='*40}\n>>> MANUAL FLIGHT CONTROL <<<\n{'='*40}\n")
        else:
            prev_control_color = None

        #Check if we are armed or not
        if ARMED:
            led.setColor(control_color)
            commands = controls
        else:
            led.setColor(safety_color)
            commands = defaults

        ##Send PWM signals to rcio
        rc.set_commands(commands)

        # IF running in SIL mode push states to the OpenGL shared state manager
        if MODE == 'SIL':
            # Extract position (X, Y, Z) and attitude (Roll, Pitch, Yaw) from your model/sensors
            cgin.set(1, 1, model.state[0])  # X position
            cgin.set(2, 1, model.state[1])  # Y position
            cgin.set(3, 1, model.state[2])  # Z position

            ptpin.set(1, 1, model.ptp[0])  # Roll (rad)
            ptpin.set(2, 1, model.ptp[1])  # Pitch (rad)
            ptpin.set(3, 1, model.ptp[2])  # Yaw (rad)

            # Safely push updates to the renderer via thread-safe lock
            glhandle_g.state.UpdateRender(RunTime, cgin, ptpin, 1, keyboard_out, ARMED, safety_color, control_color)

        #Print to Home
        if (RunTime - printTime) > PRINTTIME:
            str_rpy = [f"{ang:3.3f}" for ang in rpy_ahrs] #convert rpy to 3 sig figs
            str_g = [f"{gi:2.3f}" for gi in gdegs] #convert ang vel to 3 sig figs
            if MODE == 'SIMONLY':
                str_xyz = [f"{model.state[i]:1.3f}" for i in range(0,3)]
                str_speed = [f"{gps_llh.speed:3.3f}",f"{gps_llh.speed_x:3.3f}",f"{gps_llh.speed_y:3.3f}"]
                print(f"{RunTime:4.4f}", str_xyz, str_rpy,str_speed)
            else:
                if ARMED:
                    arm_str = "AUTOPILOT" if control_color == 'Blue' else "ARMED"
                else:
                    arm_str = f"DISARMED({safety_color})"
                str_pwm = [f"{pwm:1.3f}" for pwm in rc.pwm_commands] #convert pwm commands to 3 sig figs
                print(f"{RunTime:4.4f}", f"[{arm_str}]", f"{elapsedTime:1.4f}", rc.rcin.rcsignals, str_pwm, str_rpy, str_g, f"{baro.ALT:.3f}", gps_llh.altitude)
            printTime+=PRINTTIME

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
            if gps_llh.speed_x == -99:
                speed_X = 0.0
                speed_Y = 0.0
            else:
                ang = rpy_ahrs[2]*np.pi/180.0
                speed_X = gps_llh.speed_x*np.cos(ang) + gps_llh.speed_y*np.sin(ang)
                speed_Y = -gps_llh.speed_x*np.sin(ang) + gps_llh.speed_y*np.cos(ang)
            logger.outdata[7] = speed_X
            logger.outdata[8] = speed_Y
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
            #RC Channel #1-6
            #print(rc.rcin.rcsignals)
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
        #Now, if we're running in SIL mode, we need a wait loop until our real time catches up to our simulation time
        if MODE == 'SIL':
            while (time.time() - realStartTime) < RunTime:
                time.sleep(0.001) #sleep for 1 ms to not burn CPU cycles

##Now we actually start the main routine
if MODE=='SIL':
    print("Launching with OpenGL Graphical Interface...")
    # 1. Start physics engine on a background thread
    physics_thread = threading.Thread(target=run_physics_simulation, daemon=True)
    physics_thread.start()

    # 2. Run the OpenGL GUI main loop on the primary main thread
    glhandle_g.loop_init(fileroot=None,vehicleType=VEHICLE,inFarplane=10000, inWidth=800, inHeight=600)
else:
    print("Launching in Headless / Non-GUI Mode...")
    # Run physics directly on the main thread (ideal for Raspberry Pi & server runs)
    run_physics_simulation()

#If the program ends it means we're running in modeling mode
#and so we need to copy soem files
logger.close()
model.logger.close()
command = 'cp ' + str(logger.filename) + ' ' + str(logger.directory) + '0.csv'
os.system(command)
command = 'cp ' + str(model.logger.filename) + ' ' + str(model.logger.directory) + '0.csv'
os.system(command)
