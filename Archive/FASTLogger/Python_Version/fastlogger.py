##Include the datalogger class
import sys

#####APPEND LOCATION OF HIL
sys.path.append('/home/pi/HIL/')
#Import some things that everything uses
import Util.util

###BAROMETER
import Baro.ms5611
baro = Baro.ms5611.MS5611()
baro.initialize()

######IMU
import IMU.mpu9250
imu = IMU.mpu9250.MPU9250()
imu.initialize()

###GPS
import GPS.gps
gps = GPS.gps.GPS()
gps.initialize()

###ADC
import ADC.adc
adc = ADC.adc.ADC()
results = [0] * adc.channel_count

#######TIMER
import time

sys.path.append('/home/pi/Python/')
#Creating a variable called Datalogger. just like int or double I can do Datalogger
#Looked for the FIle
from datalogger.datalogger import *
print("Looking for File in " + sys.argv[1]);
logger = Datalogger()
logger.findfile(sys.argv[1]);
#Then we open it
logger.open();
#Let's make a MATLAB variable for outputting data
import numpy as np
outdata = np.zeros(25)

##Import CPU temp module
from temperature.temperature import *
cputemp = temperature()

#######SERIAL/TELEMETRY
TELEMETRYONOFF = 1
if TELEMETRYONOFF:
        from telemetry.telemetry import *
        ser = Telemetry(57600,"/dev/ttyAMA0",period=1.0) #Set the baudrate, port and period in seconds
        ##Make a number array for telemetry
        number_array = np.zeros(7) 

#We create a loop to write stuff
t0 = time.time()
prevTime = time.time()

##DEBUG PARAM
DEBUG = 1 #Set to 0 to suppress output

#############MAIN ROUTINE####################3
print("Running Datalogger Test Script \n")
Util.util.check_apm()

for i in range(0,100000):

        #Poll Barometer
        baro.update()
        #Poll IMU
        a,g,rpy,imutemp = imu.getALL() #roll and pitch are both negative.
        #Poll GPS
        gps.update()
        #Poll ADC Channels
        snp = adc.update(results)
        #Updat Cputemp
        cputemp.update()

        ##Send Telemetry Data
        if TELEMETRYONOFF:
                if ((time.time()-t0)-ser.lastTime) > ser.period:
                        number_array[0] = rpy[0];
                        number_array[1] = rpy[1];
                        number_array[2] = rpy[2];
                        number_array[3] = gps.longitude;
                        number_array[4] = gps.latitude;
                        number_array[5] = gps.altitude;
                        number_array[6] = time.time()-t0;
                        ser.SerialSendArray(number_array,1) #trailing zero is to turn off echo
                        ser.lastTime = time.time()-t0;
                
        outstr = ''
        if DEBUG:
                outstr = 'i = ' + str(i)
                outstr += " Time = " + str(time.time()-t0) + " Loop = " + str(time.time()-prevTime)
                outstr += " Baro(P,T) = " +str(baro.PRES) + " " + str(baro.TEMP)
                outstr += " T(IMU) = " + str(imutemp) + " T (CPU) " + str(cputemp.temp)
                outstr += " RPY = " + str(rpy[0]) + " " + str(rpy[1]) +" " + str(rpy[2])
                outstr += " PQR = " + str(g[0]) + " " + str(g[1]) +" " + str(g[2])
                outstr += " ACC = " + str(a[0]) + " " + str(a[1]) +" " + str(a[2])
                outstr += " GPS = " + str(gps.latitude) + " " + str(gps.longitude) + " " + str(gps.altitude)
                outstr += " Baro Alt = " + str(baro.ALT)
                outstr += " ADC = " + str(snp[0]) + " " + str(snp[1]) + " " + str(snp[2]) + " " + str(snp[3]) + " " + str(snp[4]) + " " + str(snp[5])
                print(outstr)
                
        #Populate the outdata Matrix
        outdata[0] = time.time()-t0
        outdata[1] = time.time()-prevTime
        prevTime = time.time()
        outdata[2] = baro.PRES
        outdata[3] = baro.TEMP 
        outdata[4] = imutemp
        outdata[5] = cputemp.temp
        outdata[6] = gps.latitude
        outdata[7] = gps.longitude
        outdata[8] = gps.altitude
        outdata[9] = baro.ALT
        outdata[10] = rpy[0]
        outdata[11] = rpy[1]
        outdata[12] = rpy[2]
        outdata[13] = g[0]
        outdata[14] = g[1]
        outdata[15] = g[2]
        outdata[16] = a[0]
        outdata[17] = a[1]
        outdata[18] = a[2]
        outdata[19] = snp[0]
        outdata[20] = snp[1]
        outdata[21] = snp[2]
        outdata[22] = snp[3]
        outdata[23] = snp[4]
        outdata[24] = snp[5]
        logger.println(outdata);

        time.sleep(0.1)
