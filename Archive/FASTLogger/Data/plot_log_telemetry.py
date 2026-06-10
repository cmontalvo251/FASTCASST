import mio as m
import numpy as np
import quad as Q
import matplotlib.pyplot as plt
from pdf import *

##Create PDF Object
pp = PDF(0,plt)

###Import Telemetry First
data = np.loadtxt('Drone_Test/Drone_Flew_Barely_Telemetry_2021_7_21_9_38_47.txt')
roll_telem = data[:,0]
pitch_telem = data[:,1]
yaw_telem = data[:,2]
lon_telem = data[:,3]
lat_telem = data[:,4]
altitude_telem = data[:,5]
time_telem = data[:,6]

###Then import raspberry Pi log file
filename = 'Drone_Test/Drone_Flew_Barely_7_21_2021.txt'
delimiter  = ','

data = m.loadtxt(filename,delimiter)
timepi = data[:,0]
looptime = data[:,1]
Pressure = data[:,2]
baro_TEMP = data[:,3]
IMU_TEMP = data[:,4]
CPU_TEMP = data[:,5]
lat = data[:,6]
lon = data[:,7]
gps_alt = data[:,8]
baro_alt = data[:,9]
rollpi = data[:,10]
pitchpi = data[:,11]
yawpi = data[:,12]
roll_rate = data[:,13]
pitch_rate = data[:,14]
yaw_rate = data[:,15]
a_x = data[:,16]
a_y = data[:,17]
a_z = data[:,18]
analog1 = data[:,19]
analog2 = data[:,20]
analog3 = data[:,21]
analog4 = data[:,22]
analog5 = data[:,23]
analog6 = data[:,24]

###Finally Import Drone Data
#fileName = 'Drone_Test/example_quad_data.log'
fileName = 'Drone_Test/Drone_Test_IRIS_Takeoff_7_21_2021.txt'
quad_data = Q.get_quad_data(fileName)
if not quad_data:
    sys.exit()
print("Quad Data Directory = ",quad_data[-1])
baroRead = quad_data[0]
gpsRead = quad_data[1]
rcinoutRead = quad_data[2]
IMURead = quad_data[3]
ATTRead = quad_data[4]
BATTRead = quad_data[5]
print(dir(ATTRead))
print(dir(IMURead))
print(dir(baroRead))
offset = 23.5
yoffset = -100+10+10
factor = 1.0

####NOW MAKE PLOTS
plt.figure()
plt.plot(time_telem,roll_telem,'r*',label='Telemetry')
plt.plot(timepi,rollpi,'b-',label='FASTlogger')
plt.plot((ATTRead.timeMS_ATT-ATTRead.timeMS_ATT[0])*factor+offset,-ATTRead.ROLL,'g-',label='Iris+')
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Roll Angle (deg)')
plt.legend()
pp.savefig()

plt.figure()
plt.plot(time_telem,pitch_telem,'r*',label='Telemetry')
plt.plot(timepi,pitchpi,'b-',label='FASTlogger')
plt.plot((ATTRead.timeMS_ATT-ATTRead.timeMS_ATT[0])*factor+offset,-ATTRead.PITCH,'g-',label='Iris+')
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Pitch Angle (deg)')
plt.legend()
pp.savefig()

plt.figure()
plt.plot(time_telem,yaw_telem,'r*',label='Telemetry')
plt.plot(timepi,yawpi,'b-',label='FASTlogger')
plt.plot((ATTRead.timeMS_ATT-ATTRead.timeMS_ATT[0])*factor+offset,ATTRead.YAW+yoffset,'g-',label='Iris+')
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Yaw Angle (deg)')
plt.legend()
pp.savefig()

plt.figure()
plt.plot(timepi,a_z,label='FASTLogger')
#plt.plot(IMURead.timeMS_IMU-IMURead.timeMS_IMU[0],IMURead.AccX,label='IrisX')
#plt.plot(IMURead.timeMS_IMU-IMURead.timeMS_IMU[0],IMURead.AccY,label='IrisY')
plt.plot((IMURead.timeMS_IMU-IMURead.timeMS_IMU[0])*factor+offset,IMURead.AccZ+9.81,label='IrisZ')
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Z Acceleration (m/s^2)')
plt.legend()
pp.savefig()

plt.figure()
timeBaro = np.delete(timepi,np.where(Pressure==-99))
PressureStrip = np.delete(Pressure,np.where(Pressure==-99))
plt.plot((baroRead.timeSec-baroRead.timeSec[0])*factor+offset, baroRead.pressureMB, label='IRIS+')
plt.plot(timeBaro,PressureStrip,label='FASTLogger')
plt.grid()
plt.legend()
plt.xlabel('Time (sec)')
plt.ylabel('Pressure (MB)')
pp.savefig()

pp.close()