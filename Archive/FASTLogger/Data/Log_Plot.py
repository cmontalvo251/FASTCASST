import numpy as np
import matplotlib.pyplot as plt
from pdf import *
import mio as m

pp = PDF(0,plt)

#file = open('Bench_Test_C++_6_24_2021_12.32PM.txt')
#delimeter = ' '
#file = open('Bench_Test_Python_6_24_2021_12.40PM.txt')
#filename = 'Bike_Test_7_13_14.26PM_C++.txt'
#filename = 'Bike_Test_7_13_14.33PM_Python.txt'
#filename = 'Bench_Test_Python_6_24_2021_12.40PM.txt'
#filename = 'Automobile_Test_7_20_2021.txt'
#filename = 'Drone_Test/Drone_Test_Did_Not_Takeoff_7_21_2021.txt'
filename = 'Drone_Test/Drone_Flew_Barely_7_21_2021.txt'
delimiter  = ','

data = m.loadtxt(filename,delimiter)
time = data[:,0]
looptime = data[:,1]
Pressure = data[:,2]
baro_TEMP = data[:,3]
IMU_TEMP = data[:,4]
CPU_TEMP = data[:,5]
lat = data[:,6]
lon = data[:,7]
gps_alt = data[:,8]
baro_alt = data[:,9]
roll = data[:,10]
pitch = data[:,11]
yaw = data[:,12]
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

fig = plt.figure()
timeP = np.delete(time,np.where(Pressure==-99))
Pressure = np.delete(Pressure,np.where(Pressure==-99))
plt.plot(timeP,Pressure)
plt.xlabel('Time')
plt.ylabel('Pressure(Pascals)')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,looptime)
plt.xlabel('Time (sec)')
plt.ylabel('Loop Time (sec)')
plt.grid()
pp.savefig()

plt.figure()
timeBaroT = np.delete(time,np.where(baro_TEMP==-99))
timeIMUT = np.delete(time,np.where(IMU_TEMP==-99))
timeCPUT = np.delete(time,np.where(CPU_TEMP==-99))
baro_TEMP = np.delete(baro_TEMP,np.where(baro_TEMP==-99))
IMU_TEMP = np.delete(IMU_TEMP,np.where(IMU_TEMP==-99))
CPU_TEMP = np.delete(CPU_TEMP,np.where(CPU_TEMP==-99))
plt.plot(timeBaroT,baro_TEMP,label='Baro')
plt.plot(timeIMUT,IMU_TEMP,label='IMU')
plt.plot(timeCPUT,CPU_TEMP,label='CPU')
plt.xlabel('Time')
plt.ylabel('Temperature (C)')
plt.legend()
plt.grid()
pp.savefig()

latstrip = []
lonstrip = []
for i in range(0,len(lat)):
	if lat[i] != -99 and lon[i] != -99 and int(lat[i]) != 0 and int(lon[i]) != 0:
		latstrip.append(lat[i])
		lonstrip.append(lon[i])
fig = plt.figure()
plti = fig.add_subplot(1,1,1)
plti.plot(lonstrip,latstrip)
plti.set_xlabel('Longitude')
plti.set_ylabel('Latitude')
plti.get_yaxis().get_major_formatter().set_useOffset(False)
plti.get_xaxis().get_major_formatter().set_useOffset(False)
plt.gcf().subplots_adjust(left=0.2)
plt.grid()
pp.savefig()

plt.figure()
timeGPS = np.delete(time, np.where(gps_alt == -99))
gps_alt = np.delete(gps_alt,np.where(gps_alt == -99))
timeBaro = np.delete(time, np.where(baro_alt == -99))
baro_alt = np.delete(baro_alt,np.where(baro_alt == -99))
plt.plot(timeGPS,gps_alt,label='GPS')
plt.plot(timeBaro,baro_alt,label='Baro')
plt.xlabel('Time')
plt.ylabel('Altitude (m)')
plt.legend()
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,roll)
plt.xlabel('Time')
plt.ylabel('Roll Angle (Degrees)')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,pitch)
plt.xlabel('Time')
plt.ylabel('Pitch Angle (degrees)')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,yaw)
plt.xlabel('Time')
plt.ylabel('Yaw Angle (Degrees')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,a_x)
plt.xlabel('Time')
plt.ylabel('X Acceleration (m/s^2')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,a_y)
plt.xlabel('Time')
plt.ylabel('Y Acceleration (m/s^2)')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,a_z)
plt.xlabel('Time')
plt.ylabel('Z Acceleration (m/s^2)')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,roll_rate)
plt.xlabel('Time')
plt.ylabel('Roll Rate (Degrees/s)')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,pitch_rate)
plt.xlabel('Time')
plt.ylabel('Pitch Rate (degrees/s')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,yaw_rate)
plt.xlabel('Time')
plt.ylabel('Yaw Rate (Degrees/s)')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,analog1)
plt.xlabel('Time')
plt.ylabel('Analog 1')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,analog2)
plt.xlabel('Time')
plt.ylabel('Analog 2')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,analog3)
plt.xlabel('Time')
plt.ylabel('Analog 3')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,analog4)
plt.xlabel('Time')
plt.ylabel('Analog 4')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,analog5)
plt.xlabel('Time')
plt.ylabel('Analog 5')
plt.grid()
pp.savefig()

plt.figure()
plt.plot(time,analog6)
plt.xlabel('Time')
plt.ylabel('Analog 6')
plt.grid()
pp.savefig()

pp.close()