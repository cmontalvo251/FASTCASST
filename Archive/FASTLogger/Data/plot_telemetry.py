import numpy as np
import matplotlib.pyplot as plt
from pdf import *

pp = PDF(0,plt)
#data = np.loadtxt('Drone_Test/Outside_On_Drone_No_Takeoff_Telemetry_2021_7_21_9_31_59.txt')
data = np.loadtxt('Drone_Test/Drone_Flew_Barely_Telemetry_2021_7_21_9_38_47.txt')

roll = data[:,0]
pitch = data[:,1]
yaw = data[:,2]
lon = data[:,3]
lat = data[:,4]
altitude = data[:,5]
time = data[:,6]

plt.figure()
plt.plot(time,roll)
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Roll (deg)')
pp.savefig()

plt.figure()
plt.plot(time,pitch)
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Pitch (deg)')
pp.savefig()

plt.figure()
plt.plot(time,yaw)
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Yaw (deg)')
pp.savefig()

plt.figure()
plt.plot(time,altitude)
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('GPS Altitude (m)')
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

pp.close()


