import numpy as np
import matplotlib.pyplot as plt


###THIS FILE WAS THE WAYPOINT TEST
data = np.loadtxt('logs/Ground_Station_09_20_2022_15_00_35.csv')
##THIS WAS TELEMETRY TEST
#data = np.loadtxt('logs/Ground_Station_09_20_2022_14_45_38.csv')
##THIS WAS HEADING TEST
#data = np.loadtxt('logs/Ground_Station_09_20_2022_14_39_39.csv')

time = data[:,0]
lat = data[:,1]
lon = data[:,2]
compass = data[:,3]
imuyaw = data[:,4]
gpsyaw = data[:,5]

###HEADING PLOT
plt.figure()
plt.plot(time,compass,label='Compass')
plt.plot(time,imuyaw,label='IMU')
plt.plot(time,gpsyaw,label='GPS')
plt.xlabel('Time (sec)')
plt.ylabel('Degrees (deg)')
plt.grid()
plt.legend()

plt.figure()
plt.plot(lon,lat)
plt.grid()
plt.xlabel('Longitude (Deg)')
plt.ylabel('Latitude (Deg)')

##CONVERT TO XY
NM2FT=6076.115485560000
FT2M=0.3048
latO = 30.49168586730957
lonO = -88.21194458007812
x_vec = (lat - latO)*60*NM2FT*FT2M; #%%//North direction - Xf , meters
y_vec = (lon - lonO)*60*NM2FT*FT2M*np.cos(latO*np.pi/180); #%%//East direction - Yf, meters

plt.figure()
plt.plot(x_vec,y_vec)
plt.grid()
plt.xlabel('X (m)')
plt.ylabel('Y (m)')

dx = x_vec[1:-1]-x_vec[:-2]
dy = y_vec[1:-1]-y_vec[:-2]
dt = time[1:-1]-time[:-2]
dd = np.sqrt(dx**2 + dy**2)
speed = dd/dt

plt.figure()
plt.plot(time[0:-2],speed)
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Speed (m/s)')

plt.show()
