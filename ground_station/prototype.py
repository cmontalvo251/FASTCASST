#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def R123(phi,theta,psi):
    #%compute R such that v(inertial) = R v(body)
    #%Compute sines and cosines
    ctheta = np.cos(theta);
    stheta = np.sin(theta);
    sphi = np.sin(phi);
    cphi = np.cos(phi);
    spsi = np.sin(psi);
    cpsi = np.cos(psi);
    #%Kinematics
    R = np.array([[ctheta*cpsi,sphi*stheta*cpsi-cphi*spsi,cphi*stheta*cpsi+sphi*spsi],[ctheta*spsi,sphi*stheta*spsi+cphi*cpsi,cphi*stheta*spsi-sphi*cpsi],[-stheta,sphi*ctheta,cphi*ctheta]]);
    return R

def CubeDraw(phi,theta,psi):
	dx = 1.0
	dy = 1.0
	dz = 1.0
	##FACES (TOP,BOTTOM,FRONT,BACK,SIDE,SIDE)
	xv = np.array([[-1,1,1,-1],[-1,1,1,-1],[1,1,1,1],[-1,-1,-1,-1],[1,1,-1,-1],[1,1,-1,-1]])*dx/2.0
	yv = np.array([[1,1,-1,-1],[1,1,-1,-1],[-1,-1,1,1],[-1,-1,1,1],[1,1,1,1],[-1,-1,-1,-1]])*dy/2.0
	zv = np.array([[-1,-1,-1,-1],[1,1,1,1],[-1,1,1,-1],[-1,1,1,-1],[1,-1,-1,1],[1,-1,-1,1]])*dz/2.0
	T = R123(phi,theta,psi)
	xc = dx/2.0
	yc = dy/2.0
	zc = dz/2.0
	rc = np.array([xc,yc,zc])
	fig = plt.figure()
	ax = fig.add_subplot(111,projection='3d')
	for jj in range(0,6): ##Loop through faces
		for ii in range(0,4): ##Loop through vertices
			xs = xv[jj][ii]
			ys = yv[jj][ii]
			zs = zv[jj][ii]
			xyz = np.array([xs,ys,zs])
			xyz_trans = rc + np.matmul(T,xyz);
			xv[jj][ii] = xyz_trans[0]
			yv[jj][ii] = xyz_trans[1]
			zv[jj][ii] = xyz_trans[2]
		pc = Poly3DCollection([list(zip(xv[jj],yv[jj],zv[jj]))])
		pc.set_facecolor('r')
		pc.set_edgecolor('k')
		ax.add_collection(pc)

# Lat/Lon/Alt       |
# Baro Alt          |  MAP (LAT/LON in Cartesion) | Altitude Bars 
# -------------------------------------------------------------------------
# RPY               |   3D Cube                    | Time
# -------------------------------------------------------------------------
# Pitot Speed       |                              
# GPS Speed         |  Speed Bars                | Bar graphs of RX signals

# 1 - lat -> convert to X
# 2 - lon -> convert to Y
# 3 - gps alt
# 4 - baro pressure -> Convert to altitude
# 5 - roll
# 6 - pitch
# 7 - yaw
# 8 - Time
# 9  - ADC signal (x2?)
# 10 - GPS speed - compute myself? 
# 11 - 14/15 - RX signals (4 or 5 signals)

##Let's make individual plots first

##Grid 1,2 - LAT/LON
t = np.linspace(0,2*np.pi,100)
latitude = 30.69 + 1.0 * np.sin(t)
longitude = -88.1 + 1.0*np.cos(t)
plt.plot(longitude,latitude)
plt.grid()
plt.xlabel('Longitude (deg)')
plt.ylabel('Latitude (deg)')

##Grid 1,3 - Altitude bars
gps_altitude = 10.5
baro_altitude = 8.5
comm_altitude = 10.
plt.figure()
plt.plot([-10,10],[gps_altitude,gps_altitude],'b-',label='GPS')
plt.plot([-10,10],[baro_altitude,baro_altitude],'r-',label='Barometer')
plt.plot([-10,10],[comm_altitude,comm_altitude],'g--',label='Command')
plt.ylabel('Altitude (m)')
plt.ylim([-10,50])
plt.grid()
plt.legend()

###Grid 2,2 - 3D Cube
roll = 20.0*np.pi/180.0
pitch = -30*np.pi/180.0
yaw = 75*np.pi/180.0
CubeDraw(roll,pitch,yaw)

###Grid 3,2 - Speed bars
gps_speed = 22.0
pitot_speed = 18.5
comm_speed = 20.0
plt.figure()
plt.plot([-10,10],[gps_speed,gps_speed],'b-',label='GPS')
plt.plot([-10,10],[pitot_speed,pitot_speed],'r-',label='Pitot Probe')
plt.plot([-10,10],[comm_speed,comm_speed],'g--',label='Command')
plt.ylabel('Speed (m/s)')
plt.ylim([0,30])
plt.grid()
plt.legend()

###Grid 3,3 - Bar Graphs
throttle = 1800.
aileron = 992.
elevator = 2000.
rudder = 1500.
width = 10.0
fig = plt.figure()
ax = fig.add_subplot(111)
plt.grid()
ax.add_patch(pt.Rectangle([0,0],width,throttle,fc='b'))
ax.add_patch(pt.Rectangle([width,0],width,aileron,fc='r'))
ax.add_patch(pt.Rectangle([2*width,0],width,elevator,fc='g'))
ax.add_patch(pt.Rectangle([3*width,0],width,rudder,fc='m'))
plt.ylabel('PWM (us)')
plt.xlabel('Throttle   Aileron   Elevator  Rudder')
plt.xlim([0,40])
plt.ylim([0,2400])
plt.show()