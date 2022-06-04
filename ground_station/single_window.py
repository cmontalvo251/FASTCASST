#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

#######FUNCTIONS######################
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

def CubeDraw(ax,phi,theta,psi):
	dx = 1.0
	dy = 0.5
	dz = 0.25
	##FACES (TOP,BOTTOM,FRONT,BACK,SIDE,SIDE)
	xv = np.array([[-1,1,1,-1],[-1,1,1,-1],[1,1,1,1],[-1,-1,-1,-1],[1,1,-1,-1],[1,1,-1,-1]])*dx/2.0
	yv = np.array([[1,1,-1,-1],[1,1,-1,-1],[-1,-1,1,1],[-1,-1,1,1],[1,1,1,1],[-1,-1,-1,-1]])*dy/2.0
	zv = np.array([[-1,-1,-1,-1],[1,1,1,1],[-1,1,1,-1],[-1,1,1,-1],[1,-1,-1,1],[1,-1,-1,1]])*dz/2.0
	T = R123(phi,theta,psi)
	xc = 0.5
	yc = 0.5
	zc = 0.5
	rc = np.array([xc,yc,zc])
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
		pc = Poly3DCollection([list(zip(xv[jj],yv[jj],zv[jj]))],alpha=0.5)
		pc.set_facecolor('r')
		pc.set_edgecolor('k')
		ax.add_collection(pc)
	ax.get_xaxis().set_visible(False)
	ax.get_yaxis().set_visible(False)
	ax.get_zaxis().set_visible(False)
##############################################

##DATA
t = np.linspace(0,2*np.pi,100)
latitude = 30.69 + 1.0 * np.sin(t)
longitude = -88.1 + 1.0*np.cos(t)
gps_altitude = 10.5
baro_altitude = 8.5
comm_altitude = 10.
roll = 0.0*np.pi/180.0
pitch = 0*np.pi/180.0
yaw = 0*np.pi/180.0
gps_speed = 22.0
pitot_speed = 18.5
comm_speed = 20.0
throttle = 1800.
aileron = 992.
elevator = 2000.
rudder = 1500.
width = 10;

####CREATE PLOTS
fig = plt.figure(figsize=(1440,810))
##GRID 1,1
ax11 = fig.add_subplot(332)
ax11.text(0,0.9,'Latitude = '+str(latitude[10]))
ax11.text(0,0.6,'Longitude = '+str(longitude[10]))
ax11.text(0,0.3,'GPS Alt (m) = '+str(gps_altitude))
ax11.text(0,0,'Baro Alt (m) = '+str(baro_altitude))
ax11.get_xaxis().set_visible(False)
ax11.get_yaxis().set_visible(False)
##Grid 1,2 - LAT/LON
ax12 = fig.add_subplot(331)
ax12.plot(longitude,latitude)
ax12.set_xlabel('Longitude (deg)')
ax12.set_ylabel('Latitude (deg)')
ax12.grid()
##Grid 1,3 - Altitude bars
ax13 = fig.add_subplot(333)
ax13.plot([-10,10],[gps_altitude,gps_altitude],'b-',label='GPS')
ax13.plot([-10,10],[baro_altitude,baro_altitude],'r-',label='Barometer')
ax13.plot([-10,10],[comm_altitude,comm_altitude],'g--',label='Command')
ax13.set_ylabel('Altitude (m)')
ax13.grid()
ax13.legend()
##GRID 2,1
ax21 = fig.add_subplot(336)
ax21.text(0,0.9,'Roll (deg) = '+str(roll*180./np.pi))
ax21.text(0,0.45,'Pitch (deg) = '+str(pitch*180./np.pi))
ax21.text(0,0,'Yaw (deg) = '+str(yaw*180./np.pi))
ax21.get_xaxis().set_visible(False)
ax21.get_yaxis().set_visible(False)
###Grid 2,2 - 3D Cube
ax22 = fig.add_subplot(335,projection='3d')
CubeDraw(ax22,roll,pitch,yaw)
###Grid 2,3
ax23 = fig.add_subplot(334)
ax23.text(0,0.5,'Time (sec) = '+str(t[10]))
##Grid 3,1
ax31 = fig.add_subplot(338)
ax31.text(0,0.8,'GPS Speed (m/s) ='+str(gps_speed))
ax31.text(0,0.4,'Pitot Speed (m/s) = '+str(pitot_speed))
ax31.get_xaxis().set_visible(False)
ax31.get_yaxis().set_visible(False)
###Grid 3,2 - Speed bars
ax32 = fig.add_subplot(337)
ax32.plot([-10,10],[gps_speed,gps_speed],'b-',label='GPS')
ax32.plot([-10,10],[pitot_speed,pitot_speed],'r-',label='Pitot Probe')
ax32.plot([-10,10],[comm_speed,comm_speed],'g--',label='Command')
ax32.set_ylabel('Speed (m/s)')
ax32.set_ylim([0,30])
ax32.grid()
ax32.legend()
###Grid 3,3 - Bar Graphs
ax33 = fig.add_subplot(339)
ax33.grid()
ax33.add_patch(pt.Rectangle([0,0],width,throttle,fc='b'))
ax33.add_patch(pt.Rectangle([width,0],width,aileron,fc='r'))
ax33.add_patch(pt.Rectangle([2*width,0],width,elevator,fc='g'))
ax33.add_patch(pt.Rectangle([3*width,0],width,rudder,fc='m'))
ax33.set_ylabel('PWM (us)')
ax33.set_xlabel('Throttle   Aileron   Elevator  Rudder')
ax33.set_xlim([0,40])
ax33.set_ylim([0,2400])

plt.show()