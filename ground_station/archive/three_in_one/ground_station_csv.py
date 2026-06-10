#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import time
import struct
import datetime

#######FUNCTIONS######################
def bitsToFloat(b):
	if (b > 2147483647):
		b = -2147483648 + (b - 2147483647)
	s = struct.pack('>l', b)
	return struct.unpack('>f', s)[0]

def lineToFloat(line):
	##Ok this loop works. Time to unpack properly
	#Remove first 2 chars because that's just the number
	#followed by a colon (Also remove the trailing two characters. They are \r\n I think)
	hexdata = line[2:-2]
	#Ok now what we do is convert the hex number to an int
	integer = int(hexdata,16)
	#Then finally we convert the int bits to float
	value = bitsToFloat(integer)
	#And print it for debugging
	#print('0x = ',hexdata,' intbits = ',integer,' float = ',value)
	return value

def readFile(filename):
	##Alright when running in SIL mode we will need to read the csv files.
	#First we are expecting 6 packets from FASTKit
	telemetry_packet = np.zeros(6)
	file = open(filename)
	counter = 0
	for line in file:
		telemetry_packet[counter] = lineToFloat(line)
		counter+=1
	file.close()
	return telemetry_packet

def getFileName(filenumber):
	return '../' + str(filenumber) + '.csv'

def maxFile(filenumber):
	searching = 1
	while searching:
		try:
			file = open(getFileName(filenumber))
			file.close()
			filenumber+=1
		except:
			#If you throw an error it means that
			#you found the last file
			searching = 0
	if filenumber < 0:
		filenumber = 1
	return filenumber-1

def ConvertPressure2Alt(pressure):
    pascals = pressure/0.01;
    altitude = (1.0-((pascals/101325.0)**(1.0/5.25588)))/(2.2557*(10**(-5.0)))
    return altitude

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

class WINDOW():
	def __init__(self,parent=None):
		####CREATE PLOTS
		self.fig = plt.figure(figsize=(144,81))
		##GRID 1,1
		self.ax11 = self.fig.add_subplot(332)
		self.ax11.get_xaxis().set_visible(False)
		self.ax11.get_yaxis().set_visible(False)
		##Grid 1,2 - LAT/LON
		self.ax12 = self.fig.add_subplot(331)
		##GRID 1,3
		self.ax13 = self.fig.add_subplot(333)
		##GRID 2,1
		self.ax21 = self.fig.add_subplot(336)
		self.ax21.get_xaxis().set_visible(False)
		self.ax21.get_yaxis().set_visible(False)
		###Grid 2,2 - 3D Cube
		self.ax22 = self.fig.add_subplot(335,projection='3d')
		###Grid 2,3
		self.ax23 = self.fig.add_subplot(334)
		##Grid 3,1
		self.ax31 = self.fig.add_subplot(338)
		self.ax31.get_xaxis().set_visible(False)
		self.ax31.get_yaxis().set_visible(False)
		###Grid 3,2 - Speed bars
		self.ax32 = self.fig.add_subplot(337)
		##GRID 3,3
		self.ax33 = self.fig.add_subplot(339)
		#Create empty arrays
		self.t = []
		self.longitude = []
		self.latitude = []
		#Create logging file
		outfilename = 'logs/Ground_Station_'+datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")+'.csv'
		self.outfile = open(outfilename,'w')

	def logData(self,telemetry_packet):
		ctr = 0
		for o in telemetry_packet:
			s = str(o)
			if ctr != len(telemetry_packet)-1:
				s+=","
			ctr+=1
			self.outfile.write(s)
		self.outfile.write("\n")
		self.outfile.flush()

	def sendNewData(self,telemetry_packet):
		time = telemetry_packet[0]
		self.t.append(time)
		self.roll = telemetry_packet[1]
		self.pitch = telemetry_packet[2]
		self.yaw = telemetry_packet[3]
		latitude = telemetry_packet[4]
		self.latitude.append(latitude)
		longitude = telemetry_packet[5]
		self.longitude.append(longitude)
		self.gps_altitude = telemetry_packet[6]
		baro_pressure = telemetry_packet[7]
		self.baro_altitude = ConvertPressure2Alt(baro_pressure)
		self.gps_speed = telemetry_packet[8]
		self.pitot_speed = telemetry_packet[9]
		self.throttle = telemetry_packet[10]
		self.aileron = telemetry_packet[11]
		self.elevator = telemetry_packet[12]
		self.rudder = telemetry_packet[13]
		##Then log data
		self.logData(telemetry_packet)

	def clearwindow(self):
		self.ax11.clear()
		self.ax12.clear()
		self.ax13.clear()
		self.ax21.clear()
		self.ax22.clear()
		self.ax23.clear()
		self.ax31.clear()
		self.ax32.clear()
		self.ax33.clear()

	def updatewindow(self):
		##GRID 1,1
		self.ax11.text(0,0.9,'Latitude = '+str(self.latitude[-1]))
		self.ax11.text(0,0.6,'Longitude = '+str(self.longitude[-1]))
		self.ax11.text(0,0.3,'GPS Alt (m) = '+str(self.gps_altitude))
		self.ax11.text(0,0,'Baro Alt (m) = '+str(self.baro_altitude))
		##GRID 1,2
		self.ax12.plot(self.longitude,self.latitude)
		self.ax12.set_xlabel('Longitude (deg)')
		self.ax12.set_ylabel('Latitude (deg)')
		self.ax12.grid()
		##GRID 1,3
		self.ax13.plot([-10,10],[self.gps_altitude,self.gps_altitude],'b-',label='GPS')
		self.ax13.plot([-10,10],[self.baro_altitude,self.baro_altitude],'r-',label='Barometer')
		comm_altitude = 10
		self.ax13.plot([-10,10],[comm_altitude,comm_altitude],'g--',label='Command')
		self.ax13.legend()
		self.ax13.set_ylabel('Altitude (m)')
		self.ax13.grid()
		self.ax13.set_ylim([0,100])
		#GRID 2,1
		self.ax21.text(0,0.9,'Roll (deg) = '+str(self.roll))
		self.ax21.text(0,0.45,'Pitch (deg) = '+str(self.pitch))
		self.ax21.text(0,0,'Yaw (deg) = '+str(self.yaw))
		##GRID 2,2
		CubeDraw(self.ax22,self.roll*np.pi/180.,self.pitch*np.pi/180.0,self.yaw*np.pi/180.0)
		##GRID 2,3
		self.ax23.text(0,0.5,'Time (sec) = '+str(self.t[-1]))
		##GRID 3,1
		self.ax31.text(0,0.8,'GPS Speed (m/s) ='+str(self.gps_speed))
		self.ax31.text(0,0.4,'Pitot Speed (m/s) = '+str(self.pitot_speed))
		##GRID 3,2
		self.ax32.plot([-10,10],[self.gps_speed,self.gps_speed],'b-',label='GPS')
		self.ax32.plot([-10,10],[self.pitot_speed,self.pitot_speed],'r-',label='Pitot Probe')
		comm_speed = 20.0
		self.ax32.plot([-10,10],[comm_speed,comm_speed],'g--',label='Command')
		self.ax32.set_ylabel('Speed (m/s)')
		self.ax32.set_ylim([0,30])
		self.ax32.grid()
		self.ax32.legend()
		##GRID 3,3
		width = 10;
		self.ax33.add_patch(pt.Rectangle([0,0],width,self.throttle,fc='b'))
		self.ax33.add_patch(pt.Rectangle([width,0],width,self.aileron,fc='r'))
		self.ax33.add_patch(pt.Rectangle([2*width,0],width,self.elevator,fc='g'))
		self.ax33.add_patch(pt.Rectangle([3*width,0],width,self.rudder,fc='m'))
		self.ax33.grid()
		self.ax33.set_ylabel('PWM (us)')
		self.ax33.set_xlabel('Throttle   Aileron   Elevator  Rudder')
		self.ax33.set_xlim([0,40])
		self.ax33.set_ylim([0,2400])
        
##DATA - All data commented out comes from FASTKit at the moment
#Everything else is simulated for now
t = np.arange(0,1000*np.pi,1)
#latitude = 30.69 + 1.0 * np.sin(t)
#longitude = -88.1 + 1.0*np.cos(t)
gps_altitude = 10.5 + 1.0*np.sin(t)
baro_pressure = 1013.25 + 3.0*np.sin(t)
#roll = 0.0 + 10.0*np.exp(0.1*t)
#pitch = 0.0 + 20.0*np.sin(t)
#yaw = 0 + 45.0*np.sin(t)
gps_speed = 22.0 + 0.2*np.sin(t)
pitot_speed = 18.5 + 3.0*np.cos(t)
throttle = 1800. + 200*np.sin(t)
aileron = 992. + np.exp(0.1*t)
elevator = 2000. - np.exp(0.1*t)
rudder = 1500. + 500*np.cos(t)
telemetry_packet = np.zeros(14)
fastkit_packet = np.zeros(6)

##Create window
GND = WINDOW()
##Initialize Filenumber at zero
filenumber = 0
i = 0
while True:
	filenumberNEW = maxFile(filenumber)
	print('FastKit at = ',filenumberNEW,' Python Time = ',time.monotonic())
	if filenumberNEW != filenumber:
		#This means we have a new file
		filenumber = filenumberNEW
		#Now this means though we need to read the file before
		if filenumber > 0:
			print('Reading File = ',filenumber-1)
			fastkit_packet = readFile(getFileName(filenumber-1))	
			print(fastkit_packet)
			#Clear the GUI window
			GND.clearwindow()
			telemetry_packet[0] = fastkit_packet[0]
			telemetry_packet[1] = fastkit_packet[1]
			telemetry_packet[2] = fastkit_packet[2]
			telemetry_packet[3] = fastkit_packet[3]
			telemetry_packet[4] = fastkit_packet[4]
			telemetry_packet[5] = fastkit_packet[5]
			telemetry_packet[6] = gps_altitude[i]
			telemetry_packet[7] = baro_pressure[i]
			telemetry_packet[8] = gps_speed[i]
			telemetry_packet[9] = pitot_speed[i]
			telemetry_packet[10] = throttle[i]
			telemetry_packet[11] = aileron[i]
			telemetry_packet[12] = elevator[i]
			telemetry_packet[13] = rudder[i]
			i+=1
			GND.sendNewData(telemetry_packet)
			GND.updatewindow()
			plt.pause(0.01)
	#Then sleep for 0.1 seconds
	time.sleep(0.1)
