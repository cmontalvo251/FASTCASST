#!/usr/bin/python3

####USER INPUTS
#0 = use fake data file to read data
#1 = read csv files from SIL mode 
#2 = use the serial port to read data
SERIAL = 2 #0, 1 or 2
#0 = data just prints to command line
#1 = data also prints to a nice GUI
GUI = 1 #0 or 1

import numpy as np
import time
import datetime
import matplotlib.pyplot as plt
import matplotlib.patches as pt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os
import sys
sys.path.append('../libraries/')
from Comms.Comms import Comms as U

def readFile(filename):
	##Alright when running in SIL mode we will need to read the csv files.
	#First we are expecting 8 packets from FASTCASST
	file = open(filename)
	counter = 0
	for line in file:
		ser.fast_packet[counter] = ser.lineToFloat(line)
		counter+=1
	file.close()
	return ser.fast_packet

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

def get_fake_data(counter):
	t = time.monotonic()
	lat = 32.69 + 1.0 * np.sin(t)
	lon = -88.1 + 1.0*np.cos(t)
	gps_altitude = 50 + 10.0*np.sin(t)
	baro_pressure = 1013 - 3.0*(np.sin(t)+1)
	altitude = ConvertPressure2Alt(baro_pressure)
	roll = 0.0 + 10.0*np.sin(t)
	pitch = 0.0 + 20.0*np.sin(t)
	compass = 0 + 45.0*np.sin(t)
	gps_speed = 22.0 + 5.0*np.sin(t)
	pitot_speed = 22.0 + 5.0*np.cos(t)
	throttle = 1500. + 500*np.sin(t)
	aileron = 1500. + 500*np.sin(t)
	elevator = 1500. - 500*np.sin(t)
	rudder = 1500. + 500*np.cos(t)
	gndstation_packet[0] = t
	gndstation_packet[1] = roll
	gndstation_packet[2] = pitch
	gndstation_packet[3] = compass
	gndstation_packet[4] = lat
	gndstation_packet[5] = lon
	gndstation_packet[6] = altitude
	gndstation_packet[7] = gps_speed
	gndstation_packet[8] = gps_altitude
	gndstation_packet[9] = pitot_speed
	gndstation_packet[10] = throttle
	gndstation_packet[11] = aileron
	gndstation_packet[12] = elevator
	gndstation_packet[13] = rudder
	return gndstation_packet

def updatePacket(value,position):
	if position >= 0:
		ser.fast_packet[position] = value
		gndstation_packet[0] = ser.fast_packet[0] #time
		gndstation_packet[1] = ser.fast_packet[1] #roll
		gndstation_packet[2] = ser.fast_packet[2] #pitch
		gndstation_packet[3] = ser.fast_packet[3] #compass
		gndstation_packet[4] = ser.fast_packet[4] #lat
		gndstation_packet[5] = ser.fast_packet[5] #lon
		gndstation_packet[6] = ser.fast_packet[6] #baro altitude
		gndstation_packet[7] = ser.fast_packet[7] #gps speed
		gndstation_packet[8] = -99 #GPS Altitude
		gndstation_packet[9] = -99 #Pitot speed
		gndstation_packet[10] = -99 #throttle
		gndstation_packet[11] = -99 #aileron
		gndstation_packet[12] = -99 #elevator
		gndstation_packet[13] = -99 #rudder
		NEW_DATA = True
		print('Fast Packet = ',ser.fast_packet)
	else:
		NEW_DATA = False
	return gndstation_packet,NEW_DATA


class WINDOW():
	def __init__(self,parent=None):
		print('Initializing Window')
		####CREATE PLOTS
		self.fig = plt.figure(figsize=(144,81))
		##GRID 1,1
		print('Grid 1,1')
		self.ax11 = self.fig.add_subplot(332)
		self.ax11.get_xaxis().set_visible(False)
		self.ax11.get_yaxis().set_visible(False)
		##Grid 1,2 - LAT/LON
		print('Grid 1,2')
		self.ax12 = self.fig.add_subplot(331)
		##GRID 1,3
		print('Grid 1,3')
		self.ax13 = self.fig.add_subplot(333)
		##GRID 2,1
		print('Grid 2,1')
		self.ax21 = self.fig.add_subplot(336)
		self.ax21.get_xaxis().set_visible(False)
		self.ax21.get_yaxis().set_visible(False)
		###Grid 2,2 - 3D Cube
		print('Grid 2,2')
		self.ax22 = self.fig.add_subplot(335,projection='3d')
		###Grid 2,3
		print('Grid 2,3')
		self.ax23 = self.fig.add_subplot(334)
		##Grid 3,1
		print('Grid 3,1')
		self.ax31 = self.fig.add_subplot(338)
		self.ax31.get_xaxis().set_visible(False)
		self.ax31.get_yaxis().set_visible(False)
		###Grid 3,2 - Speed bars
		print('Grid 3,2')
		self.ax32 = self.fig.add_subplot(337)
		##GRID 3,3
		print('Grid 3,3')
		self.ax33 = self.fig.add_subplot(339)
		#Create empty arrays
		self.t = []
		self.longitude = []
		self.latitude = []

	def R123(self,phi,theta,psi):
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

	def CubeDraw(self,ax,phi,theta,psi):
		dx = 1.0
		dy = 0.5
		dz = 0.25
		##FACES (TOP,BOTTOM,FRONT,BACK,SIDE,SIDE)
		xv = np.array([[-1,1,1,-1],[-1,1,1,-1],[1,1,1,1],[-1,-1,-1,-1],[1,1,-1,-1],[1,1,-1,-1]])*dx/2.0
		yv = np.array([[1,1,-1,-1],[1,1,-1,-1],[-1,-1,1,1],[-1,-1,1,1],[1,1,1,1],[-1,-1,-1,-1]])*dy/2.0
		zv = np.array([[-1,-1,-1,-1],[1,1,1,1],[-1,1,1,-1],[-1,1,1,-1],[1,-1,-1,1],[1,-1,-1,1]])*dz/2.0
		T = self.R123(phi,theta,psi)
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

	def sendNewData(self,gndstation_packet):
		time = gndstation_packet[0] #time
		self.t.append(time)
		self.roll = gndstation_packet[1]  #roll
		self.pitch = gndstation_packet[2] #pitch
		self.yaw = gndstation_packet[3] #yaw
		latitude = gndstation_packet[4] #lat
		longitude = gndstation_packet[5] #lon
		if latitude > 30 and latitude < 40 and longitude < -80 and longitude > -90:
			self.latitude.append(latitude)
			self.longitude.append(longitude)
		self.baro_altitude = gndstation_packet[6]
		#baro_pressure = gndstation_packet[7]
		#self.baro_altitude = ConvertPressure2Alt(baro_pressure)
		self.gps_speed = gndstation_packet[7]
		self.gps_altitude = gndstation_packet[8]
		self.pitot_speed = gndstation_packet[9]
		self.throttle = gndstation_packet[10]
		self.aileron = gndstation_packet[11]
		self.elevator = gndstation_packet[12]
		self.rudder = gndstation_packet[13]

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
		try:
			self.ax11.text(0,0.9,'Latitude = '+str(self.latitude[-1]))
			self.ax11.text(0,0.6,'Longitude = '+str(self.longitude[-1]))
		except:
			pass
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
		comm_altitude = 50.0
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
		self.CubeDraw(self.ax22,self.roll*np.pi/180.,self.pitch*np.pi/180.0,self.yaw*np.pi/180.0)
		##GRID 2,3
		self.ax23.text(0,0.5,'Time (sec) = '+str(self.t[-1]))
		##GRID 3,1
		self.ax31.text(0,0.8,'GPS Speed (m/s) ='+str(self.gps_speed))
		self.ax31.text(0,0.4,'Pitot Speed (m/s) = '+str(self.pitot_speed))
		##GRID 3,2
		self.ax32.plot([-10,10],[self.gps_speed,self.gps_speed],'b-',label='GPS')
		self.ax32.plot([-10,10],[self.pitot_speed,self.pitot_speed],'r-',label='Pitot Probe')
		comm_speed = 15.0
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

##CREATE EMPTY ARRAY PACKETS
gndstation_packet = np.zeros(14)

##Initialize ground station log file
outfilename = 'logs/Ground_Station_'+datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")+'.csv'
outfile = open(outfilename,'w')

#Create Empty serial object
ser = U()

if SERIAL == 2:
	##Open Serial Comms
	print('Opening Serial port')
	print('All available serial ports...')
	os.system('ls /dev/ttyUSB*')
	#ser = U(57600,"/dev/ttyUSB0",period=1.0) 
	ser.SerialInit(57600,"/dev/ttyUSB0",period=1.0)	
	print('Serial initialization done')

##If GUI is on create the window
if GUI:
	##Create window
	print('Creating Window')
	GND = WINDOW()
	print('Window Created')

##MAIN LOOP
loop_counter = 0
filenumber = 0
NEW_DATA = False
while True:
	##Increment Loop counter
	loop_counter += 1
	
	##READ DATA DEPENDING ON SERIAL FLAG
	#Use Fake data for reading data
	if SERIAL==0:
		NEW_DATA = True
		gndstation_packet = get_fake_data(loop_counter)
	#Read data from csv file created by SIL
	elif SERIAL==1:
		filenumberNEW = maxFile(filenumber)
		print('Fast SIL at = ',filenumberNEW,' Python Time = ',time.monotonic())
		if filenumberNEW != filenumber:
			#This means we have a new file
			filenumber = filenumberNEW
			#Now this means though we need to read the file before
			if filenumber > 0:
				print('Reading File = ',filenumber-1)
				ser.fast_packet = readFile(getFileName(filenumber-1))	
				gndstation_packet,NEW_DATA = updatePacket(ser.fast_packet[0],0)
	elif SERIAL==2:
		position = -1
		print('Reading Serial....',time.monotonic())
		value,position,bytestring = ser.SerialGetNumber(1)
		print('Value Received, Position, Bytes = ',value,position,bytestring)
		gndstation_packet,NEW_DATA = updatePacket(value,position)
		#print('HERE: GUI,NEW_DATA,position = ',GUI,NEW_DATA,position)
		if position < 7:
			NEW_DATA = False

	##If the GUI is on update the window
	#print('GUI,NEW_DATA,position = ',GUI,NEW_DATA,position)
	if GUI==1 and NEW_DATA==True:
		print("Updating GUI.....")
		GND.clearwindow()
		GND.sendNewData(gndstation_packet)
		GND.updatewindow()
		plt.pause(0.0000000000001)

	##Write data to file if we got new data
	if NEW_DATA:
		outstr = ''
		for i in gndstation_packet:
			outstr+=(str(i)+' ')
		print('Packets Received (time) = ',gndstation_packet[0])
		outfile.write(outstr)
		outfile.write('\n')
		##Reset new data flag
		NEW_DATA = False
	
	

    

    
        
            

