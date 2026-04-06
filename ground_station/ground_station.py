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
import matplotlib.widgets as mwidgets
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os
import sys
sys.path.append('../libraries/')
from Comms.Comms import Comms as U

EARTH_RADIUS_M = 6371000.0

def readFile(filename):
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
		gndstation_packet[7] = ser.fast_packet[7] #gps speed (m/s)
		gndstation_packet[8] = -99  #GPS Altitude (not in current packet)
		gndstation_packet[9] = -99  #Pitot speed
		gndstation_packet[10] = -99 #throttle
		gndstation_packet[11] = -99 #aileron
		gndstation_packet[12] = -99 #elevator
		gndstation_packet[13] = -99 #rudder
		NEW_DATA = True
		print('Fast Packet = ', ser.fast_packet)
	else:
		NEW_DATA = False
	return gndstation_packet, NEW_DATA


def _gps_bearing(lat1, lon1, lat2, lon2):
	"""Bearing from (lat1,lon1) to (lat2,lon2) in degrees [0,360)."""
	lat1_r, lat2_r = np.radians(lat1), np.radians(lat2)
	dlon_r = np.radians(lon2 - lon1)
	x = np.sin(dlon_r) * np.cos(lat2_r)
	y = np.cos(lat1_r)*np.sin(lat2_r) - np.sin(lat1_r)*np.cos(lat2_r)*np.cos(dlon_r)
	return (np.degrees(np.arctan2(x, y)) + 360.0) % 360.0

def _gps_distance(lat1, lon1, lat2, lon2):
	"""Haversine distance in meters."""
	lat1_r, lat2_r = np.radians(lat1), np.radians(lat2)
	dlat_r = np.radians(lat2 - lat1)
	dlon_r = np.radians(lon2 - lon1)
	a = np.sin(dlat_r/2)**2 + np.cos(lat1_r)*np.cos(lat2_r)*np.sin(dlon_r/2)**2
	return EARTH_RADIUS_M * 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))


class WINDOW():
	def __init__(self, parent=None):
		print('Initializing Window')
		self.fig = plt.figure(figsize=(144, 81))

		##Leave room at the bottom for waypoint controls
		self.fig.subplots_adjust(bottom=0.12)

		##Autopilot waypoint state
		self.target_lat = None
		self.target_lon = None
		self._pending_lat = ''
		self._pending_lon = ''

		####CREATE MAIN PLOTS (3x3 grid)
		print('Grid 1,1 - GPS Readout')
		self.ax11 = self.fig.add_subplot(332)
		self.ax11.get_xaxis().set_visible(False)
		self.ax11.get_yaxis().set_visible(False)

		print('Grid 1,2 - LAT/LON Map')
		self.ax12 = self.fig.add_subplot(331)

		print('Grid 1,3')
		self.ax13 = self.fig.add_subplot(333)

		print('Grid 2,1')
		self.ax21 = self.fig.add_subplot(336)
		self.ax21.get_xaxis().set_visible(False)
		self.ax21.get_yaxis().set_visible(False)

		print('Grid 2,2 - 3D Cube')
		self.ax22 = self.fig.add_subplot(335, projection='3d')

		print('Grid 2,3')
		self.ax23 = self.fig.add_subplot(334)
		self.ax23.get_xaxis().set_visible(False)
		self.ax23.get_yaxis().set_visible(False)

		print('Grid 3,1')
		self.ax31 = self.fig.add_subplot(338)
		self.ax31.get_xaxis().set_visible(False)
		self.ax31.get_yaxis().set_visible(False)

		print('Grid 3,2 - Speed bars')
		self.ax32 = self.fig.add_subplot(337)

		print('Grid 3,3 - PWM bars')
		self.ax33 = self.fig.add_subplot(339)

		##Empty history arrays
		self.t = []
		self.longitude = []
		self.latitude = []

		####WAYPOINT CONTROL WIDGETS (bottom strip)
		print('Creating waypoint controls...')
		##Text box for latitude input
		ax_lat_box = self.fig.add_axes([0.10, 0.03, 0.15, 0.04])
		self.tb_lat = mwidgets.TextBox(ax_lat_box, 'Target Lat: ', initial='')
		self.tb_lat.on_submit(self._on_lat_submit)

		##Text box for longitude input
		ax_lon_box = self.fig.add_axes([0.35, 0.03, 0.15, 0.04])
		self.tb_lon = mwidgets.TextBox(ax_lon_box, 'Target Lon: ', initial='')
		self.tb_lon.on_submit(self._on_lon_submit)

		##Send waypoint button
		ax_btn = self.fig.add_axes([0.55, 0.02, 0.12, 0.06])
		self.btn_send = mwidgets.Button(ax_btn, 'Send Waypoint', color='lightblue', hovercolor='deepskyblue')
		self.btn_send.on_clicked(self._on_send_waypoint)

		##Status label (updates after send)
		ax_status = self.fig.add_axes([0.70, 0.03, 0.25, 0.04])
		ax_status.get_xaxis().set_visible(False)
		ax_status.get_yaxis().set_visible(False)
		ax_status.set_xlim([0, 1])
		ax_status.set_ylim([0, 1])
		self._status_text = ax_status.text(0, 0.5, 'No waypoint sent yet.',
		                                    fontsize=10, va='center')
		self._ax_status = ax_status

		print('Window created.')

	# -------------------------------------------------------------------------
	# WAYPOINT CALLBACKS
	# -------------------------------------------------------------------------

	def _on_lat_submit(self, text):
		self._pending_lat = text.strip()

	def _on_lon_submit(self, text):
		self._pending_lon = text.strip()

	def _on_send_waypoint(self, event):
		"""Send waypoint to the boat via serial radio link."""
		lat_str = self._pending_lat or self.tb_lat.text.strip()
		lon_str = self._pending_lon or self.tb_lon.text.strip()

		if not lat_str or not lon_str:
			self._status_text.set_text('ERROR: Enter both Lat and Lon first.')
			self.fig.canvas.draw_idle()
			return

		try:
			wp_lat = float(lat_str)
			wp_lon = float(lon_str)
		except ValueError:
			self._status_text.set_text('ERROR: Invalid coordinates (use decimal degrees).')
			self.fig.canvas.draw_idle()
			return

		if not (-90 <= wp_lat <= 90) or not (-180 <= wp_lon <= 180):
			self._status_text.set_text('ERROR: Lat must be -90..90, Lon -180..180.')
			self.fig.canvas.draw_idle()
			return

		##Store as current target for display on map
		self.target_lat = wp_lat
		self.target_lon = wp_lon

		##Send over serial radio link: "W:LAT:LON\r"
		cmd = f'W:{wp_lat:.6f}:{wp_lon:.6f}\r'
		if ser.hComm is not None:
			try:
				ser.SerialPutString(cmd, echo=0)
				status_msg = f'Waypoint sent: {wp_lat:.5f}, {wp_lon:.5f}'
				print(f'[GND STATION] {status_msg}')
			except Exception as e:
				status_msg = f'Send ERROR: {e}'
				print(f'[GND STATION] {status_msg}')
		else:
			status_msg = f'[SIM] Waypoint: {wp_lat:.5f}, {wp_lon:.5f} (no serial)'
			print(f'[GND STATION] {status_msg}')

		self._status_text.set_text(status_msg)
		self.fig.canvas.draw_idle()

	# -------------------------------------------------------------------------
	# ATTITUDE VISUALIZATION
	# -------------------------------------------------------------------------

	def R123(self, phi, theta, psi):
		ctheta = np.cos(theta)
		stheta = np.sin(theta)
		sphi = np.sin(phi)
		cphi = np.cos(phi)
		spsi = np.sin(psi)
		cpsi = np.cos(psi)
		R = np.array([
			[ctheta*cpsi, sphi*stheta*cpsi-cphi*spsi, cphi*stheta*cpsi+sphi*spsi],
			[ctheta*spsi, sphi*stheta*spsi+cphi*cpsi, cphi*stheta*spsi-sphi*cpsi],
			[-stheta,     sphi*ctheta,                 cphi*ctheta]
		])
		return R

	def CubeDraw(self, ax, phi, theta, psi):
		dx, dy, dz = 1.0, 0.5, 0.25
		xv = np.array([[-1,1,1,-1],[-1,1,1,-1],[1,1,1,1],[-1,-1,-1,-1],[1,1,-1,-1],[1,1,-1,-1]])*dx/2.0
		yv = np.array([[1,1,-1,-1],[1,1,-1,-1],[-1,-1,1,1],[-1,-1,1,1],[1,1,1,1],[-1,-1,-1,-1]])*dy/2.0
		zv = np.array([[-1,-1,-1,-1],[1,1,1,1],[-1,1,1,-1],[-1,1,1,-1],[1,-1,-1,1],[1,-1,-1,1]])*dz/2.0
		T = self.R123(phi, theta, psi)
		xc, yc, zc = 0.5, 0.5, 0.5
		rc = np.array([xc, yc, zc])
		for jj in range(6):
			for ii in range(4):
				xyz = np.array([xv[jj][ii], yv[jj][ii], zv[jj][ii]])
				xyz_trans = rc + np.matmul(T, xyz)
				xv[jj][ii] = xyz_trans[0]
				yv[jj][ii] = xyz_trans[1]
				zv[jj][ii] = xyz_trans[2]
			pc = Poly3DCollection([list(zip(xv[jj], yv[jj], zv[jj]))], alpha=0.5)
			pc.set_facecolor('r')
			pc.set_edgecolor('k')
			ax.add_collection(pc)
		ax.get_xaxis().set_visible(False)
		ax.get_yaxis().set_visible(False)
		ax.get_zaxis().set_visible(False)

	# -------------------------------------------------------------------------
	# DATA INGESTION
	# -------------------------------------------------------------------------

	def sendNewData(self, gndstation_packet):
		time_val   = gndstation_packet[0]
		self.t.append(time_val)
		self.roll  = gndstation_packet[1]
		self.pitch = gndstation_packet[2]
		self.yaw   = gndstation_packet[3]
		latitude   = gndstation_packet[4]
		longitude  = gndstation_packet[5]
		if latitude > 30 and latitude < 40 and longitude < -80 and longitude > -90:
			self.latitude.append(latitude)
			self.longitude.append(longitude)
		self.baro_altitude = gndstation_packet[6]
		self.gps_speed     = gndstation_packet[7]
		self.gps_altitude  = gndstation_packet[8]
		self.pitot_speed   = gndstation_packet[9]
		self.throttle      = gndstation_packet[10]
		self.aileron       = gndstation_packet[11]
		self.elevator      = gndstation_packet[12]
		self.rudder        = gndstation_packet[13]

	# -------------------------------------------------------------------------
	# DISPLAY
	# -------------------------------------------------------------------------

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

		##GRID 1,1 — GPS READOUT
		self.ax11.set_title('GPS Readout', fontsize=10, fontweight='bold')
		try:
			cur_lat = self.latitude[-1]
			cur_lon = self.longitude[-1]
			self.ax11.text(0, 0.92, f'Latitude  = {cur_lat:.6f} °', fontsize=9)
			self.ax11.text(0, 0.74, f'Longitude = {cur_lon:.6f} °', fontsize=9)
		except IndexError:
			self.ax11.text(0, 0.92, 'Latitude  = (no fix)', fontsize=9)
			self.ax11.text(0, 0.74, 'Longitude = (no fix)', fontsize=9)
			cur_lat, cur_lon = None, None
		self.ax11.text(0, 0.56, f'GPS Speed = {self.gps_speed:.2f} m/s', fontsize=9)
		self.ax11.text(0, 0.38, f'Baro Alt  = {self.baro_altitude:.1f} m', fontsize=9)
		##Show distance/bearing to waypoint if one is active
		if self.target_lat is not None and cur_lat is not None:
			dist = _gps_distance(cur_lat, cur_lon, self.target_lat, self.target_lon)
			brng = _gps_bearing(cur_lat, cur_lon, self.target_lat, self.target_lon)
			self.ax11.text(0, 0.20,
			               f'WP Dist   = {dist:.1f} m', fontsize=9, color='blue')
			self.ax11.text(0, 0.02,
			               f'WP Bearing= {brng:.1f} °', fontsize=9, color='blue')
		elif self.target_lat is not None:
			self.ax11.text(0, 0.20,
			               f'WP: {self.target_lat:.5f}, {self.target_lon:.5f}',
			               fontsize=9, color='blue')

		##GRID 1,2 — LAT/LON MAP
		self.ax12.plot(self.longitude, self.latitude, 'b-', linewidth=1, label='Track')
		try:
			##Current position marker
			self.ax12.plot(self.longitude[-1], self.latitude[-1],
			               'go', markersize=8, label='Current')
		except IndexError:
			pass
		##Waypoint marker
		if self.target_lat is not None:
			self.ax12.plot(self.target_lon, self.target_lat,
			               'r*', markersize=14, label='Waypoint')
		self.ax12.set_xlabel('Longitude (deg)')
		self.ax12.set_ylabel('Latitude (deg)')
		self.ax12.legend(fontsize=7, loc='best')
		self.ax12.grid()

		##GRID 1,3 — Altitude
		self.ax13.plot([-10,10], [self.gps_altitude, self.gps_altitude],
		               'b-', label='GPS Alt')
		self.ax13.plot([-10,10], [self.baro_altitude, self.baro_altitude],
		               'r-', label='Baro Alt')
		comm_altitude = 50.0
		self.ax13.plot([-10,10], [comm_altitude, comm_altitude], 'g--', label='Command')
		self.ax13.legend()
		self.ax13.set_ylabel('Altitude (m)')
		self.ax13.grid()
		self.ax13.set_ylim([0, 100])

		##GRID 2,1 — Attitude text
		self.ax21.text(0, 0.9,  f'Roll  (deg) = {self.roll:.2f}')
		self.ax21.text(0, 0.45, f'Pitch (deg) = {self.pitch:.2f}')
		self.ax21.text(0, 0,    f'Yaw   (deg) = {self.yaw:.2f}')

		##GRID 2,2 — 3D Cube attitude viz
		self.CubeDraw(self.ax22,
		              self.roll  * np.pi/180.,
		              self.pitch * np.pi/180.,
		              self.yaw   * np.pi/180.)

		##GRID 2,3 — Autopilot status
		self.ax23.set_title('Autopilot', fontsize=10, fontweight='bold')
		self.ax23.text(0, 0.75, f'Time (sec) = {self.t[-1]:.1f}', fontsize=9)
		if self.target_lat is not None:
			self.ax23.text(0, 0.50,
			               f'Target Lat = {self.target_lat:.6f}', fontsize=9, color='blue')
			self.ax23.text(0, 0.25,
			               f'Target Lon = {self.target_lon:.6f}', fontsize=9, color='blue')
		else:
			self.ax23.text(0, 0.50, 'No waypoint active', fontsize=9, color='grey')

		##GRID 3,1 — Speed text
		self.ax31.text(0, 0.8,  f'GPS Speed (m/s)   = {self.gps_speed:.2f}')
		self.ax31.text(0, 0.4,  f'Pitot Speed (m/s) = {self.pitot_speed:.2f}')

		##GRID 3,2 — Speed bars
		self.ax32.plot([-10,10], [self.gps_speed,   self.gps_speed],   'b-', label='GPS')
		self.ax32.plot([-10,10], [self.pitot_speed, self.pitot_speed], 'r-', label='Pitot')
		comm_speed = 15.0
		self.ax32.plot([-10,10], [comm_speed, comm_speed], 'g--', label='Command')
		self.ax32.set_ylabel('Speed (m/s)')
		self.ax32.set_ylim([0, 30])
		self.ax32.grid()
		self.ax32.legend()

		##GRID 3,3 — PWM bars
		width = 10
		self.ax33.add_patch(pt.Rectangle([0,        0], width, self.throttle, fc='b'))
		self.ax33.add_patch(pt.Rectangle([width,    0], width, self.aileron,  fc='r'))
		self.ax33.add_patch(pt.Rectangle([2*width,  0], width, self.elevator, fc='g'))
		self.ax33.add_patch(pt.Rectangle([3*width,  0], width, self.rudder,   fc='m'))
		self.ax33.grid()
		self.ax33.set_ylabel('PWM (us)')
		self.ax33.set_xlabel('Throttle   Aileron   Elevator  Rudder')
		self.ax33.set_xlim([0, 40])
		self.ax33.set_ylim([0, 2400])


##CREATE EMPTY ARRAY PACKETS
gndstation_packet = np.zeros(14)

##Initialize ground station log file
outfilename = ('logs/Ground_Station_'
               + datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
               + '.csv')
outfile = open(outfilename, 'w')

#Create Empty serial object
ser = U()

if SERIAL == 2:
	print('Opening Serial port')
	print('All available serial ports...')
	os.system('ls /dev/ttyUSB*')
	ser.SerialInit(57600, "/dev/ttyUSB0", period=1.0)
	print('Serial initialization done')

##If GUI is on create the window
if GUI:
	print('Creating Window')
	GND = WINDOW()
	print('Window Created')

##MAIN LOOP
loop_counter = 0
filenumber = 0
NEW_DATA = False
while True:
	loop_counter += 1

	##READ DATA DEPENDING ON SERIAL FLAG
	if SERIAL == 0:
		NEW_DATA = True
		gndstation_packet = get_fake_data(loop_counter)
	elif SERIAL == 1:
		filenumberNEW = maxFile(filenumber)
		print('Fast SIL at = ', filenumberNEW, ' Python Time = ', time.monotonic())
		if filenumberNEW != filenumber:
			filenumber = filenumberNEW
			if filenumber > 0:
				print('Reading File = ', filenumber-1)
				ser.fast_packet = readFile(getFileName(filenumber-1))
				gndstation_packet, NEW_DATA = updatePacket(ser.fast_packet[0], 0)
	elif SERIAL == 2:
		position = -1
		print('Reading Serial....', time.monotonic())
		value, position, bytestring = ser.SerialGetNumber(1)
		print('Value Received, Position, Bytes = ', value, position, bytestring)
		gndstation_packet, NEW_DATA = updatePacket(value, position)
		if position < 7:
			NEW_DATA = False

	##Update GUI if we have new data
	if GUI == 1 and NEW_DATA == True:
		print("Updating GUI.....")
		GND.clearwindow()
		GND.sendNewData(gndstation_packet)
		GND.updatewindow()
		plt.pause(0.0000000000001)

	##Write data to log file
	if NEW_DATA:
		outstr = ''
		for i in gndstation_packet:
			outstr += (str(i) + ' ')
		print('Packets Received (time) = ', gndstation_packet[0])
		outfile.write(outstr)
		outfile.write('\n')
		NEW_DATA = False
