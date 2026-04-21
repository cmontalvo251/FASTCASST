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
import threading
try:
	import requests
	REQUESTS_AVAILABLE = True
except ImportError:
	REQUESTS_AVAILABLE = False
	print('WARNING: requests not installed — ground station location disabled. Run: pip install requests')
sys.path.append('../libraries/')
from Comms.Comms import Comms as U

try:
	import contextily as cx
	CONTEXTILY_AVAILABLE = True
except ImportError:
	CONTEXTILY_AVAILABLE = False
	print('WARNING: contextily not installed — map tiles disabled. Run: pip install contextily')

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
	lat = 32.69 + 0.001 * np.sin(t * 0.2)
	lon = -88.1 + 0.001 * np.cos(t * 0.2)
	baro_pressure = 1013 - 3.0*(np.sin(t)+1)
	altitude = ConvertPressure2Alt(baro_pressure)
	roll    = 5.0 * np.sin(t * 0.5)
	pitch   = 3.0 * np.sin(t * 0.3)
	compass = (180.0 + 45.0 * np.sin(t * 0.1)) % 360.0
	gps_speed = 2.0 + 1.0 * np.sin(t * 0.4)
	motor1  = 1500. + 400 * np.sin(t)
	motor2  = 1500. + 400 * np.sin(t)
	rudder  = 1500. + 400 * np.cos(t)
	fix_quality = 3.0
	gndstation_packet[0]  = t
	gndstation_packet[1]  = roll
	gndstation_packet[2]  = pitch
	gndstation_packet[3]  = compass
	gndstation_packet[4]  = lat
	gndstation_packet[5]  = lon
	gndstation_packet[6]  = altitude
	gndstation_packet[7]  = gps_speed
	gndstation_packet[8]  = 0.0          # unused
	gndstation_packet[9]  = 0.0          # unused
	gndstation_packet[10] = motor1
	gndstation_packet[11] = motor2
	gndstation_packet[12] = rudder
	gndstation_packet[13] = fix_quality
	return gndstation_packet

def updatePacket(value,position):
	if position >= 0:
		ser.fast_packet[position] = value
		gndstation_packet[0]  = ser.fast_packet[0]  #time
		gndstation_packet[1]  = ser.fast_packet[1]  #roll
		gndstation_packet[2]  = ser.fast_packet[2]  #pitch
		gndstation_packet[3]  = ser.fast_packet[3]  #compass heading
		gndstation_packet[4]  = ser.fast_packet[4]  #lat
		gndstation_packet[5]  = ser.fast_packet[5]  #lon
		gndstation_packet[6]  = ser.fast_packet[6]  #baro altitude
		gndstation_packet[7]  = ser.fast_packet[7]  #gps speed (m/s)
		gndstation_packet[8]  = 0.0                 # unused
		gndstation_packet[9]  = 0.0                 # unused
		gndstation_packet[10] = ser.fast_packet[8]  # motor 1
		gndstation_packet[11] = ser.fast_packet[9]  # motor 2
		gndstation_packet[12] = ser.fast_packet[10] # rudder
		gndstation_packet[13] = ser.fast_packet[11] # gps fix quality
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

		##Leave room at the bottom for mission planner controls
		self.fig.subplots_adjust(bottom=0.17)

		##Mission planner state
		self._mission_waypoints = []   # list of (lat, lon) added by operator
		self.target_lat = None         # first waypoint (for map/readout display)
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
		self.t = [0.0]
		self.longitude = []
		self.latitude = []
		self.speed_history    = []
		self.time_history     = []
		self.gps_heading      = None
		self._last_map_extent = None   # cached basemap extent — only refetch when it changes
		self.gs_lat           = None   # ground station location (fetched via IP geolocation)
		self.gs_lon           = None
		self._boat_waypoints  = []     # waypoints currently loaded on the boat (received via radio)
		self._last_draw_time  = 0.0    # throttle redraws when no new data

		##Default display values — shown before first telemetry packet arrives
		self.roll         = 0.0
		self.pitch        = 0.0
		self.heading      = 0.0
		self.baro_altitude = 0.0
		self.gps_speed    = 0.0
		self.fix_quality  = 0
		self.motor1       = 1504.0
		self.motor2       = 1504.0
		self.rudder       = 1504.0

		self._fetch_gs_location()

		####MISSION PLANNER WIDGETS (bottom two rows)
		print('Creating mission planner controls...')

		##Row 1 (y~0.10): coordinate entry + Add WP button
		ax_lat_box = self.fig.add_axes([0.05, 0.10, 0.13, 0.04])
		self.tb_lat = mwidgets.TextBox(ax_lat_box, 'Lat: ', initial='')
		self.tb_lat.on_submit(self._on_lat_submit)

		ax_lon_box = self.fig.add_axes([0.27, 0.10, 0.13, 0.04])
		self.tb_lon = mwidgets.TextBox(ax_lon_box, 'Lon: ', initial='')
		self.tb_lon.on_submit(self._on_lon_submit)

		ax_btn_add = self.fig.add_axes([0.44, 0.09, 0.10, 0.06])
		self.btn_add = mwidgets.Button(ax_btn_add, 'Add WP',
		                               color='lightgreen', hovercolor='mediumseagreen')
		self.btn_add.on_clicked(self._on_add_waypoint)

		##Row 2 (y~0.02): mission control buttons
		ax_btn_remove = self.fig.add_axes([0.05, 0.02, 0.10, 0.06])
		self.btn_remove = mwidgets.Button(ax_btn_remove, 'Remove\nLast',
		                                  color='lightyellow', hovercolor='gold')
		self.btn_remove.on_clicked(self._on_remove_last)

		ax_btn_send = self.fig.add_axes([0.20, 0.02, 0.12, 0.06])
		self.btn_send = mwidgets.Button(ax_btn_send, 'Send Mission',
		                                color='lightblue', hovercolor='deepskyblue')
		self.btn_send.on_clicked(self._on_send_mission)

		ax_btn_clear = self.fig.add_axes([0.37, 0.02, 0.12, 0.06])
		self.btn_clear = mwidgets.Button(ax_btn_clear, 'Clear Mission',
		                                 color='lightsalmon', hovercolor='tomato')
		self.btn_clear.on_clicked(self._on_clear_mission)

		##Status / mission list display
		ax_status = self.fig.add_axes([0.55, 0.01, 0.43, 0.14])
		ax_status.get_xaxis().set_visible(False)
		ax_status.get_yaxis().set_visible(False)
		ax_status.set_xlim([0, 1])
		ax_status.set_ylim([0, 1])
		self._status_text = ax_status.text(0.01, 0.85, 'No mission loaded.',
		                                    fontsize=8, va='top')
		self._ax_status = ax_status

		print('Window created.')

	# -------------------------------------------------------------------------
	# GROUND STATION LOCATION
	# -------------------------------------------------------------------------

	def _fetch_gs_location(self):
		"""Fetch ground station lat/lon via IP geolocation in a background thread."""
		if not REQUESTS_AVAILABLE:
			return
		def _fetch():
			try:
				r = requests.get('http://ip-api.com/json/', timeout=5)
				data = r.json()
				if data.get('status') == 'success':
					self.gs_lat = data['lat']
					self.gs_lon = data['lon']
					print(f'[GND STATION] Location acquired: {self.gs_lat:.5f}, {self.gs_lon:.5f}')
				else:
					print('[GND STATION] IP geolocation returned no result.')
			except Exception as e:
				print(f'[GND STATION] Could not fetch location: {e}')
		threading.Thread(target=_fetch, daemon=True).start()

	# -------------------------------------------------------------------------
	# WAYPOINT CALLBACKS
	# -------------------------------------------------------------------------

	def _on_lat_submit(self, text):
		self._pending_lat = text.strip()

	def _on_lon_submit(self, text):
		self._pending_lon = text.strip()

	def _on_add_waypoint(self, event):
		"""Validate and add a coordinate to the local mission list."""
		lat_str = self._pending_lat or self.tb_lat.text.strip()
		lon_str = self._pending_lon or self.tb_lon.text.strip()

		if not lat_str or not lon_str:
			self._update_status('ERROR: Enter both Lat and Lon first.')
			return

		try:
			wp_lat = float(lat_str)
			wp_lon = float(lon_str)
		except ValueError:
			self._update_status('ERROR: Invalid coordinates (use decimal degrees).')
			return

		if not (-90 <= wp_lat <= 90) or not (-180 <= wp_lon <= 180):
			self._update_status('ERROR: Lat must be -90..90, Lon -180..180.')
			return

		self._mission_waypoints.append((wp_lat, wp_lon))
		##Track first WP as the display target
		if len(self._mission_waypoints) == 1:
			self.target_lat = wp_lat
			self.target_lon = wp_lon

		##Clear entry boxes for next waypoint
		self._pending_lat = ''
		self._pending_lon = ''
		self.tb_lat.set_val('')
		self.tb_lon.set_val('')

		self._update_status(self._mission_summary())
		self.fig.canvas.draw_idle()

	def _on_remove_last(self, event):
		"""Remove the most recently added waypoint from the local list."""
		if self._mission_waypoints:
			removed = self._mission_waypoints.pop()
			##Keep display target in sync
			if self._mission_waypoints:
				self.target_lat = self._mission_waypoints[0][0]
				self.target_lon = self._mission_waypoints[0][1]
			else:
				self.target_lat = None
				self.target_lon = None
			print(f'[GND STATION] Removed WP: {removed[0]:.6f}, {removed[1]:.6f}')
		self._update_status(self._mission_summary())
		self.fig.canvas.draw_idle()

	def _on_send_mission(self, event):
		"""Send the full ordered waypoint list to the vehicle."""
		if not self._mission_waypoints:
			self._update_status('ERROR: Add at least one waypoint before sending.')
			self.fig.canvas.draw_idle()
			return

		##Build: MISSION:lat1:lon1:lat2:lon2:...\r
		parts = ['MISSION']
		for lat, lon in self._mission_waypoints:
			parts.append(f'{lat:.6f}')
			parts.append(f'{lon:.6f}')
		cmd = ':'.join(parts) + '\r'

		n = len(self._mission_waypoints)
		if ser.hComm is not None:
			try:
				ser.SerialPutString(cmd, echo=0)
				status_msg = (f'Mission sent: {n} WP(s) + return to start.\n'
				              + self._mission_summary())
				print(f'[GND STATION] Mission sent ({n} waypoints).')
			except Exception as e:
				status_msg = f'Send ERROR: {e}'
				print(f'[GND STATION] {status_msg}')
		else:
			status_msg = (f'[SIM] Mission queued: {n} WP(s) + return to start.\n'
			              + self._mission_summary())
			print(f'[GND STATION] {status_msg}')

		self._update_status(status_msg)
		self.fig.canvas.draw_idle()

	def _on_clear_mission(self, event):
		"""Clear the local mission list and send CLEAR to the vehicle."""
		self._mission_waypoints = []
		self.target_lat = None
		self.target_lon = None
		cmd = 'CLEAR\r'
		if ser.hComm is not None:
			try:
				ser.SerialPutString(cmd, echo=0)
			except Exception as e:
				print(f'[GND STATION] Clear send error: {e}')
		self._update_status('Mission cleared.')
		self.fig.canvas.draw_idle()

	def _mission_summary(self):
		"""Return a compact text summary of the current mission list."""
		if not self._mission_waypoints:
			return 'No mission loaded.'
		lines = [f'Mission ({len(self._mission_waypoints)} WP + return to start):']
		for i, (la, lo) in enumerate(self._mission_waypoints, 1):
			lines.append(f'  WP{i}: {la:.5f}, {lo:.5f}')
		lines.append('  --> Return to start')
		return '\n'.join(lines)

	def _update_status(self, msg):
		self._status_text.set_text(msg)
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

	def CompassDraw(self, ax, heading_deg):
		ax.set_xlim(-1.4, 1.4)
		ax.set_ylim(-1.4, 1.4)
		ax.set_aspect('equal')
		ax.axis('off')
		ax.set_facecolor('#f0f4f8')

		##Outer ring
		theta = np.linspace(0, 2*np.pi, 200)
		ax.fill(np.cos(theta), np.sin(theta), color='white', zorder=1)
		ax.plot(np.cos(theta), np.sin(theta), 'k-', linewidth=2, zorder=2)

		##Tick marks and cardinal labels
		cardinals = {0: 'N', 90: 'E', 180: 'S', 270: 'W'}
		for deg in range(0, 360, 5):
			rad = np.radians(90.0 - deg)
			if deg % 90 == 0:
				r0, lw = 0.78, 2.5
			elif deg % 45 == 0:
				r0, lw = 0.82, 1.5
			else:
				r0, lw = 0.88, 0.8
			ax.plot([r0*np.cos(rad), np.cos(rad)],
			        [r0*np.sin(rad), np.sin(rad)],
			        'k-', linewidth=lw, zorder=3)
			if deg in cardinals:
				ax.text(1.18*np.cos(rad), 1.18*np.sin(rad), cardinals[deg],
				        ha='center', va='center', fontsize=11,
				        fontweight='bold', zorder=4)

		##Degree labels at 30° intervals (skip cardinals)
		for deg in range(30, 360, 30):
			if deg % 90 != 0:
				rad = np.radians(90.0 - deg)
				ax.text(1.30*np.cos(rad), 1.30*np.sin(rad), str(deg),
				        ha='center', va='center', fontsize=6, color='#555555', zorder=4)

		##Heading needle — red forward, white back
		h_rad = np.radians(90.0 - heading_deg)
		ax.annotate('', xy=(0.72*np.cos(h_rad), 0.72*np.sin(h_rad)),
		            xytext=(-0.30*np.cos(h_rad), -0.30*np.sin(h_rad)),
		            arrowprops=dict(arrowstyle='->', color='red',
		                            lw=3, mutation_scale=20), zorder=5)
		ax.plot(-0.30*np.cos(h_rad), -0.30*np.sin(h_rad),
		        'o', color='white', markersize=8, markeredgecolor='k',
		        markeredgewidth=1, zorder=6)
		ax.plot(0, 0, 'ko', markersize=5, zorder=7)

		ax.set_title('Heading', fontsize=10, fontweight='bold')

	# -------------------------------------------------------------------------
	# DATA INGESTION
	# -------------------------------------------------------------------------

	def sendNewData(self, gndstation_packet):
		time_val        = gndstation_packet[0]
		self.t.append(time_val)
		self.roll       = gndstation_packet[1]
		self.pitch      = gndstation_packet[2]
		self.heading    = gndstation_packet[3]
		latitude        = gndstation_packet[4]
		longitude       = gndstation_packet[5]
		if (latitude != 0.0 and longitude != 0.0
		        and abs(latitude) != 99 and abs(longitude) != 99):
			self.latitude.append(latitude)
			self.longitude.append(longitude)
		##GPS course-over-ground: bearing between the last two valid GPS positions
		if len(self.latitude) >= 2:
			self.gps_heading = _gps_bearing(
				self.latitude[-2], self.longitude[-2],
				self.latitude[-1], self.longitude[-1]
			)
		else:
			self.gps_heading = None
		self.baro_altitude  = gndstation_packet[6]
		self.gps_speed      = gndstation_packet[7]
		##Telemetry sends motor/rudder as [-1, 1]. Convert to µs for display.
		##Inverse of rcio convert(): µs = (val * half_range + mid) * 1000
		_mid  = 1.504
		_half = (2.010 - 0.995) / 2.0   # 0.5075 ms
		self.motor1      = (gndstation_packet[10] * _half + _mid) * 1000.0
		self.motor2      = (gndstation_packet[11] * _half + _mid) * 1000.0
		self.rudder      = (gndstation_packet[12] * _half + _mid) * 1000.0
		self.fix_quality = int(gndstation_packet[13])

		##Rolling 90-second speed history
		self.speed_history.append(self.gps_speed)
		self.time_history.append(time_val)
		if len(self.time_history) > 1:
			cutoff = self.time_history[-1] - 90.0
			while len(self.time_history) > 1 and self.time_history[0] < cutoff:
				self.time_history.pop(0)
				self.speed_history.pop(0)

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

		##GRID 1,1 — GPS / STATUS READOUT
		self.ax11.set_title('Navigation', fontsize=10, fontweight='bold')
		self.ax11.set_xlim([0, 1])
		self.ax11.set_ylim([0, 1])
		fix_labels = {0: 'NO FIX', 1: 'GPS', 2: 'DGPS', 3: 'PPS', 4: 'RTK', 5: 'Float RTK'}
		fix_colors = {0: 'red', 1: 'orange', 2: 'yellowgreen', 3: 'green', 4: 'green', 5: 'yellowgreen'}
		fix_str   = fix_labels.get(self.fix_quality, str(self.fix_quality))
		fix_color = fix_colors.get(self.fix_quality, 'grey')
		try:
			cur_lat = self.latitude[-1]
			cur_lon = self.longitude[-1]
			self.ax11.text(0.02, 0.92, f'Lat       = {cur_lat:.6f} °', fontsize=9)
			self.ax11.text(0.02, 0.78, f'Lon       = {cur_lon:.6f} °', fontsize=9)
		except IndexError:
			self.ax11.text(0.02, 0.92, 'Lat       = (no fix)', fontsize=9, color='red')
			self.ax11.text(0.02, 0.78, 'Lon       = (no fix)', fontsize=9, color='red')
			cur_lat, cur_lon = None, None
		self.ax11.text(0.02, 0.64, f'Speed     = {self.gps_speed:.2f} m/s  ({self.gps_speed*1.944:.1f} kts)', fontsize=9)
		self.ax11.text(0.02, 0.50, f'Heading   = {self.heading:.1f} °', fontsize=9)
		self.ax11.text(0.02, 0.36, f'Baro Alt  = {self.baro_altitude:.1f} m', fontsize=9)
		self.ax11.text(0.02, 0.22, f'GPS Fix   = ', fontsize=9)
		self.ax11.text(0.30, 0.22, fix_str, fontsize=9, color=fix_color, fontweight='bold')
		##Distance/bearing to next waypoint
		if self._mission_waypoints and cur_lat is not None:
			wp1_lat, wp1_lon = self._mission_waypoints[0]
			dist = _gps_distance(cur_lat, cur_lon, wp1_lat, wp1_lon)
			brng = _gps_bearing(cur_lat, cur_lon, wp1_lat, wp1_lon)
			self.ax11.text(0.02, 0.08,
			               f'WP1  {dist:.0f} m  /  {brng:.1f} °',
			               fontsize=9, color='blue', fontweight='bold')
		elif self.target_lat is not None and cur_lat is not None:
			dist = _gps_distance(cur_lat, cur_lon, self.target_lat, self.target_lon)
			brng = _gps_bearing(cur_lat, cur_lon, self.target_lat, self.target_lon)
			self.ax11.text(0.02, 0.08,
			               f'WP   {dist:.0f} m  /  {brng:.1f} °',
			               fontsize=9, color='blue', fontweight='bold')

		##GRID 1,2 — MAP
		self.ax12.set_title('Map', fontsize=10, fontweight='bold')
		if len(self.latitude) >= 1:
			##Set map extent with padding around the full track
			lat_min, lat_max = min(self.latitude), max(self.latitude)
			lon_min, lon_max = min(self.longitude), max(self.longitude)
			lat_pad = max(0.0008, (lat_max - lat_min) * 0.25)
			lon_pad = max(0.0008, (lon_max - lon_min) * 0.25)
			self.ax12.set_xlim([lon_min - lon_pad, lon_max + lon_pad])
			self.ax12.set_ylim([lat_min - lat_pad, lat_max + lat_pad])

			##Track line + current position marker
			self.ax12.plot(self.longitude, self.latitude,
			               'b-', linewidth=2, zorder=5, label='Track')
			self.ax12.plot(self.longitude[-1], self.latitude[-1],
			               'go', markersize=10, zorder=6, label='Boat')

			##Ground station location
			if self.gs_lat is not None and self.gs_lon is not None:
				self.ax12.plot(self.gs_lon, self.gs_lat,
				               's', color='orange', markersize=10, zorder=6,
				               label='Ground Station')
				self.ax12.annotate('GS', xy=(self.gs_lon, self.gs_lat),
				                   fontsize=7, color='orange', fontweight='bold',
				                   xytext=(4, 4), textcoords='offset points', zorder=7)

			##Mission waypoints + route
			if self._mission_waypoints:
				wp_lats = [la for la, lo in self._mission_waypoints]
				wp_lons = [lo for la, lo in self._mission_waypoints]
				route_lats = wp_lats + [wp_lats[0]]
				route_lons = wp_lons + [wp_lons[0]]
				self.ax12.plot(route_lons, route_lats, 'r--',
				               linewidth=1.5, alpha=0.8, zorder=4, label='Mission')
				for i, (la, lo) in enumerate(self._mission_waypoints, 1):
					self.ax12.plot(lo, la, 'r*', markersize=14, zorder=7)
					self.ax12.annotate(f'WP{i}', xy=(lo, la), fontsize=8,
					                   color='red', fontweight='bold',
					                   xytext=(4, 4), textcoords='offset points',
					                   zorder=8)

			##Boat's active waypoints (received via 2-way radio link)
			if self._boat_waypoints:
				bwp_lats = [la for la, lo in self._boat_waypoints]
				bwp_lons = [lo for la, lo in self._boat_waypoints]
				self.ax12.plot(bwp_lons, bwp_lats, 'c^', markersize=10, zorder=8, label='Boat WPs')
				for i, (bla, blo) in enumerate(self._boat_waypoints, 1):
					self.ax12.annotate(f'B{i}', xy=(blo, bla), fontsize=8,
					                   color='darkcyan', fontweight='bold',
					                   xytext=(4, -10), textcoords='offset points',
					                   zorder=9)

			##OSM basemap tiles — only re-fetch when the map extent shifts enough
			##to need new tiles (avoids blocking the GUI every second).
			new_extent = (round(lon_min - lon_pad, 4), round(lon_max + lon_pad, 4),
			              round(lat_min - lat_pad, 4), round(lat_max + lat_pad, 4))
			extent_changed = (self._last_map_extent != new_extent)
			if CONTEXTILY_AVAILABLE and extent_changed and (lon_max - lon_min > 0 or lat_max - lat_min > 0):
				try:
					cx.add_basemap(self.ax12, crs='EPSG:4326',
					               source=cx.providers.OpenStreetMap.Mapnik,
					               zoom='auto', attribution=False)
					self._last_map_extent = new_extent
				except Exception as e:
					self.ax12.grid()
					self.ax12.text(0.01, 0.01, f'Map tiles unavailable: {e}',
					               fontsize=6, transform=self.ax12.transAxes,
					               color='red')
			elif not CONTEXTILY_AVAILABLE:
				self.ax12.grid()

			self.ax12.set_xlabel('Longitude')
			self.ax12.set_ylabel('Latitude')
			self.ax12.legend(fontsize=7, loc='best')
		else:
			self.ax12.text(0.5, 0.5, 'Waiting for GPS fix...',
			               ha='center', va='center', fontsize=10,
			               transform=self.ax12.transAxes)

		##GRID 1,3 — Compass dial
		self.CompassDraw(self.ax13, self.heading)

		##GRID 2,1 — Attitude text
		self.ax21.set_title('Attitude', fontsize=10, fontweight='bold')
		self.ax21.set_xlim([0, 1])
		self.ax21.set_ylim([0, 1])
		roll_color  = 'red' if abs(self.roll)  > 20 else 'black'
		pitch_color = 'red' if abs(self.pitch) > 15 else 'black'
		self.ax21.text(0.05, 0.72, f'Roll',    fontsize=10, color='grey')
		self.ax21.text(0.05, 0.52, f'Pitch',   fontsize=10, color='grey')
		self.ax21.text(0.05, 0.32, f'Heading', fontsize=10, color='grey')
		self.ax21.text(0.45, 0.72, f'{self.roll:+.1f} °',    fontsize=12, fontweight='bold', color=roll_color)
		self.ax21.text(0.45, 0.52, f'{self.pitch:+.1f} °',   fontsize=12, fontweight='bold', color=pitch_color)
		self.ax21.text(0.45, 0.32, f'{self.heading:.1f} °',  fontsize=12, fontweight='bold')

		##GRID 2,2 — 3D Cube attitude viz
		self.CubeDraw(self.ax22,
		              self.roll  * np.pi/180.,
		              self.pitch * np.pi/180.,
		              self.heading * np.pi/180.)

		##GRID 2,3 — Autopilot status
		self.ax23.set_title('Autopilot', fontsize=10, fontweight='bold')
		self.ax23.set_xlim([0, 1])
		self.ax23.set_ylim([0, 1])
		self.ax23.text(0, 0.95, f'Time (sec) = {self.t[-1]:.1f}', fontsize=9)

		##Top half: ground station pending mission
		n_wps = len(self._mission_waypoints)
		self.ax23.text(0, 0.84, 'Ground Mission:', fontsize=8, color='navy', fontweight='bold')
		if n_wps > 0:
			self.ax23.text(0, 0.74,
			               f'{n_wps} WP(s) + return to start', fontsize=8, color='blue')
			for i, (la, lo) in enumerate(self._mission_waypoints, 1):
				y_pos = 0.64 - (i - 1) * 0.10
				if y_pos > 0.50:
					self.ax23.text(0.02, y_pos,
					               f'WP{i}: {la:.5f}, {lo:.5f}',
					               fontsize=7, color='darkblue')
		else:
			self.ax23.text(0, 0.74, 'None queued.', fontsize=8, color='grey')

		##Divider
		self.ax23.plot([0, 1], [0.48, 0.48], color='#cccccc', linewidth=0.8)

		##Bottom half: boat's active waypoints (received over radio)
		n_bwps = len(self._boat_waypoints)
		self.ax23.text(0, 0.43, 'Boat Active WPs:', fontsize=8, color='darkcyan', fontweight='bold')
		if n_bwps > 0:
			self.ax23.text(0, 0.33,
			               f'{n_bwps} WP(s) loaded on boat', fontsize=8, color='teal')
			for i, (bla, blo) in enumerate(self._boat_waypoints, 1):
				y_pos = 0.23 - (i - 1) * 0.10
				if y_pos > -0.02:
					self.ax23.text(0.02, y_pos,
					               f'B{i}: {bla:.5f}, {blo:.5f}',
					               fontsize=7, color='darkcyan')
		else:
			self.ax23.text(0, 0.33, 'None reported yet.', fontsize=8, color='grey')

		##GRID 3,1 — Heading readout (compass + GPS course-over-ground)
		self.ax31.set_title('Heading', fontsize=10, fontweight='bold')
		self.ax31.set_xlim([0, 1])
		self.ax31.set_ylim([0, 1])
		self.ax31.text(0.05, 0.68, 'Compass',   fontsize=10, color='grey')
		self.ax31.text(0.05, 0.22, 'GPS Track', fontsize=10, color='grey')
		self.ax31.text(0.55, 0.68, f'{self.heading:.1f} °',
		               fontsize=16, fontweight='bold')
		if self.gps_heading is not None:
			self.ax31.text(0.55, 0.22, f'{self.gps_heading:.1f} °',
			               fontsize=16, fontweight='bold', color='steelblue')
		else:
			self.ax31.text(0.55, 0.22, 'no fix',
			               fontsize=12, color='#aaaaaa')
		##Divider between the two
		self.ax31.plot([0.03, 0.97], [0.48, 0.48], color='#cccccc', linewidth=0.8)

		##GRID 3,2 — Speed history
		self.ax32.set_title('Speed History (90 s)', fontsize=10, fontweight='bold')
		if len(self.speed_history) > 1:
			t0 = self.time_history[0]
			t_rel = [t - t0 for t in self.time_history]
			self.ax32.plot(t_rel, self.speed_history, 'b-', linewidth=2)
			self.ax32.fill_between(t_rel, self.speed_history, alpha=0.15, color='blue')
			peak = max(self.speed_history)
			self.ax32.set_ylim([0, max(1.0, peak * 1.3)])
		else:
			self.ax32.set_ylim([0, 5])
		self.ax32.set_ylabel('Speed (m/s)')
		self.ax32.set_xlabel('Time (s)')
		self.ax32.grid(True, alpha=0.4)

		##GRID 3,3 — PWM bars (Motor 1, Motor 2, Rudder)
		width = 10
		self.ax33.set_title('PWM Output', fontsize=10, fontweight='bold')
		self.ax33.add_patch(pt.Rectangle([0,       0], width, self.motor1, fc='steelblue'))
		self.ax33.add_patch(pt.Rectangle([width,   0], width, self.motor2, fc='steelblue'))
		self.ax33.add_patch(pt.Rectangle([2*width, 0], width, self.rudder, fc='darkorange'))
		##1500 µs neutral line
		self.ax33.axhline(1500, color='green', linestyle='--', linewidth=1, alpha=0.7)
		self.ax33.grid(True, axis='y', alpha=0.4)
		self.ax33.set_ylabel('PWM (µs)')
		self.ax33.set_xticks([width/2, width + width/2, 2*width + width/2])
		self.ax33.set_xticklabels(['Motor 1', 'Motor 2', 'Rudder'])
		self.ax33.set_xlim([0, 30])
		self.ax33.set_ylim([1000, 2000])


##CREATE EMPTY ARRAY PACKETS
gndstation_packet = np.zeros(14)

##Initialize ground station log file
outfilename = ('logs/Ground_Station_'
               + datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
               + '.csv')
outfile = open(outfilename, 'w')

#Create Empty serial object (12-slot packet: time,roll,pitch,compass,lat,lon,alt,speed,motor1,motor2,rudder,fix)
ser = U(packet_size=12)

if SERIAL == 2:
	print('Opening Serial port')
	import platform
	if platform.system() == 'Windows':
		GROUND_RADIO_PORT = 'COM3'
	else:
		GROUND_RADIO_PORT = '/dev/ttyUSB0'
	ser.SerialInit(57600, GROUND_RADIO_PORT, period=1.0)
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
		value, position, bytestring = ser.SerialGetNumber(0)
		gndstation_packet, NEW_DATA = updatePacket(value, position)
		if position < 11:
			NEW_DATA = False
		##Parse WPS status broadcast from the boat ("WPS:N:lat1:lon1:...\r" stripped to bytestring)
		if bytestring.startswith('WPS:'):
			try:
				parts = bytestring.split(':')
				count = int(parts[1])
				wps = []
				for i in range(count):
					wps.append((float(parts[2 + i*2]), float(parts[3 + i*2])))
				if GUI:
					GND._boat_waypoints = wps
				print(f'[GND] Boat WPs received: {wps}')
			except Exception as _e:
				print(f'[GND] WPS parse error: {_e}  raw={bytestring}')

	##Ingest new telemetry when available
	if GUI == 1 and NEW_DATA:
		GND.sendNewData(gndstation_packet)

	##Redraw at ~2 Hz regardless of data so the window always appears
	if GUI == 1:
		_now = time.monotonic()
		if _now - GND._last_draw_time >= 0.5:
			GND._last_draw_time = _now
			GND.clearwindow()
			GND.updatewindow()
			plt.pause(0.05)
		else:
			plt.pause(0.01)

	##Write data to log file
	if NEW_DATA:
		outstr = ''
		for i in gndstation_packet:
			outstr += (str(i) + ' ')
		print('Packets Received (time) = ', gndstation_packet[0])
		outfile.write(outstr)
		outfile.write('\n')
		NEW_DATA = False
