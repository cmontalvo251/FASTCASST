import sys
import time
import numpy as np

sys.path.append('../Util')
sys.path.append('../libraries/Util')
import util

##NOTE: This library supports NMEA GPS antennas connected via serial (USB or UART).
## Common setups:
##   USB GPS dongle    -> /dev/ttyUSB0
##   UART GPS (NEO-6M) -> /dev/ttyAMA1 or /dev/ttyS0  (NOT ttyAMA0, that's the radio)
## Baud rate is typically 9600 for most GPS modules.
## Install pyserial if needed: sudo pip3 install pyserial

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print('WARNING: pyserial not installed. Run: sudo pip3 install pyserial')


class GPS():
    def __init__(self, port='/dev/ttyUSB0', baud=9600):
        ##Set defaults
        self.latitude = -99
        self.longitude = -99
        self.altitude = -99
        self.speed = -99
        self.fix_quality = 0      # 0=no fix, 1=GPS, 2=DGPS
        self.num_satellites = 0
        self.has_fix = False

        ##For historical data / utility methods
        self.latitude_vec = []
        self.longitude_vec = []
        self.time_vec = []
        self.x_vec = []
        self.y_vec = []
        self.filterConstant = 0.2
        self.NM2FT = 6076.115485560000
        self.FT2M = 0.3048
        self.GPSVAL = 60.0 * self.NM2FT * self.FT2M
        self.latO = 33.16
        self.lonO = -88.1

        ##Serial connection
        self.port = port
        self.baud = baud
        self._ser = None
        self._buffer = ''

        ##Polling timing
        self.GPSNEXT = 0.1   # poll every 100ms
        self.GPSTime = -self.GPSNEXT

        self.SIL = util.isSIL()
        self.initialize()

    def initialize(self):
        if self.SIL:
            print('Running in SIL mode.... emulating GPS')
            return

        if not SERIAL_AVAILABLE:
            print('ERROR: pyserial not available. Cannot connect to GPS antenna.')
            print('Install with: sudo pip3 install pyserial')
            return

        ##Try the user-specified port first, then common fallbacks
        ports_to_try = [self.port, '/dev/ttyAMA0', '/dev/ttyUSB0',
                        '/dev/ttyUSB1', '/dev/ttyAMA1', '/dev/ttyS0']
        ##Remove duplicates while preserving order
        seen = set()
        unique_ports = []
        for p in ports_to_try:
            if p not in seen:
                seen.add(p)
                unique_ports.append(p)

        print('Initializing GPS antenna....')
        for p in unique_ports:
            try:
                self._ser = serial.Serial(p, self.baud, timeout=0.1)
                self.port = p
                print(f'GPS connected on {p} at {self.baud} baud')
                return
            except Exception as e:
                print(f'  GPS: Could not open {p}: {e}')

        print('WARNING: Could not connect to GPS antenna on any serial port!')
        print('GPS will report default values. Check your wiring and port.')

    # -------------------------------------------------------------------------
    # NMEA PARSING
    # -------------------------------------------------------------------------

    def _validate_checksum(self, sentence):
        """Validate NMEA checksum. Returns True if valid or no checksum present."""
        if '*' not in sentence:
            return True
        try:
            data, chk = sentence[1:].rsplit('*', 1)
            calc = 0
            for c in data:
                calc ^= ord(c)
            return f'{calc:02X}' == chk.strip().upper()
        except Exception:
            return False

    def _parse_latlon(self, value_str, direction, is_lat):
        """Convert NMEA DDMM.MMMM or DDDMM.MMMM string to decimal degrees."""
        val = float(value_str)
        deg = int(val / 100)
        minutes = val - deg * 100
        decimal = deg + minutes / 60.0
        if direction in ('S', 'W'):
            decimal = -decimal
        return decimal

    def _parse_nmea(self, sentence):
        """Parse a single NMEA sentence and update GPS state."""
        sentence = sentence.strip()
        if len(sentence) < 6 or not sentence.startswith('$'):
            return
        if not self._validate_checksum(sentence):
            return
        ##Strip checksum for parsing
        if '*' in sentence:
            sentence = sentence[:sentence.rfind('*')]

        parts = sentence.split(',')
        msg_type = parts[0]

        ##GGA - Fix data: lat, lon, fix quality, satellites, altitude
        if msg_type in ('$GPGGA', '$GNGGA', '$GLGGA'):
            try:
                if len(parts) >= 10 and parts[2] and parts[4]:
                    self.fix_quality = int(parts[6]) if parts[6] else 0
                    self.num_satellites = int(parts[7]) if parts[7] else 0
                    self.has_fix = self.fix_quality > 0
                    if self.has_fix:
                        self.latitude = self._parse_latlon(parts[2], parts[3], is_lat=True)
                        self.longitude = self._parse_latlon(parts[4], parts[5], is_lat=False)
                        if parts[9]:
                            self.altitude = float(parts[9])
            except (ValueError, IndexError):
                pass

        ##RMC - Recommended minimum: lat, lon, speed, validity flag
        elif msg_type in ('$GPRMC', '$GNRMC', '$GLRMC'):
            try:
                if len(parts) >= 9:
                    self.has_fix = (parts[2] == 'A')   # A=Active/valid, V=Void
                    if self.has_fix and parts[3] and parts[5]:
                        self.latitude = self._parse_latlon(parts[3], parts[4], is_lat=True)
                        self.longitude = self._parse_latlon(parts[5], parts[6], is_lat=False)
                        if parts[7]:
                            self.speed = float(parts[7]) * 0.514444  # knots -> m/s
            except (ValueError, IndexError):
                pass

    # -------------------------------------------------------------------------
    # POLLING / UPDATING
    # -------------------------------------------------------------------------

    def poll(self, RunTime):
        if (RunTime - self.GPSTime) > self.GPSNEXT:
            self.GPSTime = RunTime
            self.update()

    def update(self):
        if self.SIL:
            self.latitude = 30.69
            self.longitude = -88.10
            self.altitude = 0.0
            self.speed = 0.0
            self.has_fix = True
            self.fix_quality = 1
            self.num_satellites = 6
            return

        if self._ser is None:
            return

        try:
            waiting = self._ser.in_waiting
            if waiting > 0:
                raw = self._ser.read(waiting)
                self._buffer += raw.decode('ascii', errors='ignore')
                ##Parse all complete lines
                while '\n' in self._buffer:
                    line, self._buffer = self._buffer.split('\n', 1)
                    self._parse_nmea(line)
        except Exception as e:
            print(f'GPS read error: {e}')

    # -------------------------------------------------------------------------
    # UTILITY METHODS (unchanged from original)
    # -------------------------------------------------------------------------

    def setOrigin(self, latO, lonO):
        self.latO = latO
        self.lonO = lonO

    def convertXYZ2LATLON(self, x, y, z):
        self.lat = x / self.GPSVAL + self.latO
        self.lon = y / (self.GPSVAL * np.cos(self.latO * np.pi / 180)) + self.lonO
        self.alt = -z
        return self.lat, self.lon, self.alt

    def convertLATLONVEC2XY(self, *argv):
        if len(argv) == 2:
            self.latitude_vec = argv[0]
            self.longitude_vec = argv[1]
        self.x_vec = (self.latitude_vec - self.latO) * 60 * self.NM2FT * self.FT2M
        self.y_vec = (self.longitude_vec - self.lonO) * 60 * self.NM2FT * self.FT2M * np.cos(self.latO * np.pi / 180)

    def setFilterConstant(self, s):
        self.filterConstant = s

    def computeVelocityVec(self):
        self.vx_raw_vec = np.zeros(len(self.x_vec) - 1)
        self.vy_raw_vec = np.zeros(len(self.y_vec) - 1)
        self.vx_vec = 0 * self.vx_raw_vec
        self.vy_vec = 0 * self.vy_raw_vec
        for i in range(0, len(self.time_vec) - 2):
            dt = self.time_vec[i + 1] - self.time_vec[i]
            self.vx_raw_vec[i] = (self.x_vec[i + 1] - self.x_vec[i]) / dt
            self.vy_raw_vec[i] = (self.y_vec[i + 1] - self.y_vec[i]) / dt
            if i > 0:
                self.vx_vec[i] = self.vx_vec[i - 1] * self.filterConstant + self.vx_raw_vec[i] * (1 - self.filterConstant)
                self.vy_vec[i] = self.vy_vec[i - 1] * self.filterConstant + self.vy_raw_vec[i] * (1 - self.filterConstant)
            else:
                self.vx_vec[i] = self.vx_raw_vec[i]
                self.vy_vec[i] = self.vy_raw_vec[i]
        self.v_raw_vec = np.sqrt(self.vx_raw_vec ** 2 + self.vy_raw_vec ** 2)
        self.speed = np.sqrt(self.vx_vec ** 2 + self.vy_vec ** 2)

    def plot(self, plt):
        if len(self.latitude_vec) > 0:
            fig = plt.figure()
            plti = fig.add_subplot(1, 1, 1)
            plti.grid()
            plti.set_xlabel('Longitude (deg)')
            plti.set_ylabel('Latitude (deg)')
            plti.get_yaxis().get_major_formatter().set_useOffset(False)
            plti.get_xaxis().get_major_formatter().set_useOffset(False)
            plt.gcf().subplots_adjust(left=0.18)
            plti.plot(self.longitude_vec, self.latitude_vec)
        if len(self.x_vec) > 0:
            fig = plt.figure()
            plti = fig.add_subplot(1, 1, 1)
            plti.grid()
            plti.set_xlabel('Y (m) E-W')
            plti.set_ylabel('X (m) N-S')
            plti.get_yaxis().get_major_formatter().set_useOffset(False)
            plti.get_xaxis().get_major_formatter().set_useOffset(False)
            plt.gcf().subplots_adjust(left=0.18)
            plti.plot(self.y_vec, self.x_vec)
        if len(self.vx_raw_vec) > 0:
            fig = plt.figure()
            plti = fig.add_subplot(1, 1, 1)
            plti.grid()
            plti.set_xlabel('Time (sec)')
            plti.set_ylabel('Velocity (m/s)')
            plti.plot(self.time_vec[0:-1], self.vx_raw_vec, label='Raw X')
            plti.plot(self.time_vec[0:-1], self.vy_raw_vec, label='Raw Y')
            plti.plot(self.time_vec[0:-1], self.vx_vec, label='X')
            plti.plot(self.time_vec[0:-1], self.vy_vec, label='Y')
            plti.legend()
