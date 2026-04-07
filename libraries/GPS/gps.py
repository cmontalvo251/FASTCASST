import sys
import time
import numpy as np

sys.path.append('../Util')
sys.path.append('../libraries/Util')
sys.path.append('../Ublox')
sys.path.append('../libraries/Ublox')
import util

try:
    import ublox
    UBLOX_AVAILABLE = True
except ImportError:
    UBLOX_AVAILABLE = False
    print('WARNING: ublox library not found.')


class GPS():
    def __init__(self, port='spi:0.0', baud=5000000):
        ##Set defaults
        self.latitude       = -99
        self.longitude      = -99
        self.altitude       = -99
        self.speed          = -99
        self.fix_quality    = 0
        self.num_satellites = 0
        self.has_fix        = False

        ##For historical data / utility methods
        self.latitude_vec  = []
        self.longitude_vec = []
        self.time_vec      = []
        self.x_vec         = []
        self.y_vec         = []
        self.filterConstant = 0.2
        self.NM2FT  = 6076.115485560000
        self.FT2M   = 0.3048
        self.GPSVAL = 60.0 * self.NM2FT * self.FT2M
        self.latO   = 33.16
        self.lonO   = -88.1

        ##Polling timing
        self.GPSNEXT = 0.1
        self.GPSTime = -self.GPSNEXT

        self._ubl = None
        self.SIL  = util.isSIL()
        self.initialize(port, baud)

    def initialize(self, port='spi:0.0', baud=5000000):
        if self.SIL:
            print('Running in SIL mode — emulating GPS')
            return

        if not UBLOX_AVAILABLE:
            print('ERROR: ublox library not available.')
            return

        print(f'Initializing Navio2 GPS via {port}...')
        try:
            self._ubl = ublox.UBlox(port, baudrate=baud, timeout=2)
            self._ubl.configure_poll_port()
            self._ubl.configure_port(port=ublox.PORT_SERIAL1, inMask=1, outMask=0)
            self._ubl.configure_port(port=ublox.PORT_USB,     inMask=1, outMask=1)
            self._ubl.configure_port(port=ublox.PORT_SERIAL2, inMask=1, outMask=0)
            self._ubl.configure_solution_rate(rate_ms=200)   # 5 Hz
            self._ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_PVT, 1)
            print('GPS initialized.')
        except Exception as e:
            print(f'ERROR: Could not initialize GPS: {e}')
            self._ubl = None

    def poll(self, RunTime):
        if (RunTime - self.GPSTime) > self.GPSNEXT:
            self.GPSTime = RunTime
            self.update()

    def update(self):
        if self.SIL:
            self.latitude       = 30.69
            self.longitude      = -88.10
            self.altitude       = 0.0
            self.speed          = 0.0
            self.has_fix        = True
            self.fix_quality    = 1
            self.num_satellites = 6
            return

        if self._ubl is None:
            return

        try:
            msg = self._ubl.receive_message()
            if msg is None:
                return
            if msg.name() == 'NAV_PVT':
                msg.unpack()
                fix  = msg._fields.get('fixType', 0)
                sats = msg._fields.get('numSV',   0)
                lat  = msg._fields.get('lat',     0) * 1e-7
                lon  = msg._fields.get('lon',     0) * 1e-7
                alt  = msg._fields.get('height',  0) * 1e-3   # mm -> m
                spd  = msg._fields.get('gSpeed',  0) * 1e-3   # mm/s -> m/s

                self.fix_quality    = fix
                self.num_satellites = sats
                self.has_fix        = fix >= 3
                if self.has_fix:
                    self.latitude  = lat
                    self.longitude = lon
                    self.altitude  = alt
                    self.speed     = spd
        except Exception as e:
            print(f'GPS read error: {e}')

    # -------------------------------------------------------------------------
    # UTILITY METHODS
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
            self.latitude_vec  = argv[0]
            self.longitude_vec = argv[1]
        self.x_vec = (self.latitude_vec  - self.latO) * 60 * self.NM2FT * self.FT2M
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
                self.vx_vec[i] = self.vx_vec[i-1] * self.filterConstant + self.vx_raw_vec[i] * (1 - self.filterConstant)
                self.vy_vec[i] = self.vy_vec[i-1] * self.filterConstant + self.vy_raw_vec[i] * (1 - self.filterConstant)
            else:
                self.vx_vec[i] = self.vx_raw_vec[i]
                self.vy_vec[i] = self.vy_raw_vec[i]
        self.v_raw_vec = np.sqrt(self.vx_raw_vec**2 + self.vy_raw_vec**2)
        self.speed     = np.sqrt(self.vx_vec**2    + self.vy_vec**2)
