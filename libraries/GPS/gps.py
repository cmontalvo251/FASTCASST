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
        self.prev_latitude = -99
        self.longitude      = -99
        self.prev_longitude = -99
        self.altitude       = -99
        self.heading = -999
        self.speed          = -99
        self.fix_quality    = 0
        self.num_satellites = 0
        self.has_fix        = False
        ##For historical data / utility methods
        self.latitude_vec  = []
        self.latitude_vec = []
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

        self.ubl = None
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
            self.ubl = ublox.UBlox(port, baudrate=baud, timeout=2)
            self.ubl.configure_poll_port()
            self.ubl.configure_port(port=ublox.PORT_SERIAL1, inMask=1, outMask=0)
            self.ubl.configure_port(port=ublox.PORT_USB,     inMask=1, outMask=1)
            self.ubl.configure_port(port=ublox.PORT_SERIAL2, inMask=1, outMask=0)
            self.ubl.configure_solution_rate(rate_ms=200)   # 5 Hz
            self.ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_PVT, 1)
            print('GPS initialized.')
        except Exception as e:
            print(f'ERROR: Could not initialize GPS: {e}')
            self.ubl = None

    def getfloat(self,instr):
        #print(instr)
        loc = instr.find('=')
        #print(loc)
        raw = instr[loc+1:]
        return np.float(raw)

    ##This function will parse a line
    def parse(self,instr):
        #You'll notice that the line is separated by spaces so first use the line.split command
        list = instr.split(' ')
        #print(list)
        #Then we extract the relevant data sets
        lonstr = list[1]
        latstr = list[2]
        altstr = list[3]
        #print(lonstr,latstr,altstr)
        ##You'll then notice that each str is separated by an equal sign. We want everything after the equal
        #sign. There are a few ways to do this. We could use split again or we could find the = and grab everything
        #after it. I think I'll go with the grab everything after it.
        self.speed = 0.0 #Revisit. this doesn't work right now
        self.longitude = self.getfloat(lonstr)/10000000.0
        self.latitude = self.getfloat(latstr)/10000000.0  ##I'm dividing by random numbers until it looks right
        self.altitude = self.getfloat(altstr)/1000.0

    def poll(self,RunTime): 
        #print('Polling GPS....',RunTime,self.GPSTime,self.GPSNEXT)
        if (RunTime - self.GPSTime) > self.GPSNEXT:
            self.elapsedTime = RunTime - self.GPSTime
            self.GPSTime = RunTime
            self.update()
        
    def update(self):
        #time.sleep(0.1)
        if not self.SIL:
            msg = self.ubl.receive_message()
            if msg is None:
                if opts.reopen:
                    self.ubl.close()
                    self.ubl = UBLX.UBlox("spi:0.0", baudrate=5000000, timeout=2)
                return
            #print(msg.name())
            if msg.name() == "NAV_POSLLH":
                outstr = str(msg).split(",")[1:]
                outstr = "".join(outstr)
                self.parse(outstr)
                #print(outstr)
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
    #        if msg.name() == "NAV_STATUS":
    #            outstr = str(msg).split(",")[1:2]
    #            outstr = "".join(outstr)
    #            print(outstr)
        else:
            self.latitude = 30.69
            self.longitude = -88.10
            self.altitude = 0.0
            self.speed = 0.0
            self.has_fix        = True
            self.fix_quality    = 1
            self.num_satellites = 6

        #Then we compute speed and heading here
        self.compute_heading_velocity()
        return
    
    def compute_heading_velocity(self):
        if self.prev_latitude != -99 and self.prev_longitude != -99:
            #Get delta lat and delta lon
            dlat = self.latitude - self.prev_latitude
            dlon = self.longitude - self.prev_longitude
            if self.heading == -99:
                self.heading = np.arctan2(dlon,dlat)*180/np.pi
            else:
                #Compute heading with filtering to smooth it out.
                self.heading = np.arctan2(dlon,dlat)*180/np.pi*self.filterConstant + self.heading*(1-self.filterConstant)
            if self.heading < 0:
                self.heading += 360
            if self.heading > 360:
                self.heading -= 360
            #print('Heading:',self.heading)
            #print('dlat:',dlat,'dlon:',dlon)
            #print('prev lat:',self.prev_latitude,'prev lon:',self.prev_longitude)
            #print('lat:',self.latitude,'lon:',self.longitude)
            #print('delta lat (m):',dlat*111000,'delta lon (m):',dlon*111000*np.cos(self.latitude*np.pi/180))
            if self.speed == -99:
                self.speed = np.sqrt((dlat*111000)**2 + (dlon*111000*np.cos(self.latitude*np.pi/180))**2)/self.elapsedTime
            else:
                self.speed = np.sqrt((dlat*111000)**2 + (dlon*111000*np.cos(self.latitude*np.pi/180))**2)/self.elapsedTime*self.filterConstant + self.speed*(1-self.filterConstant)
        else:
            self.heading = -999
            self.speed = -99
        self.prev_latitude = self.latitude
        self.prev_longitude = self.longitude

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
