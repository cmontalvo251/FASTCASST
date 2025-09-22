import sys
sys.path.append('../libraries/Ublox')
sys.path.append('../libraries/Util')
import util
import ublox as UBLX
import numpy as np

##NOTE THAT FUNCTIONS LIKE IFOV AND computeGeocentricLATLON functions are in
##gpscomms.py which is over in aerospace.git/satcomms/gpscomms.py

class GPS():
    def __init__(self):
        ##Set defaults
        self.latitude = -99
        self.longitude = -99
        self.altitude = -99
        self.speed = -99
        self.latitude_vec = []
        self.longitude_vec = []
        self.time_vec = []
        self.x_vec = []
        self.y_vec = [] 
        self.filterConstant = 0.2
        self.NM2FT=6076.115485560000
        self.FT2M=0.3048
        self.GPSVAL = 60.0*self.NM2FT*self.FT2M
        self.latO = 33.16
        self.lonO = -88.1
        self.initialize()
    def initialize(self):
        self.GPSNEXT = 0.25
        self.GPSTime = -self.GPSNEXT
        self.SIL = util.isSIL()
        if self.SIL:
            print('Running in SIL mode....emulating GPS')
        else:
            print('Initializing GPS....')
            self.ubl = UBLX.UBlox("spi:0.0", baudrate=5000000, timeout=2)
            self.ubl.configure_poll_port()
            self.ubl.configure_poll(UBLX.CLASS_CFG, UBLX.MSG_CFG_USB)
            #ubl.configure_poll(UBLX.CLASS_MON, UBLX.MSG_MON_HW)

            self.ubl.configure_port(port=UBLX.PORT_SERIAL1, inMask=1, outMask=0)
            self.ubl.configure_port(port=UBLX.PORT_USB, inMask=1, outMask=1)
            self.ubl.configure_port(port=UBLX.PORT_SERIAL2, inMask=1, outMask=0)
            self.ubl.configure_poll_port()
            self.ubl.configure_poll_port(UBLX.PORT_SERIAL1)
            self.ubl.configure_poll_port(UBLX.PORT_SERIAL2)
            self.ubl.configure_poll_port(UBLX.PORT_USB)
            self.ubl.configure_solution_rate(rate_ms=1000)

            self.ubl.set_preferred_dynamic_model(None)
            self.ubl.set_preferred_usePPP(None)
            
            self.ubl.configure_message_rate(UBLX.CLASS_NAV, UBLX.MSG_NAV_POSLLH, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_NAV, UBLX.MSG_NAV_PVT, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_NAV, UBLX.MSG_NAV_STATUS, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_NAV, UBLX.MSG_NAV_SOL, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_NAV, UBLX.MSG_NAV_VELNED, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_NAV, UBLX.MSG_NAV_SVINFO, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_NAV, UBLX.MSG_NAV_VELECEF, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_NAV, UBLX.MSG_NAV_POSECEF, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_RXM, UBLX.MSG_RXM_RAW, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_RXM, UBLX.MSG_RXM_SFRB, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_RXM, UBLX.MSG_RXM_SVSI, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_RXM, UBLX.MSG_RXM_ALM, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_RXM, UBLX.MSG_RXM_EPH, 1)
            self.ubl.configure_message_rate(UBLX.CLASS_NAV, UBLX.MSG_NAV_TIMEGPS, 5)
            self.ubl.configure_message_rate(UBLX.CLASS_NAV, UBLX.MSG_NAV_CLOCK, 5)
            self.ubl.configure_message_rate(UBLX.CLASS_NAV, UBLX.MSG_NAV_DGPS, 5)
            print('GPS initialized')

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
        self.longitude = self.getfloat(lonstr)/10000000.0
        self.latitude = self.getfloat(latstr)/10000000.0  ##I'm dividing by random numbers until it looks right
        self.altitude = self.getfloat(altstr)/1000.0

    def poll(self,RunTime):
        if (RunTime - self.GPSTime) > self.GPSNEXT:
            GPSTime = RunTime
            self.update()
        
    def update(self):
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
    #        if msg.name() == "NAV_STATUS":
    #            outstr = str(msg).split(",")[1:2]
    #            outstr = "".join(outstr)
    #            print(outstr)
        else:
            self.latitude = 30.69
            self.longitude = -88.10
            self.altitude = 0.0
            self.speed = 0.0
        return

    def setOrigin(self,latO,lonO):
        self.latO = latO
        self.lonO = lonO

    def convertXYZ2LATLON(self,x,y,z):
        self.lat = x/self.GPSVAL + self.latO
        self.lon = y/(self.GPSVAL*np.cos(self.latO*np.pi/180)) + self.lonO
        self.alt = -z
        return self.lat,self.lon,self.alt

    def convertLATLONVEC2XY(self,*argv):
        if len(argv) == 2:
            self.latitude_vec = argv[0]
            self.longitude_vec = argv[1]
        self.x_vec = (self.latitude_vec - self.latO)*60*self.NM2FT*self.FT2M; #%%//North direction - Xf , meters
        self.y_vec = (self.longitude_vec - self.lonO)*60*self.NM2FT*self.FT2M*np.cos(self.latO*np.pi/180); #%%//East direction - Yf, meters

    def setFilterConstant(self,s):
        self.filterConstant = s

    def computeVelocityVec(self):
        self.vx_raw_vec = np.zeros(len(self.x_vec)-1)
        self.vy_raw_vec = np.zeros(len(self.y_vec)-1)
        self.vx_vec = 0*self.vx_raw_vec
        self.vy_vec = 0*self.vy_raw_vec
        for i in range(0,len(self.time_vec)-2):
            dt = self.time_vec[i+1]-self.time_vec[i]
            self.vx_raw_vec[i] = (self.x_vec[i+1] - self.x_vec[i])/dt
            self.vy_raw_vec[i] = (self.y_vec[i+1] - self.y_vec[i])/dt
            if i > 0:
                self.vx_vec[i] = self.vx_vec[i-1]*self.filterConstant + self.vx_raw_vec[i]*(1-self.filterConstant)
                self.vy_vec[i] = self.vy_vec[i-1]*self.filterConstant + self.vy_raw_vec[i]*(1-self.filterConstant)
            else:
                self.vx_vec[i] = self.vx_raw_vec[i]
                self.vy_vec[i] = self.vy_raw_vec[i]
        self.v_raw_vec = np.sqrt(self.vx_raw_vec**2 + self.vy_raw_vec**2)
        self.speed = np.sqrt(self.vx_vec**2 + self.vy_vec**2)

    def plot(self,plt):
        if len(self.latitude_vec) > 0:
            fig = plt.figure()
            plti = fig.add_subplot(1,1,1)
            plti.grid()
            plti.set_xlabel('Longitude (deg)')
            plti.set_ylabel('Latitude (deg)')
            plti.get_yaxis().get_major_formatter().set_useOffset(False)
            plti.get_xaxis().get_major_formatter().set_useOffset(False)
            plt.gcf().subplots_adjust(left=0.18)
            plti.plot(self.longitude_vec,self.latitude_vec)
        if len(self.x_vec) > 0:
            fig = plt.figure()
            plti = fig.add_subplot(1,1,1)
            plti.grid()
            plti.set_xlabel('Y (m) E-W')
            plti.set_ylabel('X (m) N-S')
            plti.get_yaxis().get_major_formatter().set_useOffset(False)
            plti.get_xaxis().get_major_formatter().set_useOffset(False)
            plt.gcf().subplots_adjust(left=0.18)
            plti.plot(self.y_vec,self.x_vec)
        if len(self.vx_raw_vec) > 0:
            fig = plt.figure()
            plti = fig.add_subplot(1,1,1)
            plti.grid()
            plti.set_xlabel('Time (sec)')
            plti.set_ylabel('Velocity (m/s)')
            plti.plot(self.time_vec[0:-1],self.vx_raw_vec,label='Raw X')
            plti.plot(self.time_vec[0:-1],self.vy_raw_vec,label='Raw Y')
            #plti.plot(self.time_vec[0:-1],self.v_raw_vec,label='Raw Total')
            plti.plot(self.time_vec[0:-1],self.vx_vec,label='X')
            plti.plot(self.time_vec[0:-1],self.vy_vec,label='Y')
            #plti.plot(self.time_vec[0:-1],self.v_vec,label='Total')
        
            plti.legend()




