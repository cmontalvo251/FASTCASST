import Util.util
import GPS.ublox as UBLX
import numpy as np

class GPS():
    def __init__(self):
        self.latitude = -99
        self.longitude = -99
        self.altitude = -99
        self.NM2FT=6076.115485560000
        self.FT2M=0.3048
        self.GPSVAL = 60.0*self.NM2FT*self.FT2M
        self.latO = 33.16
        self.lonO = -88.1
    def initialize(self):
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
        self.latitude = self.getfloat(latstr)/10000000.0  ##I'm dividing by random numbers until shit looks right
        self.altitude = self.getfloat(altstr)/1000.0
        
    def update(self):
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
        return

    def setOrigin(self,latO,lonO):
        self.latO = latO
        self.lonO = lonO

    def convertXYZ2LATLON(self,x,y,z):
        self.lat = x/self.GPSVAL + self.latO
        self.lon = y/(self.GPSVAL*np.cos(self.latO*np.pi/180)) + self.lonO
        self.alt = -z
        return self.lat,self.lon,self.alt