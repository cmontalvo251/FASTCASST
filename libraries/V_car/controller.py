import numpy as np
import math

class CONTROLLER():
    def __init__(self,WAYPOINTS,CONTROLMODE):
        self.Kp_steer = 0.008
        self.base_throttle = 0.4
        self.NUMCONTROLS = 2
        self.setdefaults()
        self.WAYPOINTS = WAYPOINTS
        self.WAYPOINTSLAT = WAYPOINTS[0]
        self.WAYPOINTSLON = WAYPOINTS[1]
        self.NUMWAYPOINTS = len(self.WAYPOINTSLAT)
        self.WAYINDEX = 1
        self.lastTime = 0.0
        self.elapsedTime = 0.0
        self.velocity_command = -99.0
        self.heading_command = -99.0
        self.velocityint = 0.0
        self.motor = 0
        self.aileron = 0
        self.CONTROLMODE = CONTROLMODE
        return

    def delpsi(self,psi1: float, psi2: float) -> float:
        """Calculates relative angle wrapped between -pi and pi."""
        dpsi = psi2 - psi1
        return math.atan2(math.sin(dpsi), math.cos(dpsi))

    def setdefaults(self):
        self.defaults = [0, 0]   #-1 is minimum and 0 is mid, 1 is maximum
        self.controls = [0, 0]
        self.color = 'Red'

    def loop(self, RunTime, rcin, gps_llh, rpy, g, baro):

        self.elapsedTime = RunTime - self.lastTime
        self.lastTime = RunTime
        self.setdefaults()

        if rcin.autopilot < 1500:
            ##Manual control
            self.color = 'Green'
            self.controls[0] = rcin.throttlerc
            self.controls[1] = rcin.rollrc
        elif rcin.autopilot > 1500:
            #Autopilot mode
            self.color = 'Blue'
            self.heading_command = -99
            if self.CONTROLMODE >= 3:
                self.waypoint_loop(gps_llh)
            if self.CONTROLMODE >= 2:
                if self.heading_command == -99.0:
                    self.heading_command = 45.0
                self.heading_loop(rpy,g)
            if self.CONTROLMODE >= 1:
                self.velocity_loop(gps_llh)
            if self.CONTROLMODE >= 0:
                if self.velocity_command == -99.0:
                    self.throttle = 0.75
            # Output to Control Matrix 
            self.controls[0] = self.throttle
            self.controls[1] = self.aileron

        ##Saturation
        for i in range(0, self.NUMCONTROLS):
            self.controls[i] = min(max(self.controls[i],-1),1)

        ##Overrrides
        #self.controls[0] = 1.0
        #self.controls[1] = 0.0

        #print(self.controls)
        return self.controls, self.defaults, self.color

    def waypoint_loop(self,gps_llh):
        """Waypoint navigation calculations."""
        # MATLAB 1-based (1,1) and (2,1) -> 0-based [0,0] and [1,0]
        LAT = gps_llh.latitude
        LON = gps_llh.longitude

        DLAT = self.WAYPOINTSLAT[self.WAYINDEX] - LAT
        DLON = self.WAYPOINTSLON[self.WAYINDEX] - LON

        self.heading_command = math.atan2(DLON, DLAT) * 180.0 / math.pi
        NM2FT  = 6076.115485560000
        FT2M   = 0.3048
        GPSVAL = 60.0 * NM2FT * FT2M
        distance = math.sqrt(DLAT * DLON + DLAT * DLON)*GPSVAL

        if distance < 50:
            print(f"WAY (LAT,LON) = ({self.WAYPOINTSLAT[self.WAYINDEX]},{self.WAYPOINTSLON[self.WAYINDEX]}) " f"GPS (LAT,LON) = {LAT} {LON} HCOMM = {self.heading_command} DIST = {distance}")
            self.WAYINDEX += 1
            if self.WAYINDEX > self.NUMWAYPOINTS - 1:
                self.WAYINDEX = 0

    def heading_loop(self,rpy,g):
        """Heading control proportional feedback loop."""
        kp = 2.5
        # MATLAB 1-based (6,1) -> 0-based [5,0]
        heading = rpy[2]
        dheading = -self.delpsi(heading * math.pi / 180.0, self.heading_command * math.pi / 180.0)*180.0/math.pi
        if dheading > 180:
            dheading -= 180
            dheading *= -1
        if dheading < -180:
            dheading += 180
            dheading *= -1
        self.aileron = kp * dheading
        # Constrain aileron deflection
        self.aileron = min(max(self.aileron, -1),1)

    def velocity_loop(self,gps_llh):
        """PI speed controller."""
        u = gps_llh.speed
        self.velocity_command = 5.0
        velocityerror = self.velocity_command - u

        kp = 60.0/500.0
        ki = 20.0/500.0

        self.throttle = kp * velocityerror + ki * self.velocityint
        self.throttle = min(max(self.throttle,0),1)

        #print(self.throttle,u)

        # Anti-windup integration
        if -1 < self.throttle < 1:
            self.velocityint += self.elapsedTime * velocityerror