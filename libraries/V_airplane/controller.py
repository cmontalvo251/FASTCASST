import math
import numpy as np
import time

class CONTROLLER():

    def __init__(self,WAYPOINTS,CONTROLMODE):
        # Constants & PWM parameters
        self.NUMCONTROLS = 4
        self.set_defaults()
        self.WAYPOINTS = WAYPOINTS
        self.WAYPOINTSLAT = WAYPOINTS[0]
        self.WAYPOINTSLON = WAYPOINTS[1]
        self.NUMWAYPOINTS = len(self.WAYPOINTSLAT)
        self.WAYINDEX = 0
        self.lastTime = 0.0
        self.elapsedTime = 0.0
        self.pitch_command = -99.0
        self.roll_command = -99.0
        self.altitude_prev = -999.0
        self.altitude_command = 50.0
        self.heading_command = 0.0
        self.velocity_command = 15.0
        self.velocityint = 0.0
        self.headingint = 0.0
        self.throttle = -1
        self.aileron = 0
        self.elevator = 0
        self.rudder = 0
        self.CONTROLMODE = abs(CONTROLMODE)
    
    def delpsi(self,psi1: float, psi2: float) -> float:
        """Calculates relative angle wrapped between -pi and pi."""
        dpsi = psi2 - psi1
        return math.atan2(math.sin(dpsi), math.cos(dpsi))

    def set_defaults(self):
        """Resets default control outputs (TAER)."""
        self.defaults = [-1,0,0,0]
        self.controls = [-1,0,0,0]
        self.color = 'Red'

    def loop(self, currentTime,rcin,gps_llh,rpy,g,baro):
        """Main control loop handling flight control cases."""
        self.set_defaults()

        self.elapsedTime = currentTime - self.lastTime
        self.lastTime = currentTime

        # Initialize commands
        self.roll_command = -99.0
        self.pitch_command = -99.0
        self.velocity_command = -99.0
        self.altitude_command = -99.0
        self.heading_command = -99.0

        #print('CTL',rcin.rcsignals,rcin.throttlerc,rcin.rollrc,rcin.pitchrc,rcin.yawrc,rcin.autopilot)
        #rcin.autopilot = 0.0

        if rcin.autopilot < 1500:
            ##Manual control
            self.color = 'Green'
            self.controls[0] = rcin.throttlerc
            self.controls[1] = rcin.rollrc
            self.controls[2] = rcin.pitchrc
            self.controls[3] = rcin.yawrc
        elif rcin.autopilot > 1500:
            #print('AUTO')
            #Autopilot mode
            self.color = 'Blue'
            self.heading_command = -99
            if self.CONTROLMODE >= 5:
                self.waypoint_loop(gps_llh)
            if self.CONTROLMODE >= 4:
                if self.heading_command == -99.0:
                    self.heading_command = 45.0
                self.heading_loop(rpy,g)
            if self.CONTROLMODE >= 3:
                if self.altitude_command == -99.0:
                    self.altitude_command = 50.0
                self.altitude_loop(baro)
            if self.CONTROLMODE >= 2:
                if self.velocity_command == -99.0:
                    self.velocity_command = 15.0
                self.velocity_loop(gps_llh)
            if self.CONTROLMODE >= 1:
                if self.roll_command == -99.0:
                    self.roll_command = (rcin.rollrc)*50.0
                if self.pitch_command == -99.0:
                    self.pitch_command = (rcin.pitchrc)*50.0
                #self.pitch_command = 10.0
                #self.roll_command = 10.0
                self.inner_loop(rpy,g)
            if self.CONTROLMODE >= 0:
                if self.velocity_command == -99.0:
                    self.throttle = 0.75
                if self.pitch_command == -99.0:
                    self.elevator = 0.1
                if self.roll_command == -99.0:
                    self.aileron = 0
                if self.heading_command == -99.0:
                    self.rudder = 0
            # Output to Control Matrix 
            self.controls[0] = self.throttle
            self.controls[1] = self.aileron
            self.controls[2] = self.elevator
            self.controls[3] = self.rudder

        ##Saturation
        for i in range(0, self.NUMCONTROLS):
            self.controls[i] = min(max(self.controls[i],-1),1)

        ##Overrrides
        #self.controls[0] = 1.0
        #self.controls[1] = 1.0 #Full right turn (1.0)

        #print(self.controls)
        return self.controls, self.defaults, self.color

    def waypoint_loop(self,gps_llh):
        """Waypoint navigation calculations."""
        LAT = gps_llh.latitude
        LON = gps_llh.longitude

        DLAT = self.WAYPOINTSLAT[self.WAYINDEX] - LAT
        DLON = self.WAYPOINTSLON[self.WAYINDEX] - LON

        #HEADING ANGLE COMPUTATION FROM GEMINI
        phi2 = math.radians(self.WAYPOINTSLAT[self.WAYINDEX])
        phi1 = math.radians(LAT)
        delta_lambda = math.radians(DLON)
        # Formula components
        y = math.sin(delta_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        # Calculate initial bearing in radians and convert to degrees
        self.heading_command = math.atan2(y, x)*180/np.pi
        #self.heading_command = math.atan2(DLON, DLAT) * 180.0 / math.pi

        NM2FT  = 6076.115485560000
        FT2M   = 0.3048
        GPSVAL = 60.0 * NM2FT * FT2M
        distance = math.sqrt(DLAT**2 + DLON**2)*GPSVAL

        if distance < 5:
            print(f"WAY (LAT,LON) = ({self.WAYPOINTSLAT[self.WAYINDEX]},{self.WAYPOINTSLON[self.WAYINDEX]}) " f"GPS (LAT,LON) = {LAT} {LON} HCOMM = {self.heading_command} DIST = {distance}")
            self.WAYINDEX += 1
            if self.WAYINDEX > self.NUMWAYPOINTS - 1:
                self.WAYINDEX = 0

    def heading_loop(self,rpy,g):
        """Heading control feedback loop."""
        kp = 0.4
        kd = 0.25
        ki = 0.001

        heading = rpy[2] #In degrees
        yaw_rate = g[2]

        dheading = self.delpsi(heading * math.pi / 180.0, self.heading_command * math.pi / 180.0)*180.0/math.pi
        #print(heading,self.heading_command,dheading)

        if dheading > 180:
            dheading -= 180
            dheading *= -1
        if dheading < -180:
            dheading += 180
            dheading *= -1

        self.roll_command = kp * dheading + kd * (yaw_rate) + ki * self.headingint
        if 45 > self.roll_command > -45:
            self.headingint += self.elapsedTime * dheading
        self.roll_command = min(max(self.roll_command, -45), 45)

        #print(self.roll_command)
        #time.sleep(10.0)

    def altitude_loop(self,baro):
        """Barometric altitude control loop."""
        altitude = baro.ALT  # Baro altitude
        altitude_dot = 0.0
        if self.altitude_prev != -999.0:
            altitude_dot = (altitude - self.altitude_prev) / self.elapsedTime
        self.altitude_prev = altitude

        kp = 1.0
        kd = 0.5
        self.pitch_command = kp * (self.altitude_command - altitude) + kd * (0 - altitude_dot)
        self.pitch_command = min(max(self.pitch_command, -45), 45)

    def velocity_loop(self,gps_llh):
        """PI velocity/speed controller."""
        u = gps_llh.speed
        velocityerror = self.velocity_command - u
        kp = 120.0/500.0
        ki = 8.0/500.0

        self.throttle = kp * velocityerror + ki * self.velocityint
        self.throttle = min(max(self.throttle,-1), 1)
        if -1 < self.throttle < 1:
            self.velocityint += self.elapsedTime * velocityerror

    def inner_loop(self,rpy,g):
        """Inner loop attitude stabilization."""
        roll = rpy[0]
        pitch = rpy[1]
        roll_rate = g[0]
        pitch_rate = g[1]

        kpa = 10.0/500.0
        kda = 2.0/500.0
        kpe = 10.0/500.0
        kde = 0.5/500.0
        kr = 1.0

        self.aileron = kpa * (roll - self.roll_command) + kda * roll_rate
        self.elevator = kpe * (pitch - self.pitch_command) + kde * pitch_rate
        self.rudder = kr * self.aileron

        #print(self.pitch_command,pitch,pitch_rate,self.elevator)

        self.rudder = min(max(self.rudder, -1), 1)
        self.aileron = -min(max(self.aileron, -1), 1) 
        self.elevator = min(max(self.elevator, -1), 1)
