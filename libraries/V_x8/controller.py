import math
import numpy as np

class CONTROLLER():
    def __init__(self,WAYPOINTS,CONTROLMODE):
        # Constants & PWM parameters
        self.NUMCONTROLS = 8 #must be the same as forces.py
        self.PI = math.pi

        # State Variables
        self.controls = np.zeros((self.NUMCONTROLS))
        self.defaults = np.zeros((self.NUMCONTROLS))
        self.set_defaults()
        self.CONTROLMODE = abs(CONTROLMODE)
        self.WAYPOINTS = WAYPOINTS
        self.WAYPOINTSLAT = WAYPOINTS[0]
        self.WAYPOINTSLON = WAYPOINTS[1]
        self.NUMWAYPOINTS = len(self.WAYPOINTSLAT)
        self.WAYINDEX = 0

        self.elapsedTime = 0.0
        self.lastTime = 0.0

        # Control Signals & Variables
        self.throttle = -1
        self.aileron = 0
        self.elevator = 0
        self.rudder = 0
        self.autopilot = 0

        self.roll_command = -99.0
        self.pitch_command = -99.0
        self.yaw_command = -99.0
        self.altitude_command = -99.0
        self.velocity_command = -99.0

        # Integrators & Derivatives
        self.altitude_int = 0.0
        self.velocity_int = 0.0
        self.altitude_prev = -999.0
        self.distance_dot = 0.0
        self.distance_prev = -999.0
        self.prevTime = 0.0

        # Motor Commands
        self.motor_upper_left_top = -1
        self.motor_upper_right_top = -1
        self.motor_lower_left_top = -1
        self.motor_lower_right_top = -1
        self.motor_upper_left_bottom = -1
        self.motor_upper_right_bottom = -1
        self.motor_lower_left_bottom = -1
        self.motor_lower_right_bottom = -1

    def set_defaults(self):
        """Resets control matrix outputs to minimum Command val."""
        for i in range(self.NUMCONTROLS):
            self.defaults[i] = -1
            self.controls[i] = -1
        self.color = 'Red'

    def loop(self,currentTime,rcin,gps_llh,rpy,g,baro):
        """Main control loop handling flight control cases."""
        self.set_defaults()

        self.elapsedTime = currentTime - self.lastTime
        self.lastTime = currentTime

        self.roll_command = -99.0
        self.pitch_command = -99.0
        self.yaw_command = -99.0
        self.altitude_command = -99.0
        self.velocity_command = -99.0

        if rcin.autopilot < 1500:
            #Manual control
            self.color = 'Green'
            #Set controls in ACRO MODE
            self.motor_upper_left_bottom = rcin.throttlerc - rcin.rollrc - rcin.pitchrc + rcin.yawrc 
            self.motor_upper_right_bottom = rcin.throttlerc + rcin.rollrc - rcin.pitchrc - rcin.yawrc
            self.motor_lower_right_bottom = rcin.throttlerc + rcin.rollrc + rcin.pitchrc + rcin.yawrc
            self.motor_lower_left_bottom = rcin.throttlerc - rcin.rollrc + rcin.pitchrc - rcin.yawrc
            self.motor_upper_left_top = rcin.throttlerc - rcin.rollrc - rcin.pitchrc - rcin.yawrc
            self.motor_upper_right_top = rcin.throttlerc + rcin.rollrc - rcin.pitchrc + rcin.yawrc
            self.motor_lower_right_top = rcin.throttlerc + rcin.rollrc + rcin.pitchrc - rcin.yawrc
            self.motor_lower_left_top = rcin.throttlerc - rcin.rollrc + rcin.pitchrc + rcin.yawrc
        elif rcin.autopilot > 1500:
            self.color = 'Blue'
            if self.CONTROLMODE >= 4:
                self.waypoint_loop(gps_llh,currentTime)
            if self.CONTROLMODE >= 3:
                if self.velocity_command == -99.0:
                    self.velocity_command = 10.0
                self.velocity_loop(gps_llh)
            if self.CONTROLMODE >= 2:
                if self.roll_command == -99.0:
                    self.roll_command = rcin.rollrc*50.0
                if self.pitch_command == -99.0:
                    self.pitch_command = -rcin.pitchrc*50.0
                if self.yaw_command == -99.0:
                    self.yaw_command = rpy[2] + rcin.yawrc
                self.attitude_loop(rpy,g)
            if self.CONTROLMODE >= 1:
                if self.altitude_command == -99.0:
                    self.altitude_command = 200.0
                self.altitude_loop(gps_llh,baro)
            #Set controls
            self.motor_upper_left_bottom = self.throttle - self.aileron - self.elevator + self.rudder 
            self.motor_upper_right_bottom = self.throttle + self.aileron - self.elevator - self.rudder
            self.motor_lower_right_bottom = self.throttle + self.aileron + self.elevator + self.rudder
            self.motor_lower_left_bottom = self.throttle - self.aileron + self.elevator - self.rudder
            self.motor_upper_left_top = self.throttle - self.aileron - self.elevator - self.rudder
            self.motor_upper_right_top = self.throttle + self.aileron - self.elevator + self.rudder
            self.motor_lower_right_top = self.throttle + self.aileron + self.elevator - self.rudder
            self.motor_lower_left_top = self.throttle - self.aileron + self.elevator + self.rudder

        # Send motor commands to control matrix
        self.controls[0] = self.motor_upper_left_bottom
        self.controls[1] = self.motor_upper_right_bottom
        self.controls[2] = self.motor_lower_right_bottom
        self.controls[3] = self.motor_lower_left_bottom
        self.controls[4] = self.motor_upper_left_top
        self.controls[5] = self.motor_upper_right_top
        self.controls[6] = self.motor_lower_right_top
        self.controls[7] = self.motor_lower_left_top

        # Constrain control matrix values
        for i in range(self.NUMCONTROLS):
            val = self.controls[i]
            val = min(max(val,-1),1)
            self.controls[i] = val

        return self.controls, self.defaults, self.color

    def delpsi(self,psi1,psi2) -> float:
        """Calculates relative angle wrapped between -pi and pi."""
        dpsi = psi2 - psi1
        return math.atan2(math.sin(dpsi), math.cos(dpsi))

    def waypoint_loop(self,gps_llh,currentTime):
        """Waypoint navigation calculations."""
        LAT = gps_llh.latitude
        LON = gps_llh.longitude
        DLAT = self.WAYPOINTSLAT[self.WAYINDEX] - LAT
        DLON = self.WAYPOINTSLON[self.WAYINDEX] - LON
        NM2FT  = 6076.115485560000
        FT2M   = 0.3048
        GPSVAL = 60.0 * NM2FT * FT2M
        distance = math.sqrt(DLAT**2 + DLON**2)*GPSVAL

        #Compute Distance_Dot
        if self.distance_prev != distance:
            if self.distance_prev != -999.0:
                self.distance_dot = (distance - self.distance_prev) / (currentTime - self.prevTime)
            self.distance_prev = distance
            self.prevTime = currentTime

        #Compute 
        kp = 20.0 / 200.0
        kd = 100.0 / 200.0
        self.velocity_command = kp * distance + kd * self.distance_dot
        topSpeed = 10.0
        self.velocity_command = min(max(self.velocity_command, 0.0), topSpeed)

        if distance < 10.0:
            self.velocity_command = 0.0
            self.WAYINDEX += 1
            if self.WAYINDEX > self.NUMWAYPOINTS - 1:
                self.WAYINDEX = 0
        else:
            #HEADING ANGLE COMPUTATION FROM GEMINI
            phi2 = math.radians(self.WAYPOINTSLAT[self.WAYINDEX])
            phi1 = math.radians(LAT)
            delta_lambda = math.radians(DLON)
            # Formula components
            y = math.sin(delta_lambda) * math.cos(phi2)
            x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
            # Calculate initial bearing in radians and convert to degrees
            self.yaw_command = math.atan2(y, x)*180/np.pi
            #self.yaw_command = math.atan2(DLON, DLAT) * 180.0 / self.PI

    def velocity_loop(self,gps_llh):
        """Velocity PI control loop."""
        u = gps_llh.speed
        velocityerror = self.velocity_command - u
        kp = 1.0
        ki = 0.0  # 8.0 * 0
        self.pitch_command = -kp * velocityerror - ki * self.velocity_int
        self.pitch_command = min(max(self.pitch_command, -20.0), 20.0)

        if -20.0 < self.pitch_command < 20.0:
            self.velocity_int += self.elapsedTime * velocityerror

    def attitude_loop(self,rpy,g):
        """Inner attitude stabilization loop."""
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]
        roll_rate = g[0]
        pitch_rate = g[1]
        yaw_rate = g[2]

        kp = 10.0 / 500.0
        kd = 2.0 / 500.0
        kpyaw = 10.0 / 100.0
        kdyaw = 5.0 / 100.0

        droll = kp * (roll - self.roll_command) + kd * roll_rate
        droll = min(max(droll, -0.5), 0.5)

        dpitch = kp * (pitch - self.pitch_command) + kd * pitch_rate
        dpitch = min(max(dpitch, -0.5), 0.5)

        dheading = self.delpsi(yaw * self.PI / 180.0, self.yaw_command * self.PI / 180.0) * 180.0 / self.PI
        if dheading > 180.0:
            dheading -= 180.0
            dheading *= -1.0
        if dheading < -180.0:
            dheading += 180.0
            dheading *= -1.0

        dyaw = kpyaw * dheading + kdyaw * yaw_rate
        dyaw = min(max(dyaw, -0.5), 0.5)

        self.aileron = droll
        self.elevator = dpitch
        self.rudder = dyaw 

    def altitude_loop(self,gps_llh,baro):
        """Barometric altitude control loop."""
        altitude = baro.ALT
        altitude_dot = 0.0
        if self.altitude_prev != -999.0:
            altitude_dot = (altitude - self.altitude_prev) / self.elapsedTime
            #print(altitude_dot)
        self.altitude_prev = altitude

        kp = -100.0 / 2000.0
        kd = -50.0 / 500.0
        ki = -50.0 / 5000.0
        dup = kp * (altitude - self.altitude_command) + kd * altitude_dot + ki * self.altitude_int
        dup = min(max(dup,0),2)
        self.throttle = dup - 1

        if -1 < self.throttle < 1:
            self.altitude_int += self.elapsedTime * (altitude - self.altitude_command)