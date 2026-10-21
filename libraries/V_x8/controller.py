import math
import time
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
        self.PHASE = 0

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
        self.velocity_command_X = -99.0
        self.velocity_command_Y = -99.0

        # Integrators & Derivatives
        self.altitude_int = 0.0
        self.velocity_intX = 0.0
        self.velocity_intY = 0.0
        self.altitude_prev = -999.0
        self.distance_dot_X = 0.0
        self.distance_dot_Y = 0.0
        self.distance_prev_X = -999.0
        self.distance_prev_Y = -999.0
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
        self.velocity_command_X = -99.0
        self.velocity_command_Y = -99.0

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
                self.waypoint_loop(rpy,gps_llh,currentTime)
            if self.CONTROLMODE >= 3:
                if self.velocity_command_X == -99.0:
                    self.velocity_command_X = 10.0
                    self.velocity_command_Y = 0.0
                self.velocity_loop(rpy,gps_llh)
            if self.CONTROLMODE >= 2:
                ## INNER LOOP
                if self.roll_command == -99.0:
                    self.roll_command = rcin.rollrc*50.0
                if self.pitch_command == -99.0:
                    self.pitch_command = -rcin.pitchrc*50.0
                if self.yaw_command == -99.0:
                    self.yaw_command = rpy[2] + rcin.yawrc
                #self.roll_command = 45
                #self.pitch_command = 45.
                #self.yaw_command = 45.
                self.attitude_loop(rpy,g)
            if self.CONTROLMODE >= 1:
                #ALTITUDE - Z COORD
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

    def waypoint_loop(self,rpy,gps_llh,currentTime):
        """Waypoint navigation calculations."""

        ###EXTRACT RELEVANT PARAMETERS
        LAT = gps_llh.latitude
        LON = gps_llh.longitude
        LAT_COMMAND = self.WAYPOINTSLAT[self.WAYINDEX]
        LON_COMMAND = self.WAYPOINTSLON[self.WAYINDEX]
        DLAT = LAT_COMMAND - LAT
        DLON = LON_COMMAND - LON

        #COMPUTE HEADING TO TARGET
        #HEADING ANGLE COMPUTATION FROM GEMINI
        phi2 = math.radians(LAT_COMMAND)
        phi1 = math.radians(LAT)
        delta_lambda = math.radians(DLON)
        # Formula components
        y = math.sin(delta_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        # Calculate initial bearing in radians and convert to degrees
        headingToTarget = math.atan2(y, x)*180/np.pi
        #And delta to target
        yaw = rpy[2] * self.PI / 180.0
        dheading = abs(self.delpsi(yaw, headingToTarget * self.PI / 180.0) * 180.0 / self.PI)

        ##COMPUTE DISTANCE TO TARGET
        NM2FT  = 6076.115485560000
        FT2M   = 0.3048
        GPSVAL = 60.0 * NM2FT * FT2M
        distanceToTarget = math.sqrt(DLAT**2 + DLON**2)*GPSVAL
        #COMPUTE DISTANCE DOT
        distanceX = DLAT*GPSVAL
        distanceY = DLON*GPSVAL
        distanceBodyX = distanceX*np.cos(yaw) + distanceY*np.sin(yaw)
        distanceBodyY = -distanceX*np.sin(yaw) + distanceY*np.cos(yaw)
        if self.distance_prev_X != distanceBodyX:
            if self.distance_prev_X != -999.0:
                self.distance_dot_X = (distanceBodyX - self.distance_prev_X) / (currentTime - self.prevTime)
            self.distance_prev_X = distanceBodyX
            if self.distance_prev_Y != -999.0:
                self.distance_dot_Y = (distanceBodyY - self.distance_prev_Y) / (currentTime - self.prevTime)
            self.distance_prev_Y = distanceBodyY
            self.prevTime = currentTime

        #print(currentTime,self.PHASE,self.WAYINDEX,distanceToTarget,dheading)        

        if self.PHASE == 0:
            ##POINT TO TARGET!!!
            self.yaw_command = headingToTarget
            self.velocity_command_X = 0.0
            self.velocity_command_Y = 0.0
            #If we're not moving and pointing towards target proceed to phase 1
            if dheading < 0.1 and abs(gps_llh.speed) < 0.1:
                self.PHASE = 1
                #print('Moving to Phase 1')
                #time.sleep(1.0)
            return
        if self.PHASE == 1:
            ##INCREASE SPEED TOWARDS TARGET!!!
            ##Velocity PID
            kp = 20.0 / 200.0
            kd = 100.0 / 200.0
            self.velocity_command_X = kp * distanceBodyX + kd * self.distance_dot_X
            self.velocity_command_Y = kp * distanceBodyY + kd * self.distance_dot_Y
            topSpeed = 10.0
            self.velocity_command_X = min(max(self.velocity_command_X, 0.0), topSpeed)
            self.velocity_command_Y = min(max(self.velocity_command_Y, 0.0), topSpeed)
            self.yaw_command = headingToTarget #make sure we're still pointing at waypoint
            #if we've arrived at the target move to phase 2
            #print(self.yaw_command,distanceBodyX,distanceBodyY,self.velocity_command_X,self.velocity_command_Y)
            #time.sleep(1.0)
            if abs(distanceToTarget) < 20:
                self.PHASE = 2
            return
        if self.PHASE == 2:
            ###STOP AND REMAIN POINTED IN THE SAME DIRECTION
            self.velocity_command_X = 0.0
            self.velocity_command_Y = 0.0
            #self.yaw_command remains unchanged from previous PHASE
            #Once we have stopped moving we can proceed to the next waypoint
            #and PHASE 0
            if abs(gps_llh.speed) < 0.1:
                self.WAYINDEX+=1
                if self.WAYINDEX > self.NUMWAYPOINTS - 1:
                    self.WAYINDEX = 0
                self.PHASE = 0
            return

    def velocity_loop(self,rpy,gps_llh):
        """Velocity PI control loop."""
        vx = gps_llh.speed_x
        vy = gps_llh.speed_y
        yaw = rpy[2]*np.pi/180.0
        u = np.cos(yaw)*vx + np.sin(yaw)*vy
        v = -np.sin(yaw)*vx + np.cos(yaw)*vy
        #u = gps_llh.speed
        #print('CTL: ',u,vx)
        velocityerrorX = self.velocity_command_X - u
        velocityerrorY = self.velocity_command_Y - v
        kp = 1.0
        ki = 0.0  # 8.0 * 0
        self.pitch_command = -kp * velocityerrorX - ki * self.velocity_intX
        self.pitch_command = min(max(self.pitch_command, -20.0), 20.0)
        self.roll_command = kp * velocityerrorY + ki * self.velocity_intY
        self.roll_command = min(max(self.roll_command, -20.0), 20.0)

        if -20.0 < self.pitch_command < 20.0:
            self.velocity_intX += self.elapsedTime * velocityerrorX
        if -20.0 < self.roll_command < 20.0:
            self.velocity_intY += self.elapsedTime * velocityerrorY

    def attitude_loop(self,rpy,g):
        """Inner attitude stabilization loop."""

        #self.pitch_command = 10.0
        #self.roll_command = 10.0

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

        dheading = -self.delpsi(yaw * self.PI / 180.0, self.yaw_command * self.PI / 180.0) * 180.0 / self.PI
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