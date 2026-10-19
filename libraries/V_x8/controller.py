import math
import numpy as np

class CONTROLLER:

    def __init__(self,WAYPOINTS,CONTROLMODE):
        # Constants & PWM parameters
        self.NUMSIGNALS = 8 #must be the same as forces.py
        self.PI = math.pi

        # State Variables
        self.control_matrix = np.zeros((self.NUMSIGNALS))
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
        for i in range(self.NUMSIGNALS):
            self.control_matrix[i] = -1

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
        elif rcin.autopilot > 1500:
            self.color = 'Blue'
            if self.CONTROLMODE >= 4:
                self.waypoint_loop(gps_llh,currentTime)
            if self.CONTROLMODE >= 3:
                if self.velocity_command == -99.0:
                    self.velocity_command = 15.0
                self.velocity_loop(gps_llh)
            if self.CONTROLMODE >= 2:
                if self.roll_command == -99.0:
                    self.roll_command = self.rollrc*50.0
                if self.pitch_command == -99.0:
                    self.pitch_command = -self.pitchrc*50.0
                if self.yaw_command == -99.0:
                    self.yaw_command = rpy[2] + self.yawrc
                self.attitude_loop(rpy,g)
            if self.CONTROLMODE >= 1:
                if self.altitude_command == -99.0:
                    self.altitude_command = 200.0
                self.altitude_loop(gps_llh,baro)
            if icontrol >= 0:
                self.motor_upper_left_bottom = (
                self.throttle
                - (self.aileron - self.OUTMID)
                - (self.elevator - self.OUTMID)
                + (self.rudder - self.OUTMID)
            )
            self.motor_upper_right_bottom = (
                self.throttle
                + (self.aileron - self.OUTMID)
                - (self.elevator - self.OUTMID)
                - (self.rudder - self.OUTMID)
            )
            self.motor_lower_right_bottom = (
                self.throttle
                + (self.aileron - self.OUTMID)
                + (self.elevator - self.OUTMID)
                + (self.rudder - self.OUTMID)
            )
            self.motor_lower_left_bottom = (
                self.throttle
                - (self.aileron - self.OUTMID)
                + (self.elevator - self.OUTMID)
                - (self.rudder - self.OUTMID)
            )
            self.motor_upper_left_top = (
                self.throttle
                - (self.aileron - self.OUTMID)
                - (self.elevator - self.OUTMID)
                - (self.rudder - self.OUTMID)
            )
            self.motor_upper_right_top = (
                self.throttle
                + (self.aileron - self.OUTMID)
                - (self.elevator - self.OUTMID)
                + (self.rudder - self.OUTMID)
            )
            self.motor_lower_right_top = (
                self.throttle
                + (self.aileron - self.OUTMID)
                + (self.elevator - self.OUTMID)
                - (self.rudder - self.OUTMID)
            )
            self.motor_lower_left_top = (
                self.throttle
                - (self.aileron - self.OUTMID)
                + (self.elevator - self.OUTMID)
                + (self.rudder - self.OUTMID)
            )

        # Send motor commands to control matrix
        self.control_matrix[0, 0] = self.motor_upper_left_bottom
        self.control_matrix[1, 0] = self.motor_upper_right_bottom
        self.control_matrix[2, 0] = self.motor_lower_right_bottom
        self.control_matrix[3, 0] = self.motor_lower_left_bottom
        self.control_matrix[4, 0] = self.motor_upper_left_top
        self.control_matrix[5, 0] = self.motor_upper_right_top
        self.control_matrix[6, 0] = self.motor_lower_right_top
        self.control_matrix[7, 0] = self.motor_lower_left_top

        # Constrain control matrix values
        for i in range(self.NUMSIGNALS):
            val = self.control_matrix[i, 0]
            val = min(max(val, self.OUTMIN), self.OUTMAX)
            self.control_matrix[i, 0] = val

    def delpsi(psi1: float, psi2: float) -> float:
        """Calculates relative angle wrapped between -pi and pi."""
        dpsi = psi2 - psi1
        return math.atan2(math.sin(dpsi), math.cos(dpsi))

    def waypoint_loop(self, sense_matrix: np.ndarray, currentTime: float):
        """Waypoint navigation calculations."""
        X = sense_matrix[0, 0]  # MATLAB index (1,1)
        Y = sense_matrix[1, 0]  # MATLAB index (2,1)
        DY = self.WAYPOINTS_Y[self.WAYINDEX] - Y
        DX = self.WAYPOINTS_X[self.WAYINDEX] - X
        distance = math.sqrt(DY * DY + DX * DX)

        if self.distance_prev != distance:
            if self.distance_prev != -999.0:
                self.distance_dot = (distance - self.distance_prev) / (
                    currentTime - self.prevTime
                )
            self.distance_prev = distance
            self.prevTime = currentTime

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
            self.yaw_command = math.atan2(DY, DX) * 180.0 / self.PI

    def velocity_loop(self, sense_matrix: np.ndarray):
        """Velocity PI control loop."""
        u = sense_matrix[6, 0]  # MATLAB index (7,1)
        velocityerror = self.velocity_command - u
        kp = 1.0
        ki = 0.0  # 8.0 * 0
        self.pitch_command = -kp * velocityerror - ki * self.velocity_int
        self.pitch_command = min(max(self.pitch_command, -20.0), 20.0)

        if -20.0 < self.pitch_command < 20.0:
            self.velocity_int += self.elapsedTime * velocityerror

    def attitude_loop(self, sense_matrix: np.ndarray):
        """Inner attitude stabilization loop."""
        roll = sense_matrix[3, 0]  # MATLAB index (4,1)
        pitch = sense_matrix[4, 0]  # MATLAB index (5,1)
        yaw = sense_matrix[19, 0]  # MATLAB index (20,1)
        roll_rate = sense_matrix[9, 0]  # MATLAB index (10,1)
        pitch_rate = sense_matrix[10, 0]  # MATLAB index (11,1)
        yaw_rate = sense_matrix[11, 0]  # MATLAB index (12,1)

        kp = 10.0
        kd = 2.0
        kpyaw = 10.0
        kdyaw = 5.0

        droll = kp * (roll - self.roll_command) + kd * roll_rate
        droll = min(max(droll, -500.0), 500.0)

        dpitch = kp * (pitch - self.pitch_command) + kd * pitch_rate
        dpitch = min(max(dpitch, -500.0), 500.0)

        dheading = (
            delpsi(yaw * self.PI / 180.0, self.yaw_command * self.PI / 180.0)
            * 180.0
            / self.PI
        )
        if dheading > 180.0:
            dheading -= 180.0
            dheading *= -1.0
        if dheading < -180.0:
            dheading += 180.0
            dheading *= -1.0

        dyaw = kpyaw * dheading + kdyaw * yaw_rate
        dyaw = min(max(dyaw, -500.0), 500.0)

        self.aileron = droll + self.OUTMID
        self.elevator = dpitch + self.OUTMID
        self.rudder = dyaw + self.OUTMID

    def altitude_loop(self, sense_matrix: np.ndarray):
        """Barometric altitude control loop."""
        altitude = sense_matrix[27, 0]  # MATLAB index (28,1)
        altitude_dot = 0.0
        if self.altitude_prev != -999.0:
            altitude_dot = (altitude - self.altitude_prev) / self.elapsedTime
        self.altitude_prev = altitude

        kp = -100.0
        kd = -50.0
        ki = -50.0
        dup = (
            kp * (altitude - self.altitude_command)
            + kd * altitude_dot
            + ki * self.altitude_int
        )
        dup = min(max(dup, -(self.OUTMAX - self.OUTMIN)), self.OUTMAX - self.OUTMIN)
        self.throttle = self.OUTMIN + dup

        if self.OUTMIN < self.throttle < self.OUTMAX:
            self.altitude_int += self.elapsedTime * (
                altitude - self.altitude_command
            )