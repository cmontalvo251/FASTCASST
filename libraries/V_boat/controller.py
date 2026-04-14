import numpy as np

##Earth radius in meters (for haversine distance calculation)
EARTH_RADIUS_M = 6371000.0


class CONTROLLER():

    def __init__(self):
        self.NUMCONTROLS = 3  # throttle_motor1, throttle_motor2, rudder

        ##Waypoint / autopilot state
        self.target_lat = None
        self.target_lon = None
        self.waypoint_active = False
        self.arrived = False

        ##Multi-waypoint mission state
        self._mission = []          # list of (lat, lon) including return-to-start
        self._wp_index = 0
        self._mission_complete = False

        ##Autopilot tuning gains
        ##  Kp_motor:   heading error (deg) -> motor differential command
        ##              e.g. 90 deg error * 0.008 = 0.72 diff (clipped to max_differential)
        ##  Kp_rudder:  heading error (deg) -> rudder command
        ##              e.g. 90 deg error * 0.008 = 0.72 rudder (clipped to 1.0)
        ##  base_throttle: forward thrust used while navigating (0 to 1 scale)
        ##  max_differential: maximum allowed difference between motor commands
        ##  arrival_radius: stop navigating when within this many meters of target
        self.Kp_motor = 0.008
        self.Kp_rudder = 0.008
        self.base_throttle = 0.4
        self.max_differential = 0.4
        self.arrival_radius = 5.0   # meters

        return

    # -------------------------------------------------------------------------
    # WAYPOINT MANAGEMENT
    # -------------------------------------------------------------------------

    def set_waypoint(self, lat, lon):
        """Set the GPS coordinate the autopilot should navigate toward."""
        self.target_lat = float(lat)
        self.target_lon = float(lon)
        self.waypoint_active = True
        self.arrived = False
        print(f'[AUTOPILOT] Waypoint set -> lat={self.target_lat:.6f}, lon={self.target_lon:.6f}')

    def set_mission(self, waypoints, start_lat, start_lon):
        """
        Load an ordered list of GPS waypoints. The vehicle navigates each
        in sequence then returns to (start_lat, start_lon).

        waypoints  : list of (lat, lon) tuples — the operator-defined stops
        start_lat  : latitude  of the return-home point (current GPS at upload)
        start_lon  : longitude of the return-home point (current GPS at upload)
        """
        if not waypoints:
            print('[AUTOPILOT] set_mission: empty waypoint list, ignored.')
            return
        self._mission = [(float(la), float(lo)) for la, lo in waypoints]
        self._mission.append((float(start_lat), float(start_lon)))  # return-to-start
        self._wp_index = 0
        self._mission_complete = False
        self.target_lat = self._mission[0][0]
        self.target_lon = self._mission[0][1]
        self.waypoint_active = True
        self.arrived = False
        total = len(self._mission)
        print(f'[AUTOPILOT] Mission loaded: {total - 1} waypoint(s) + '
              f'return to start ({start_lat:.6f}, {start_lon:.6f})')

    def _advance_waypoint(self):
        """Advance to the next waypoint in the mission, or end the mission."""
        self._wp_index += 1
        if self._wp_index < len(self._mission):
            self.target_lat = self._mission[self._wp_index][0]
            self.target_lon = self._mission[self._wp_index][1]
            self.arrived = False
            is_last = (self._wp_index == len(self._mission) - 1)
            label = 'Return to Start' if is_last else f'WP {self._wp_index + 1}'
            print(f'[AUTOPILOT] Advancing to {label}: '
                  f'{self.target_lat:.6f}, {self.target_lon:.6f}')
        else:
            self.waypoint_active = False
            self._mission_complete = True
            self.target_lat = None
            self.target_lon = None
            print('[AUTOPILOT] Mission complete — returned to start.')

    def clear_waypoint(self):
        """Cancel the active waypoint or mission and stop autopilot navigation."""
        self.waypoint_active = False
        self.arrived = False
        self.target_lat = None
        self.target_lon = None
        self._mission = []
        self._wp_index = 0
        self._mission_complete = False
        print('[AUTOPILOT] Waypoint/mission cleared')

    # -------------------------------------------------------------------------
    # NAVIGATION MATH
    # -------------------------------------------------------------------------

    def _bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate compass bearing from (lat1,lon1) to (lat2,lon2).
        Returns degrees in [0, 360) measured clockwise from true North.
        """
        lat1_r = np.radians(lat1)
        lat2_r = np.radians(lat2)
        dlon_r = np.radians(lon2 - lon1)

        x = np.sin(dlon_r) * np.cos(lat2_r)
        y = (np.cos(lat1_r) * np.sin(lat2_r)
             - np.sin(lat1_r) * np.cos(lat2_r) * np.cos(dlon_r))

        bearing_deg = np.degrees(np.arctan2(x, y))
        return (bearing_deg + 360.0) % 360.0

    def _distance(self, lat1, lon1, lat2, lon2):
        """
        Haversine distance between two GPS points.
        Returns distance in meters.
        """
        lat1_r, lat2_r = np.radians(lat1), np.radians(lat2)
        dlat_r = np.radians(lat2 - lat1)
        dlon_r = np.radians(lon2 - lon1)

        a = (np.sin(dlat_r / 2) ** 2
             + np.cos(lat1_r) * np.cos(lat2_r) * np.sin(dlon_r / 2) ** 2)
        c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
        return EARTH_RADIUS_M * c

    def _normalize_angle(self, angle):
        """Wrap angle to [-180, 180] degrees."""
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    # -------------------------------------------------------------------------
    # AUTOPILOT CONTROL LAW
    # -------------------------------------------------------------------------

    def _autopilot_commands(self, gps_lat, gps_lon, heading_deg):
        """
        Compute [motor1, motor2, rudder] commands to steer toward the waypoint.

        Strategy:
          1. Compute bearing to waypoint and distance.
          2. Compute heading error = bearing - current_heading (normalized).
          3. Rudder = Kp_rudder * heading_error  (proportional turn command)
          4. Motor differential = Kp_motor * heading_error
             -> motor1 = base_throttle + differential  (speeds up on outside of turn)
             -> motor2 = base_throttle - differential  (slows down on inside of turn)
          5. If within arrival_radius, cut throttle and declare arrived.

        All commands are in [-1, 1] matching the RCIO convention:
          -1 = SERVO_MIN (motors off/idle)
           0 = SERVO_MID (neutral)
          +1 = SERVO_MAX (full forward)
        """
        if not self.waypoint_active or self.target_lat is None:
            return [-1, -1, 0]

        ##Reject stale / invalid GPS
        if gps_lat == -99 or gps_lon == -99:
            print('[AUTOPILOT] Waiting for valid GPS fix...')
            return [-1, -1, 0]

        dist = self._distance(gps_lat, gps_lon, self.target_lat, self.target_lon)
        bearing = self._bearing(gps_lat, gps_lon, self.target_lat, self.target_lon)

        ##Check arrival
        if dist < self.arrival_radius:
            if not self.arrived:
                print(f'[AUTOPILOT] Arrived at waypoint! Distance = {dist:.1f} m')
                if self._mission:
                    ##Mission mode: advance to the next waypoint automatically
                    self._advance_waypoint()
                else:
                    ##Single-waypoint mode: hold idle
                    self.arrived = True
            return [-1, -1, 0]

        ##Heading error: positive = need to turn right, negative = need to turn left
        heading_error = self._normalize_angle(bearing - heading_deg)

        ##Rudder proportional to heading error, clipped to [-1, 1]
        rudder = float(np.clip(self.Kp_rudder * heading_error, -1.0, 1.0))

        ##Differential motor thrust for yaw authority
        differential = float(np.clip(self.Kp_motor * heading_error,
                                     -self.max_differential, self.max_differential))
        motor1 = float(np.clip(self.base_throttle + differential, -1.0, 1.0))
        motor2 = float(np.clip(self.base_throttle - differential, -1.0, 1.0))

        print(f'[AUTOPILOT] dist={dist:.1f}m  bearing={bearing:.1f}°  '
              f'heading={heading_deg:.1f}°  err={heading_error:.1f}°  '
              f'M1={motor1:.2f}  M2={motor2:.2f}  RUD={rudder:.2f}')

        return [motor1, motor2, rudder]

    # -------------------------------------------------------------------------
    # MAIN CONTROL LOOP
    # -------------------------------------------------------------------------

    def loop(self, RunTime, rcin, gps_lat=-99, gps_lon=-99, heading_deg=0.0):
        """
        Called every control loop iteration.

        Args:
            RunTime    - elapsed time (seconds)
            rcin       - RCInput object with throttlerc, yawrc, rollrc, autopilot
            gps_lat    - current GPS latitude  (degrees), -99 if no fix
            gps_lon    - current GPS longitude (degrees), -99 if no fix
            heading_deg- current compass heading from IMU AHRS (degrees, 0 = North)

        Returns:
            controls   - [motor1, motor2, rudder] commands in [-1, 1]
            defaults   - safe defaults used when disarmed
            color      - LED color string
                          'Green'  = manual
                          'Blue'   = autopilot navigating
                          'Yellow' = autopilot idle (no waypoint / arrived)
                          'Red'    = default / error
        """
        defaults = [-1, -1, 0]   # motors idle, rudder centered
        color = 'Red'
        controls = [-1, -1, 0]

        if rcin.autopilot < 1500:
            ##Manual control: pilot drives with throttle + yaw (skid-steer) and rudder
            color = 'Green'
            controls[0] = rcin.throttlerc + rcin.yawrc
            controls[1] = rcin.throttlerc - rcin.yawrc
            controls[2] = rcin.rollrc

        elif rcin.autopilot > 1500:
            ##GPS Autopilot mode
            if self.waypoint_active and not self.arrived:
                color = 'Blue'
                controls = self._autopilot_commands(gps_lat, gps_lon, heading_deg)
            else:
                ##No waypoint loaded or already arrived — hold idle
                color = 'Yellow'
                controls = [-1, -1, 0]

        ##Servo is mounted upside down — invert rudder so left command turns left
        controls[2] = -controls[2]

        return controls, defaults, color
