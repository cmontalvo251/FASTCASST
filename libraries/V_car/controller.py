import numpy as np

##Earth radius in meters (for haversine distance calculation)
EARTH_RADIUS_M = 6371000.0

class CONTROLLER():

    def __init__(self):
        ##Waypoint / autopilot state
        self.target_lat = None
        self.target_lon = None
        self.waypoint_active = False
        self.arrived = False

        ##Multi-waypoint mission state
        self._mission = []          # list of (lat, lon) including return-to-start
        self._wp_index = 0
        self._mission_complete = False

        ##Set True by loop() when RC signal is lost and failsafe autopilot takes over.
        ##fast.py reads this to allow motor output even when ARMED is False.
        self.failsafe_active = False

        ##Autopilot tuning gains
        ##  Kp_steer:      heading error (deg) -> steering command
        ##                 e.g. 90 deg error * 0.008 = 0.72 (clipped to 1.0)
        ##                 Car has a single steering servo (no differential thrust
        ##                 like the boat), so all yaw authority comes from here.
        ##  base_throttle: forward throttle used while navigating (0 to 1 scale)
        ##  arrival_radius: stop navigating when within this many meters of target
        self.Kp_steer = 0.008
        self.base_throttle = 0.4
        self.arrival_radius = 5.0   # meters

        self.NUMCONTROLS = 2
        return

    # -------------------------------------------------------------------------
    # WAYPOINT MANAGEMENT (ported from libraries/V_boat/controller.py)
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
    # NAVIGATION MATH (identical to libraries/V_boat/controller.py)
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
        Compute [throttle, steering] commands to steer toward the waypoint.

        Unlike the boat (twin motors + rudder), the car has a single drive
        motor and one steering servo, so all yaw authority comes from the
        steering command instead of differential thrust:
          1. Compute bearing to waypoint and distance.
          2. Compute heading error = bearing - current_heading (normalized).
          3. Steering = Kp_steer * heading_error (proportional turn command)
          4. Throttle is held at base_throttle while navigating.
          5. If within arrival_radius, cut throttle and declare arrived.

        Commands are in [-1, 1] matching the RCIO convention:
          -1 = full reverse / full left, 0 = mid/stopped/straight, +1 = full forward / full right
        """
        if not self.waypoint_active or self.target_lat is None:
            return [0, 0]

        ##Reject stale / invalid GPS
        if gps_lat == -99 or gps_lon == -99:
            print('[AUTOPILOT] Waiting for valid GPS fix...')
            return [0, 0]

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
            return [0, 0]

        ##Heading error: positive = need to turn right, negative = need to turn left
        heading_error = self._normalize_angle(bearing - heading_deg)

        ##Steering proportional to heading error, clipped to [-1, 1]
        steering = float(np.clip(self.Kp_steer * heading_error, -1.0, 1.0))
        throttle = self.base_throttle

        print(f'[AUTOPILOT] dist={dist:.1f}m  bearing={bearing:.1f}°  '
              f'heading={heading_deg:.1f}°  err={heading_error:.1f}°  '
              f'THR={throttle:.2f}  STEER={steering:.2f}')

        return [throttle, steering]

    # -------------------------------------------------------------------------
    # MAIN CONTROL LOOP
    # -------------------------------------------------------------------------

    def loop(self, RunTime, rcin, gps_llh, rpy, g, baro):
        """
        Called every control loop iteration by fast.py.

        Args:
            RunTime  - elapsed time (seconds)
            rcin     - RCInput object with throttlerc, rollrc, autopilot
            gps_llh  - GPS object (uses .latitude / .longitude, -99 if no fix)
            rpy      - current AHRS orientation [roll, pitch, yaw] (degrees)
            g        - angular rates (unused here)
            baro     - barometer object (unused here)

        Returns:
            controls   - [throttle, steering] commands in [-1, 1]
            defaults   - safe defaults used when disarmed
            color      - LED color string
                          'Green'  = manual
                          'Blue'   = autopilot navigating
                          'Yellow' = autopilot idle (no waypoint / arrived)
                          'Red'    = default / error
        """
        defaults = [0, 0]   #-1 is minimum and 0 is mid, 1 is maximum
        color = 'Red'
        controls = [0, 0]
        self.failsafe_active = False

        heading_deg = rpy[2]  # yaw from AHRS

        ##RC signal lost — transmitter out of range or powered off.
        ##If a mission is loaded, continue it so the car returns home on its own.
        ##If no mission is active, idle the motor safely.
        if getattr(rcin, 'signal_lost', False):
            self.failsafe_active = True
            if self.waypoint_active and not self.arrived:
                print('[FAILSAFE] RC signal lost — continuing autopilot mission.')
                color = 'Blue'
                controls = self._autopilot_commands(gps_llh.latitude, gps_llh.longitude, heading_deg)
            else:
                print('[FAILSAFE] RC signal lost — no mission active, holding idle.')
                color = 'Yellow'
                controls = [0, 0]
            return controls, defaults, color

        if rcin.autopilot < 1500:
            ##Manual control
            color = 'Green'
            controls[0] = rcin.throttlerc
            controls[1] = rcin.rollrc
        elif rcin.autopilot > 1500:
            ##GPS Autopilot mode
            if self.waypoint_active and not self.arrived:
                color = 'Blue'
                controls = self._autopilot_commands(gps_llh.latitude, gps_llh.longitude, heading_deg)
            else:
                ##No waypoint loaded or already arrived — hold idle
                color = 'Yellow'
                controls = [0, 0]

        ##Saturation
        for i in range(0, self.NUMCONTROLS):
            if controls[i] < -1:
                controls[i] = -1
            if controls[i] > 1:
                controls[i] = 1

        return controls, defaults, color
