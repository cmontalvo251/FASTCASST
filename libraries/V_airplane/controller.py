import numpy as np

class CONTROLLER():

    def __init__(self):
        #Waypoints
        self.wp = np.array([[-88.1755, 30.6906], [-88.1750, 30.6902], [-88.1745, 30.6906]])
        self.wp_index = 0 #Keeps track of which waypoint is being used
        self.wp_index_max = len(self.wp)
        self.R = 6371*10**3 #Radius of the Earth in m
        self.NUMCONTROLS = 4 # Control outputs:throttle, aileron, elevator, rudder

        # Inner-loop attitude controller gains
        self.kp_roll = 0.01
        self.kd_roll = 0.001
        self.kp_pitch = 0.01
        self.kd_pitch = 0.001
        self.kr_rudder = 1.0

    
    def loop(self, RunTime, rcin, gps_llh=None, rpy=None, g=None, baro=None):
        """
        Flight control loop.

        Inputs:
            RunTime: Elapsed time (s)
            rcin: RC input object
            gps_llh: GPS data
            rpy: Roll, pitch, yaw from AHRS
            g: Angular rates from IMU

        Returns:
            controls: [throttle, aileron, elevator, rudder]
            defaults: Failsafe control values
            color: Status LED colour
        """

        # Failsafe/default control values
        defaults = [-1.0, 0.0, 0.0, 0.0]

        # Initialise outputs
        controls = defaults.copy()
        color = 'Red'

        # Manual control
        if rcin.autopilot < 1500:
            color = 'Green'

            controls[0] = rcin.throttlerc
            controls[1] = rcin.rollrc
            controls[2] = rcin.pitchrc
            controls[3] = rcin.yawrc
         # Autopilot
        else:
            color = 'Blue'

            # Keep throttle under RC control
            controls[0] = rcin.throttlerc

            if rpy is not None and g is not None:
                roll = rpy[0]
                pitch = rpy[1]

                roll_rate = g[0]
                pitch_rate = g[1]

                # Desired attitude: wings level and zero pitch
                roll_control = (self.kp_roll * roll+ self.kd_roll * roll_rate)
                pitch_control = (self.kp_pitch * pitch+ self.kd_pitch * pitch_rate)

                # Apply control inputs
                controls[1] = -roll_control
                controls[2] = -pitch_control
                controls[3] = self.kr_rudder * roll_control

            else:
                # Level trim if attitude/rate data is unavailable
                controls = [0.5, 0.0, 0.0, 0.0]

        # Limit all control outputs to [-1, 1]
        for i in range(self.NUMCONTROLS):
            controls[i] = float(np.clip(controls[i], -1.0, 1.0))

        return controls, defaults, color

