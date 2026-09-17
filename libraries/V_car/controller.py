import numpy as np

class CONTROLLER():
    def __init__(self):
        self.Kp_steer = 0.008
        self.base_throttle = 0.4
        self.NUMCONTROLS = 2
        return

    def loop(self, RunTime, rcin, gps_llh, rpy, g, baro):
        defaults = [0, 0]   #-1 is minimum and 0 is mid, 1 is maximum
        color = 'Red'
        controls = [0, 0]
        heading_deg = rpy[2]  # yaw from AHRS

        if rcin.autopilot < 1500:
            ##Manual control
            color = 'Green'
            controls[0] = rcin.throttlerc
            controls[1] = rcin.rollrc
        elif rcin.autopilot > 1500:
            #Autopilot mode
            controls = [0, 0]

        ##Saturation
        for i in range(0, self.NUMCONTROLS):
            if controls[i] < -1:
                controls[i] = -1
            if controls[i] > 1:
                controls[i] = 1

        return controls, defaults, color
