import numpy as np

class CONTROLLER():
    def __init__(self,WAYPOINTS):
        self.Kp_steer = 0.008
        self.base_throttle = 0.4
        self.NUMCONTROLS = 2
        self.setdefaults()
        self.WAYPOINTS = WAYPOINTS
        return

    def setdefaults(self):
        self.defaults = [0, 0]   #-1 is minimum and 0 is mid, 1 is maximum
        self.controls = [0, 0]
        self.color = 'Red'

    def loop(self, RunTime, rcin, gps_llh, rpy, g, baro):
        self.setdefaults()
        if rcin.autopilot < 1500:
            ##Manual control
            self.color = 'Green'
            self.controls[0] = rcin.throttlerc
            self.controls[1] = rcin.rollrc
        elif rcin.autopilot > 1500:
            #Autopilot mode
            self.controls = [0, 0]
            self.color = 'Blue'

        ##Saturation
        for i in range(0, self.NUMCONTROLS):
            if self.controls[i] < -1:
                self.controls[i] = -1
            if self.controls[i] > 1:
                self.controls[i] = 1

        #print(self.controls)
        return self.controls, self.defaults, self.color
