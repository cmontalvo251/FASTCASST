import numpy as np

class CONTROLLER():

    def __init__(self):
        self.NUMCONTROLS = 2
        return

    def loop(self,RunTime,rcin,gps_llh,rpy,g,baro):
        controls = [0,0] #Put motors in the middle so off?
        defaults = [0,0]
        color = 'Red'

        #Compute the controller values
        if (rcin.autopilot < 1500):
            #Manual control
            color = 'Green'
            controls[0] = rcin.throttlerc + rcin.yawrc
            controls[1] = rcin.throttlerc - rcin.yawrc
        elif rcin.autopilot > 1500:
            color = 'Blue'
            controls[0] = -1  #Make the boat spin
            controls[1] = 1 

        ##Saturation blocks
        for i in range(0,self.NUMCONTROLS):
            if (controls[i] < -1):
                controls[i] = -1
            if (controls[i] > 1):
                controls[i] = 1

        return controls,defaults,color
