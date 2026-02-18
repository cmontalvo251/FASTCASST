import numpy as np

class CONTROLLER():

    def __init__(self):
        #Waypoints
        self.NUMCONTROLS = 2 #throttle_motor1, throttle_motor2
        return
    
    def loop(self,RunTime,rcin):#,gps_llh,rpy,g,baro):
        ##Set defaults
        defaults = [-1,-1] #-1 is full off, 0 is middle and +1 is full on
        color = 'Red' #default to red color if something isn't working right

        ##Initialize control commands
        controls = [-1,-1]

        ##Create controls commands based on input from receiver
        if rcin.autopilot < 1500:
            #Manual control
            color = 'Green'
            controls[0] = rcin.throttlerc + rcin.yawrc
            controls[1] = rcin.throttlerc - rcin.yawrc
        elif rcin.autopilot > 1500:
            #Autopilot
            color = 'Blue'
            controls[0] = -1 #make the boat spin
            controls[1] = 1
        return controls,defaults,color
