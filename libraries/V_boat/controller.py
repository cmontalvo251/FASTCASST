import numpy as np

class CONTROLLER():

    def __init__(self):
        self.NUMCONTROLS = 3 #throttle_motor1, throttle_motor2, rudder
        return
    
    def loop(self,RunTime,rcin):#,gps_llh,rpy,g,baro):
        ##Set defaults
        defaults = [-1,-1,0] #-1 is full off, 0 is middle and +1 is full on
        color = 'Red' #default to red color if something isn't working right

        ##Initialize control commands
        controls = [-1,-1,0]

        ##Create controls commands based on input from receiver
        if rcin.autopilot < 1500:
            #Manual control
            color = 'Green'
            controls[0] = rcin.throttlerc + rcin.yawrc
            controls[1] = rcin.throttlerc - rcin.yawrc
            controls[2] = rcin.rollrc
        elif rcin.autopilot > 1500:
            #Autopilot
            color = 'Blue'
            controls[0] = 0.5 
            controls[1] = 0.5
            controls[2] = 1 #make the boat spin
        return controls,defaults,color
