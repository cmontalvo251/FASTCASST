import numpy as np

class CONTROLLER():

    def __init__(self):
        self.NUMCONTROLS = 3 #throttle_motor1, throttle_motor2, rudder
        return
    
    def loop(self,RunTime,rcin,gps_llh,rpy,g,baro):
        ##Set defaults
        defaults = [-1,-1,0] #-1 is full off, 0 is middle and +1 is full on
        color = 'Red' #default to red color if something isn't working right        
        controls = [-1,-1,0] ##Initialize control commands

        ##Create controls commands based on input from receiver
        if rcin.autopilot < 1500:
            #Manual control
            color = 'Green'
            controls[0] = rcin.throttlerc + rcin.yawrc ##mixing from rudder rc command
            controls[1] = rcin.throttlerc - rcin.yawr
            controls[2] = rcin.rollrc ##Aileron to control physical rudder
        elif rcin.autopilot > 1500:
            #Autopilot
            color = 'Blue'
            controls[0] = 0.5  #half speed
            controls[1] = 0.5  #+ 
            controls[2] = 1 #make the boat spin

        ##Saturation blocks
        for i in range(0,self.NUMCONTROLS):
            if (controls[i] < -1):
                controls[i] = -1
            if (controls[i] > 1):
                controls[i] = 1

        return controls,defaults,color
