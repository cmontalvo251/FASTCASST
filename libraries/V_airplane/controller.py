import numpy as np

class CONTROLLER():

    def __init__(self):
        #Waypoints
        self.wp = np.array([[-88.1755, 30.6906], [-88.1750, 30.6902], [-88.1745, 30.6906]])
        self.wp_index = 0 #Keeps track of which waypoint is being used
        self.wp_index_max = len(self.wp)
        self.R = 6371*10**3 #Radius of the Earth in m
        self.NUMCONTROLS = 4 #throttle, aileron, elevator, rudder
        return
    
    def loop(self,RunTime,rcin):#,gps_llh,rpy,g,baro):
        ##Set defaults
        defaults = [-1,0,0,0] #-1 is minimum and 0 is mid, 1 is maximum
        color = 'Red' #default to red color if something isn't working right

        ##Initialize control commands
        controls = [-1,0,0,0]

        ##Create controls commands based on input from receiver
        if rcin.autopilot < 1500:
            #Manual control
            color = 'Green'
            controls[0] = rcin.throttlerc
            controls[1] = rcin.rollrc
            controls[2] = rcin.pitchrc
            controls[3] = rcin.yawrc   
        elif rcin.autopilot > 1500:
            #Autopilot
            color = 'Blue'
            controls[0] = 0.5
            controls[1] = 1
            controls[2] = -1
            controls[3] = 1
        return controls,defaults,color

    """
    def loop(self,RunTime,rcin,gps_llh,rpy,g,baro):
        #Find the distance and angle from the car to the next waypoint
        d_long = gps_llh.longitude - self.wp[self.wp_index, 0]
        d_lat = gps_llh.latitude - self.wp[self.wp_index, 1]
        dx = d_long * np.pi/180 * self.R
        dy = d_lat * np.pi/180 * self.R
        d = np.sqrt(dx**2 + dy**2) #Distance from car to waypoint

        controls = [-1,0,0,0] #-1 is minimum and 0 is mid

        #Compute the controller values
        if (rcin.autopilot < 2):
            #Manual control
            controls[0] = rcin.throttlerc
            controls[1] = rcin.rollrc
            controls[2] = rcin.pitchrc
            controls[3] = rcin.yawrc
        else: #Remember to test this part of code!
            #Inner Loop Control
            controls[0] = rcin.throttlerc
            kp = 0.01
            kd = 0.001
            kr = 1.0
            roll = rpy[0]
            pitch = rpy[1]
            roll_rate = g[0]
            pitch_rate = g[1]

            roll_error = kp*(roll - 0) + kd*(roll_rate - 0)
            controls[1] = 0 - roll_error
            controls[2] = 0 - kp*(pitch - 0) - kd*(pitch_rate - 0)
            controls[3] = 0 + kr*roll_error
        #print('Airplane code')

        ##Saturation blocks
        for i in range(0,self.NUMCONTROLS):
            if (controls[i] < -1):
                controls[i] = -1
            if (controls[i] > 1):
                controls[i] = 1

        return controls
    """
