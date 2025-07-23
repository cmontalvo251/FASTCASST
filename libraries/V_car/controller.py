import numpy as np

class CONTROLLER():

    def __init__(self):
        #Waypoints
        self.wp = np.array([[-88.1755, 30.6906], [-88.1750, 30.6902], [-88.1745, 30.6906]])
        self.wp_index = 0 #Keeps track of which waypoint is being used
        self.wp_index_max = len(self.wp)
        self.R = 6371*10**3 #Radius of the Earth in m
        self.NUMCONTROLS = 2
        return

    def loop(self,RunTime,rcin,gps_llh,rpy,g,baro):
        #Find the distance and angle from the car to the next waypoint
        d_long = gps_llh.longitude - self.wp[self.wp_index, 0]
        d_lat = gps_llh.latitude - self.wp[self.wp_index, 1]
        dx = d_long * np.pi/180 * self.R
        dy = d_lat * np.pi/180 * self.R
        d = np.sqrt(dx**2 + dy**2) #Distance from car to waypoint

        controls = [-1,0] #-1 is minimum and 0 is mid

        #Compute the controller values
        if (rcin.autopilot < 2):
            #Manual control
            controls[0] = rcin.throttlerc
            controls[1] = rcin.rollrc
        else: #Remember to test this part of code!
            controls[0] = 0 #Throttle in the middle
            if self.wp_index >= self.wp_index_max:
                controls[0] = -1 #turn motors off 
                controls[1] = 0 #set wheels to straight
            if (d<=20): #If the distance is less than or equal to 20 m...
                controls[1] = 1 #Turns the front wheels left
                time.sleep(1) #Stops the car to signal it's reached its waypoint
                self.wp_index += 1
            else: #If the distance is greater than 20 m...
                controls[1] = 0 #... keep the wheels straight

        ##Saturation blocks
        for i in range(0,self.NUMCONTROLS):
            if (controls[i] < -1):
                controls[i] = -1
            if (controls[i] > 1):
                controls[i] = 1

        return controls