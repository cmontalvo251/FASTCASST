import numpy as np
import math

class FORCES():

    def __init__(self,mass,I):
        ##Define mass properties
        self.mass = mass
        self.I = I
        #self.mass = 4.66
        #Ixx = 0.056 #!Ixx //(kg-m^2)
        #Iyy = 0.035 #!Iyy //(kg-m^2)
        #Izz = 0.077 #!Izz //(kg-m^2)
        #Ixz = 0.0
        #Ixy = 0.0
        #Iyz = 0.0
        #self.I = np.asarray([[Ixx,Ixy,Ixz],[Ixy,Iyy,Iyz],[Ixz,Iyz,Izz]])

    def ForceMoment(self,time,state,commands):
        Force = np.asarray([0,0,0])
        Moment = np.asarray([0,0,0])

        # Extract Actuator Values
        motor1 = commands[0]
        motor2 = commands[1]

        # Extract States 
        x = state[0]
        u = state[7]
        v = state[8]
        w = state[9]
        p = state[10]
        q = state[11]
        r = state[12]

        # Friction and Dynamics Parameters
        XDAMPCOEFF = 7.26
        DAMPCOEFF = 50.0  # Guess and Check (y-direction)
        DAMPROTCOEFF = 2.0  # Guess and Check (yaw)
        d = 0.13335  # (m) - From wheel to center
        Vmax = 1.0  # (m/s) Max speed of the tank

        # Calculate Forces
        force_max = (((1.7) / 2.2) * 9.81 + 4.454) * 1.0
        s = 0.007681

        dpwm1 = motor1*500.0
        dpwm2 = motor2*500.0

        force1 = (math.copysign(1.0, dpwm1)*force_max*(1.0 - math.exp(-s * abs(dpwm1))))
        force2 = (math.copysign(1.0, dpwm2)*force_max*(1.0 - math.exp(-s * abs(dpwm2))))

        xforce = force1 + force2 - XDAMPCOEFF * u
        yforce = -DAMPCOEFF * v

        # Calculate Moments
        Nmoment = (force1 - force2) * d - DAMPROTCOEFF * r

        # Populate Forces
        Force[0] += xforce
        Force[1] += yforce

        # Populate Moments
        Moment[0] += 0.0
        Moment[1] += 0.0
        Moment[2] += Nmoment

        return Force,Moment
