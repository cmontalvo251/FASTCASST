import numpy as np

class FORCES():

    def __init__(self):
        ##Define mass properties
        self.mass = 1.0
        Ixx = 1.0
        Iyy = 2.0
        Izz = 3.0
        Ixz = 0.0
        Ixy = 0.0
        Iyz = 0.0
        self.I = np.asarray([[Ixx,Ixy,Ixz],[Ixy,Iyy,Iyz],[Ixz,Iyz,Izz]])

    def ForceMoment(self,time,state,commands):
        Force = np.asarray([0,0,0])
        Moment = np.asarray([0,0,0])

        # Friction Parameters
        d = 0.13335  # (m) - From wheel to center
        Vmax = 6.0  # (m/s) Max speed

        # Extract Actuator Values
        motor = commands[0]
        steering = commands[1]

        # Extract States 
        x = state[0]
        u = state[7]
        v = state[8]
        w = state[9]
        p = state[10]
        q = state[11]
        r = state[12]

        # Calculate Forces
        force_max = 50.0
        s = 0.007681
        dpwm = 500*(motor - 0)
        dsteer = 500*(steering - 0)

        steer_angle = (45.0 * np.pi / 180.0) * dsteer / 500.0
        force = np.sign(dpwm) * force_max * (1.0 - np.exp(-s * abs(dpwm)))
        xforce = force - 7.65 * u
        yforce = -10.0 * v + 0.0 * steer_angle

        # Calculate Moments
        Nmoment = 75.0 * (steer_angle - 0.4 * r)

        # Populate Forces
        Force[0] += xforce
        Force[1] += yforce
        Force[2] += 0.0

        # Populate Moments
        Moment[0] += 0.0
        Moment[1] += 0.0
        Moment[2] += Nmoment

        return Force,Moment
