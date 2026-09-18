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

    def ForceMoment(self,t,state):
        Force = np.asarray([0,0,0])
        Moment = np.asarray([0,0,0])
        return Force,Moment

