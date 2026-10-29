import numpy as np

class STARTRACKER():
    #This is a simply bare bones star tracker class. I don't want to program 
    #the entire quest algorithm in here so for now we're just going to assume it gives 
    #us a nice clean rpy signal 
    
    def __init__(self):
        self.rpy = np.zeros(3)

    def send(self,state):
        q0 = state[3]
        q1 = state[4]
        q2 = state[5]
        q3 = state[6]
        phi = (np.arctan2(2*(q0*q1 + q2*q3),1-2*(q1**2 + q2**2)))
        theta = np.arcsin(2*(q0*q2-q3*q1))
        psi = np.arctan2(2*(q0*q3 + q1*q2),1-2*(q2**2 + q3**2))
        self.rpy[0] = phi*180./np.pi
        self.rpy[1] = theta*180./np.pi
        self.rpy[2] = psi*180/np.pi

    def get(self):
        return self.rpy

