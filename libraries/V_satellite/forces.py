import math
import numpy as np

class FORCES:

    def __init__(self):
        # ==========================================
        # Parameters (Equivalent to params.h)
        # ==========================================
        self.KMAG = 67200
        self.NUMTURNS = 84
        self.AREA = 0.02
        self.MAXCURRENT = 0.12
        self.NUMTORQUERS = 3  # X, Y, Z active torquers
        self.NUMRWS = 3  # Reaction wheels
        self.RADRW = 0.042  # Radius of reaction wheel (m)
        self.MAXMOMENT = 0.015  # Maximum momentum Nms
        self.MASSRW = 0.13  # Mass of reaction wheel (kg)
        self.MAXTORQUE = 0.004  # Maximum Torque N-m

        # Initialize 3x1 body force and moment vectors
        self.omega_RWS = np.zeros(3)

        # Parameters & Constants
        self.dOmega_max = 10.0
        self.dPWM = 2.0
        self.IpwmC = self.dOmega_max / self.dPWM

        self.prev_time = 0.0
        self.Irw = (self.MASSRW / 2.0) * self.RADRW * self.RADRW
        self.MAXACCEL = self.MAXTORQUE / self.Irw
        self.MAXOMEGA = self.MAXMOMENT / self.Irw

        self.mass = 3.0	#!Mass (kg)
        self.Ixx = 0.005 # !Ixx (kg-m^2)
        self.Iyy = 0.011 #!Iyy(kg-m^2)
        self.Izz = 0.011 #!Izz(kg-m^2)
        self.I = np.asarray([[self.Ixx, 0.0, 0.0], [0.0,self.Iyy, 0.0], [0.0, 0.0, self.Izz]])

    def ForceMoment(self,time,state,commands,mag):
        """Calculates forces and moments for the satellite in the body frame."""
        # Zero out force and moment vectors
        Force = np.asarray([0.,0.,0.])
        Moment = np.asarray([0.,0.,0.])
        MMTVEC = np.asarray([0.,0.,0.])

        # Compute magnetorquer current and moment vector
        for i in range(3):
            if i < self.NUMTORQUERS:
                pwm_offset = commands[i]
                current = self.IpwmC * pwm_offset
                MMTVEC[i] = current
            else:
                MMTVEC[i] = 0.0

        # Saturation on current
        current_sum = np.sum(np.abs(MMTVEC))
        if current_sum > self.MAXCURRENT:
            MMTVEC *= self.MAXCURRENT / current_sum
        MMTVEC *= self.AREA * self.NUMTURNS

        # Compute total magnetic torque placed on the
        # satellite (cross product of magnetic moment
        # and B field)
        cross_res = np.cross(MMTVEC,mag)
        Moment[0] = cross_res[0]
        Moment[1] = cross_res[1]
        Moment[2] = cross_res[2]

        # Add torque from reaction wheels 
        for i in range(3, 6):
            pwm_offset = commands[i]
            rw_idx = i - 3
            #Check for saturation on angular velocity
            if (pwm_offset > 0) and (self.omega_RWS[rw_idx] >= self.MAXOMEGA):
                pwm_offset = 0.0
            if (pwm_offset < 0) and (self.omega_RWS[rw_idx] <= -self.MAXOMEGA):
                pwm_offset = 0.0

            # Convert PWM offset to angular acceleration (rad/s^2)
            alfa = pwm_offset / self.dPWM * self.MAXACCEL

            # Integrate angular velocity
            self.omega_RWS[rw_idx] += alfa * (time - self.prev_time)

            # Compute torque based on angular acceleration and add to body moment
            Moment[rw_idx] += alfa * self.Irw

        # Reset time
        self.prev_time = time

        return Force,Moment