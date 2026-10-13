import math
import numpy as np


class AeroPack:

    def __init__(self):
        self.CLzero = 0.3441029736813229
        self.CLalpha = 5.185317204810547
        self.CLq = 5.9369349224934975
        self.CLdele = 0.41181051485504044
        self.CDzero = 0.01727851900786801
        self.CDalpha = 1.0080305157535787
        self.Cybeta = -0.24095762862664397
        self.Cydelr = 0.0688450367504697
        self.Cyp = -0.027733042384004744
        self.Cyr = 0.17663419386614151
        self.CLbeta = -0.01633611041536569
        self.CLp = -0.50160400189302223
        self.CLr = 0.09800094300447591
        self.CLdela = 0.28052396489578457
        self.CLdelr = 0.004667460118675911
        self.Cmzero = -0.07636788929236879
        self.Cmalpha = -2.1859941783855077
        self.Cmq = -24.450947269394277
        self.Cmdele = -1.1530526838451003
        self.Cnp = -0.78826431744966985
        self.Cnbeta = 0.27846397495417952
        self.Cnr = -0.33411253656820367
        self.Cndela = 0.061169396698422564
        self.Cndelr = -0.014305373822254552
        self.cbar = 0.2286  # Chord length (m)
        self.bws = 1.4859  # Wing span (m)
        self.S = self.cbar * self.bws
        self.mass = 1.2364
        self.GEARTH = 9.80665
        self.Tmax = self.mass * self.GEARTH
        self.max_speed = 30.0
        self.Rrotor = (9.5 / 12.0) / (2 * 3.28)
        self.AREA = math.pi * pow(self.Rrotor, 2.0)
        self.Voltage = 3.7 * 4
        self.KV = 950.0
        self.omegaRPM = self.KV * self.Voltage
        self.omegarads = self.omegaRPM * 2.0 * math.pi / 180.0
        self.Ixx = 0.23083252176875	#!Ixx(kg-m^2)
        self.Iyy = 0.12326181467115 	#!Iyy(kg-m^2)
        self.Izz = 0.261842767316082 #!Izz(kg-m^2)
        self.spin_slope = self.omegarads 
        self.RHOSLSI = 1.225  # Sea level density standard
        self.ct = self.Tmax / (
            0.5 * self.RHOSLSI * self.AREA * pow(self.Rrotor * self.omegarads, 2.0)
        )
        self.cq = pow(self.ct, 3.0 / 2.0) / math.sqrt(2.0)


class FORCES():

    def __init__(self):
        self.aeropack = AeroPack()
        self.mass = self.aeropack.mass
        self.I = np.asarray([[self.aeropack.Ixx, 0.0, 0.0], [0.0,self.aeropack.Iyy, 0.0], [0.0, 0.0, self.aeropack.Izz]])

    def ForceMoment(self,time,state,commands):
        """Calculates aerodynamic forces and body moments."""
        Force = np.asarray([0.,0.,0.])
        Moment = np.asarray([0.,0.,0.])

        # Extract States (Converting 1-based indexing to 0-based NumPy indices)
        z = state[2]
        u = state[7]
        v = state[8]
        w = state[9]
        p = state[10]
        q = state[11]
        r = state[12]

        # Streamline / Total velocity
        vinf = math.sqrt(u * u + v * v + w * w)

        # Angle of Attack and Sideslip
        beta = math.asin(v / vinf) if vinf > 0 else 0.0
        alpha = math.atan2(w, u)

        # Non-dimensional angular velocities
        phat = (p * self.aeropack.bws) / (2.0 * vinf) if vinf > 0 else 0.0
        qhat = (q * self.aeropack.cbar) / (2.0 * vinf) if vinf > 0 else 0.0
        rhat = (r * self.aeropack.bws) / (2.0 * vinf) if vinf > 0 else 0.0

        # Extract Actuator Values (PWM)
        mewt = commands[0]
        delaUS = commands[1]
        deleUS = commands[2]
        delrUS = commands[3]

        #print(commands)

        scalar_conv = 30.0*math.pi/180.0
        dela = scalar_conv * delaUS
        dele = scalar_conv * deleUS
        delr = scalar_conv * delrUS

        # Compute Thrust
        omega = self.aeropack.spin_slope * (mewt + 1)/2.0
        #print(omega)
        thrust = 0.5*self.aeropack.RHOSLSI*self.aeropack.AREA*pow(omega * self.aeropack.Rrotor, 2.0)*self.aeropack.ct
        if vinf < self.aeropack.max_speed:
            thrust *= vinf / self.aeropack.max_speed
        else:
            thrust = 0.0

        # Force Coefficients
        CL = (self.aeropack.CLzero + (self.aeropack.CLalpha * alpha) + (self.aeropack.CLq * qhat) + (self.aeropack.CLdele * dele))
        CD = self.aeropack.CDzero + (self.aeropack.CDalpha * (alpha * alpha))
        CY = ((self.aeropack.Cybeta * beta) + (self.aeropack.Cydelr * delr) + (self.aeropack.Cyp * phat) + (self.aeropack.Cyr * rhat))
        Cm = (self.aeropack.Cmzero + (self.aeropack.Cmalpha * alpha) + (self.aeropack.Cmq * qhat) + (self.aeropack.Cmdele * dele))
        Cl = ((self.aeropack.CLbeta * beta) + (self.aeropack.CLp * phat) + (self.aeropack.CLr * rhat) + (self.aeropack.CLdela * dela) + (self.aeropack.CLdelr * delr))
        Cn = ((self.aeropack.Cnp * phat) + (self.aeropack.Cnbeta * beta) + (self.aeropack.Cnr * rhat) + (self.aeropack.Cndela * dela) + (self.aeropack.Cndelr * delr))

        # Populate Forces & Moments
        qS = 0.5 * self.aeropack.RHOSLSI * (vinf * vinf) * self.aeropack.S

        Force[0] = (CL * math.sin(alpha) - (CD * math.cos(alpha))) * qS + thrust
        Force[1] = CY * qS
        Force[2] = (-CL * math.cos(alpha) - CD * math.sin(alpha)) * qS

        #print(Cm)

        Moment[0] = self.aeropack.bws * Cl * qS
        Moment[1] = self.aeropack.cbar*Cm*qS
        Moment[2] = self.aeropack.bws * Cn * qS

        #print(Moment[1],self.aeropack.cbar,Cm,qS,qS*Cm*self.aeropack.cbar)

        return Force,Moment