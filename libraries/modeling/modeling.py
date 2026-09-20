import sys
import time
import numpy as np
import Datalogger.datalogger as D
import GPS.gps as G

class MODEL():
    def __init__(self,TIMESTEP,ICs,VEHICLE,NUMOUTPUTS,LATITUDE_ORIGIN,LONGITUDE_ORIGIN):
        self.timestep = TIMESTEP
        print('Running SIMONLY mode')
        self.state = np.zeros(13) #13 states
        self.state[0] = ICs[0] #x (m)
        self.state[1] = ICs[1] #y (m)
        self.state[2] = ICs[2] #z (m)
        ptp = np.asarray([ICs[3],ICs[4],ICs[5]])*np.pi/180.0 #convert to radians
        quat = self.euler2quat(ptp)
        self.state[3] = quat[0]
        self.state[4] = quat[1]
        self.state[5] = quat[2]
        self.state[6] = quat[3]
        self.state[7] = ICs[6] #u (m/s)
        self.state[8] = ICs[7] #v (m/s)
        self.state[9] = ICs[8] #w (m/s)
        self.state[10] = ICs[9]*np.pi/180.0 #p (rad/s)
        self.state[11] = ICs[10]*np.pi/180.0 #q (rad/s)
        self.state[12] = ICs[11]*np.pi/180.0 #r (rad/s)

        #Get mass and inertia props
        self.VEHICLE = VEHICLE
        sys.path.append('../libraries/V_'+self.VEHICLE)
        sys.path.append('libraries/V_'+self.VEHICLE)
        import forces
        self.vehicle = forces.FORCES()
        self.mass = self.vehicle.mass
        self.I = self.vehicle.I
        self.Iinv = np.linalg.inv(self.I)

        ##Setup logging
        self.logger = D.Datalogger('logs/',NUMOUTPUTS)
        headers = 'Time (sec) ,Model X(m) ,Model Y(m) ,Model Z(m) ,Model Roll (deg) ,Model Pitch (deg) ,Model Compass (deg) ,Model U(m/s) ,Model V(m/s) ,Model W(m/s) ,Model P(rad/s) ,Model Q(rad/s) ,Model R(rad/s) ,Model Mx(Gauss) ,Model My(Gauss) ,Model Mz(Gauss) ,Model GPS Latitude (deg) ,Model GPS Longitude (deg) ,Model GPS Altitude (m) ,Model GPS Heading (deg) ,Model IMU Heading (deg) ,Model Analog 1 (V) ,Model Analog 2 (V) ,Model Analog 3 (V) ,Model Analog 4 (V) ,Model Analog 5 (V) ,Model Analog 6 (V) ,Model Pressure (Pa) ,Model Pressure Altitude (m) ,Model Temperature (C) ,RC Channel #1 ,RC Channel #2 ,RC Channel #3 ,RC Channel #4 ,RC Channel #5'
        self.logger.writeheader(headers,'Model')

        #Setup GPS for lat/lon conversions
        self.gps = G.GPS(mode='SIMONLY')
        self.gps.setOrigin(LATITUDE_ORIGIN,LONGITUDE_ORIGIN)

        #Setup Barometer for pressure to altitude conversions
        import MS5611.ms5611 as MS
        self.baro = MS.MS5611(mode='SIMONLY')

    def log(self,RunTime):
        #Time (sec)
        self.logger.outdata[0] = np.round(RunTime,5)
        #X(m) Y(m) Z(m)
        self.logger.outdata[1] = self.state[0]
        self.logger.outdata[2] = self.state[1]
        self.logger.outdata[3] = self.state[2]
        #Roll (deg) ,Pitch (deg) , Compass (deg)
        ##Roll,Pitch, Yaw from quat 2 euler
        rpy = self.quat2euler(self.quat)
        self.logger.outdata[4] = rpy[0]*180/np.pi
        self.logger.outdata[5] = rpy[1]*180/np.pi
        self.logger.outdata[6] = rpy[2]*180/np.pi
        #U(m/s) ,V(m/s) ,W(m/s)
        self.logger.outdata[7] = self.state[7]
        self.logger.outdata[8] = self.state[8]
        self.logger.outdata[9] = self.state[9]
        #P(deg/s) ,Q(deg/s) ,R(deg/s)
        self.logger.outdata[11] = self.p*180/np.pi
        self.logger.outdata[12] = self.q*180/np.pi
        self.logger.outdata[13] = self.r*180/np.pi
        #Mx(Gauss) ,My(Gauss) ,Mz(Gauss) _ The model currently has no magnetometer measurements
        #we'll need to add it once we add the IGRF model for satellites
        self.logger.outdata[13] = 0.0
        self.logger.outdata[14] = 0.0
        self.logger.outdata[15] = 0.0
        #GPS Latitude (deg) ,GPS Longitude (deg) ,GPS Altitude (m)
        #Convert x,y,z
        if self.VEHICLE == 'satellite':
            #Convert to lat/lon/alt using polar coordinates
            self.latitude,self.longitude,self.altitude = self.gps.convertXY2LATLONSPHERICAL(self.state[0],self.state[1],self.state[2])
        else:
            #convert to lat/lon/alt using flat earth approx
            self.latitude,self.longitude,self.altitude = self.gps.convertXYZ2LATLON(self.state[0],self.state[1],self.state[2])
        self.logger.outdata[16] = self.latitude
        self.logger.outdata[17] = self.longitude
        self.logger.outdata[18] = self.altitude
        #GPS Heading (deg) ,IMU Heading (deg)
        self.logger.outdata[19] = rpy[2]
        self.logger.outdata[20] = rpy[2]
        #Analog 1-6 (V)
        self.logger.outdata[21] = 0
        self.logger.outdata[22] = 0
        self.logger.outdata[23] = 0
        self.logger.outdata[24] = 0
        self.logger.outdata[25] = 0
        self.logger.outdata[26] = 0
        #Pressure (Pa) #Pressure Altitude (m) #Temperature (C)
        #Barometer
        z = self.state[2]
        if self.VEHICLE == 'satellite':
            x = self.state[0]
            y = self.state[1]
            rho = np.sqrt(x**2 + y**2 + z**2)
            REARTH = 6371000.0
  	    #This is a satellite
            self.baro.ALT = rho - REARTH
        else:
            self.baro.ALT = -z
        self.baro.convertAltitude2Pressure()
        self.logger.outdata[27] = self.baro.PRES
        self.logger.outdata[28] = self.baro.ALT
        self.logger.outdata[29] = self.baro.TEMP
        #RC Channel #1-5
        self.logger.outdata[30] = self.rcsignals[0]
        self.logger.outdata[31] = self.rcsignals[1]
        self.logger.outdata[32] = self.rcsignals[2]
        self.logger.outdata[33] = self.rcsignals[3]
        self.logger.outdata[34] = self.rcsignals[4]
        self.logger.outdata[35] = self.rcsignals[5]
        #PWM Hardware Out 1-len(pwm_commands)
        for i in range(0,len(self.pwm_commands)):
            self.logger.outdata[36+i] = self.pwm_commands[i]
        self.logger.println()

    def Derivatives(self,t,dstate):
        #Need to compute statedot
        #x = state[0]
        #y = state[1]
        #z = state[2]
        #phi = state[3]
        #theta = state[4]
        #psi = state[5]
        q0 = dstate[3]
        q1 = dstate[4]
        q2 = dstate[5]
        q3 = dstate[6]
        self.quat = np.asarray([q0,q1,q2,q3])
        self.u = dstate[7]
        self.v = dstate[8]
        self.w = dstate[9]
        self.p = dstate[10]
        self.q = dstate[11]
        self.r = dstate[12]

        #Set up vectors
        uvw = np.asarray([self.u,self.v,self.w])
        pqr = np.asarray([self.p,self.q,self.r])
        
        #Kinematics
        TIB = self.RQUAT(self.quat)
        xyzdot = np.matmul(TIB,uvw)
        PQRMAT = np.asarray([[0,-self.p,-self.q,-self.r],[self.p,0,self.r,-self.q],[self.q,-self.r,0,self.p],[self.r,self.q,-self.p,0]])
        quatdot = 0.5*np.matmul(PQRMAT,self.quat)
        
        #Force and Moment Model 
        F = np.asarray([0,0,0])
        M = np.asarray([0,0,0])
        F,M = self.vehicle.ForceMoment(t,dstate,self.pwm_commands)
        
        #Dynamics
        uvwdot = F/self.mass - np.cross(pqr,uvw)
        pqrdot = np.matmul(self.Iinv,M-np.cross(pqr,np.matmul(self.I,pqr)))

        dxdt = np.concatenate([xyzdot,quatdot,uvwdot,pqrdot])

        return dxdt

    def loop(self,t,rcsignals,pwm_commands):
        self.pwm_commands = pwm_commands
        self.rcsignals = rcsignals
        #RK4 Call
        k1 = self.Derivatives(t,self.state)
        k2 = self.Derivatives(t+self.timestep/2.0,self.state+k1*self.timestep/2.0)
        k3 = self.Derivatives(t+self.timestep/2.0,self.state+k2*self.timestep/2.0)
        k4 = self.Derivatives(t+self.timestep,self.state+k3*self.timestep)
        self.statedot = (1.0/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        #Step State
        self.state += self.statedot*self.timestep

    def euler2quat(self,ptp):
        #%%%Input is a 3x1 vector and output is a 4x1 vector

        phi = ptp[0];
        theta = ptp[1]
        psi = ptp[2]

        q0 = np.cos(phi/2)*np.cos(theta/2)*np.cos(psi/2) + np.sin(phi/2)*np.sin(theta/2)*np.sin(psi/2);
        q1 = np.sin(phi/2)*np.cos(theta/2)*np.cos(psi/2) - np.cos(phi/2)*np.sin(theta/2)*np.sin(psi/2);
        q2 = np.cos(phi/2)*np.sin(theta/2)*np.cos(psi/2) + np.sin(phi/2)*np.cos(theta/2)*np.sin(psi/2);
        q3 = np.cos(phi/2)*np.cos(theta/2)*np.sin(psi/2) - np.sin(phi/2)*np.sin(theta/2)*np.cos(psi/2);

        return np.asarray([q0,q1,q2,q3]);
            
    def quat2euler(self,q0123):
        q0 = q0123[0]
        q1 = q0123[1]
        q2 = q0123[2]
        q3 = q0123[3]

        phi = (np.arctan2(2*(q0*q1 + q2*q3),1-2*(q1**2 + q2**2)))
        theta = np.arcsin(2*(q0*q2-q3*q1))
        psi = np.arctan2(2*(q0*q3 + q1*q2),1-2*(q2**2 + q3**2))

        return np.asarray([phi,theta,psi])

    def extract_Euler(self,T):
        #%%%Assuming R is a 3x3 matrix extract phi,theta,psi Euler angles
        #%%%assuming a 3-2-1 transformation sequence
        #%%%Let R be defined such that v(body) = T v(inertial)
        #Using Mark Costello's notation this would be TBI

        theta = -np.arcsin(T[0][2]);
        sphi  = T[1][2]/np.cos(theta);
        cphi = T[2][2]/np.cos(theta);
        phi = np.arctan2(sphi,cphi);
        spsi = T[0][1]/np.cos(theta);
        cpsi = T[0][0]/np.cos(theta);
        psi = np.arctan2(spsi,cpsi);

        ptp = np.zeros(3)

        ptp[0] = phi
        ptp[1] = theta
        ptp[2] = psi

        return ptp

    def extract_quaternion(self,TBI):
        #%%%Assuming T is a 3x3 matrix, extract the quaternion vector (q0,q1,q2,q3)
        #%%%assuming a 3-2-1 transformation sequence
        #%%%Let T be defined such that v(body) = T v(inertial)
        #%%In Mark Costello's notation, T would TBI
        alfa = TBI[0,1] + TBI[1,0]
        bata = TBI[2,0] + TBI[0,2]
        gama = TBI[2,1] + TBI[1,2]
        q1squared = alfa*bata/(4.0*gama);

        #%%%Solutions split into two different solutions here
        q1a = np.sqrt(q1squared);
        q1b = -np.sqrt(q1squared);
        q2a = alfa/(4.0*q1a);
        q2b = alfa/(4.0*q1b);
        q3a = bata/(4.0*q1a);
        q3b = bata/(4.0*q1b);

        #%%%These are the same so just pick one
        #%q0asquared = TBI(1,1) + q2a^2 + q3a^2 - q1a^2
        #%q0bsquared = TBI(1,1) + q2b^2 + q3b^2 - q1b^2
        q0squared = TBI[0,0] + q2a**2 + q3a**2 - q1a**2

        #%%%Solution However still splits into 4 possible solutions
        #You can get around this though by enforcing q0a to be positive
        #See this article here %%http://planning.cs.uiuc.edu/node151.html
        q0a = np.sqrt(q0squared);
        #q0b = -sqrt(q0squared);

        #%%%Here are my 2 possible solutions
        q0123aa = np.asarray([q0a,q1a,q2a,q3a])
        q0123ab = np.asarray([q0a,q1b,q2b,q3b])
        
        #%According to this website there are multiple solutions 
        #%that yield the same euler angles. 
        #%http://planning.cs.uiuc.edu/node151.html
        #%So what we want to do is compute the euler angles from the matrix
        ptp = self.extract_Euler(TBI)
        #ptp = [phi,theta,psi];
        #%Then check and see which have the same euler angles
        #because you restricted q0 to be positive above though.
        #This will only result in one unique solution from the 2.
        #Thus you can break out of this loop as soon as you find it.
        quats = [q0123aa,q0123ab]
        for q0123j in quats:
            ptpj = self.quat2euler(q0123j)
            val = abs(sum(ptpj-ptp))
            if val < 1e-10:
                return q0123j

    def RQUAT(self,q0123):
        #%compute R such that v(inertial) = R v(body)
        #%Using Mark Costello's notation this would be TIB

        q0 = q0123[0]
        q1 = q0123[1]
        q2 = q0123[2]
        q3 = q0123[3]

        R = np.asarray([[q0**2+q1**2-q2**2-q3**2,2*(q1*q2-q0*q3),2*(q0*q2+q1*q3)],[2*(q1*q2+q0*q3),(q0**2-q1**2+q2**2-q3**2),2*(q2*q3-q0*q1)],[2*(q1*q3-q0*q2),2*(q0*q1+q2*q3),q0**2-q1**2-q2**2+q3**2]])

        return R

    def R123(self,phi,theta,psi):
        #%compute R such that v(inertial) = R v(body)
        #%Compute sines and cosines
        ctheta = np.cos(theta);
        stheta = np.sin(theta);
        sphi = np.sin(phi);
        cphi = np.cos(phi);
        spsi = np.sin(psi);
        cpsi = np.cos(psi);
        #%Kinematics
        R = np.array([[ctheta*cpsi,sphi*stheta*cpsi-cphi*spsi,cphi*stheta*cpsi+sphi*spsi],[ctheta*spsi,sphi*stheta*spsi+cphi*cpsi,cphi*stheta*spsi-sphi*cpsi],[-stheta,sphi*ctheta,cphi*ctheta]]);
        return R
