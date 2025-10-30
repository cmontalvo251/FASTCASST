"""AHRS Filter Converted from C++ to Python
Aramis Hoffmann
4/28/25"""

import numpy as np

class AHRS:
##============================= Initial setup =================================
    def __init__(self):
        self.AHRS()
        
    def AHRS(self):

        self.q0 = 1; self.q1 = 0; self.q2 = 0; self.q3 = 0; self.twoKi = 0; self.twoKp = 2;
  
    def update(self, ax, ay, az, gx, gy, gz, mx, my, mz, elapsedTime):

        ## Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0) and (my == 0.0) and (mz == 0.0)):
            self.updateNOMAG(ax,ay,az,gx,gy,gz,elapsedTime);
            return;

    ## Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(not((ax == 0.0) and (ay == 0.0) and (az == 0.0))):

            ## Normalise accelerometer measurement
            recipNorm = self.invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            ## Normalise magnetometer measurement
            recipNorm = self.invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            ## Auxiliary variables to avoid repeated arithmetic
            q0q0 = self.q0 * self.q0;
            q0q1 = self.q0 * self.q1;
            q0q2 = self.q0 * self.q2;
            q0q3 = self.q0 * self.q3;
            q1q1 = self.q1 * self.q1;
            q1q2 = self.q1 * self.q2;
            q1q3 = self.q1 * self.q3;
            q2q2 = self.q2 * self.q2;
            q2q3 = self.q2 * self.q3;
            q3q3 = self.q3 * self.q3;

            ## Reference direction of Earth's magnetic field
            hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = np.sqrt(hx * hx + hy * hy);
            bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2));
            
            # Estimated direction of gravity and magnetic field
            halfvx = q1q3 - q0q2;
            halfvy = q0q1 + q2q3;
            halfvz = q0q0 - 0.5 + q3q3;
            halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2);

            ## Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

            ## Compute and apply integral feedback if enabled
            if(self.twoKi > 0.0): 
                self.integralFBx += self.twoKi * halfex * elapsedTime;	# integral error scaled by Ki
                self.integralFBy += self.twoKi * halfey * elapsedTime;
                self.integralFBz += self.twoKi * halfez * elapsedTime;
                gx += self.integralFBx;	## apply integral feedback
                gy += self.integralFBy;
                gz += self.integralFBz;
            else:
                self.integralFBx = 0.0;	# prevent integral windup
                self.integralFBy = 0.0;
                self.integralFBz = 0.0;
    
            # Apply proportional feedback
            gx += self.twoKp * halfex;
            gy += self.twoKp * halfey;
            gz += self.twoKp * halfez;
  

        # Integrate rate of change of quaternion
        gx *= (0.5 * elapsedTime);		# pre-multiply common factors
        gy *= (0.5 * elapsedTime);
        gz *= (0.5 * elapsedTime);
        qa = self.q0;
        qb = self.q1;
        qc = self.q2;
        self.q0 += (-qb * gx - qc * gy - self.q3 * gz);
        self.q1 += (qa * gx + qc * gz - self.q3 * gy);
        self.q2 += (qa * gy - qb * gz + self.q3 * gx);
        self.q3 += (qa * gz + qb * gy - qc * gx);

        # Normalise quaternion
        recipNorm = self.invSqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3);
        self.q0 *= recipNorm;
        self.q1 *= recipNorm;
        self.q2 *= recipNorm;
        self.q3 *= recipNorm;

    def updateNOMAG(self, ax, ay, az, gx, gy, gz, elapsedTime):

        #printf(" gx,gy,gz A = %lf %lf %lf ",gx,gy,gz);
        #printf("Raw Accelerometer %lf %lf %lf \n",ax,ay,az);
        self.GEARTH = 9.80665 
        ax /= self.GEARTH;
        ay /= self.GEARTH;
        az /= self.GEARTH;

        # Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(not((ax == 0.0) and (ay == 0.0) and (az == 0.0))):

            # Normalise accelerometer measurement
            recipNorm = self.invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            # Estimated direction of gravity and vector perpendicular to magnetic flux
            halfvx = self.q1 * self.q3 - self.q0 * self.q2;
            halfvy = self.q0 * self.q1 + self.q2 * self.q3;
            halfvz = self.q0 * self.q0 - 0.5 + self.q3 * self.q3;

            # Error is sum of cross product between estimated and measured direction of gravity
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            # Compute and apply integral feedback if enabled
            if(self.twoKi > 0.0):
                self.integralFBx += self.twoKi * halfex * elapsedTime;	# integral error scaled by Ki
                self.integralFBy += self.twoKi * halfey * elapsedTime;
                self.integralFBz += self.twoKi * halfez * elapsedTime;
                gx += self.integralFBx;	# apply integral feedback
                gy += self.integralFBy;
                gz += self.integralFBz;
    
            else:
                self.integralFBx = 0.0;	# prevent integral windup
                self.integralFBy = 0.0;
                self.integralFBz = 0.0;
    

            # Apply proportional feedback
            gx += self.twoKp * halfex;
            gy += self.twoKp * halfey;
            gz += self.twoKp * halfez;
  

        # Integrate rate of change of quaternion
        gx *= (0.5 * elapsedTime);		# pre-multiply common factors
        gy *= (0.5 * elapsedTime);
        gz *= (0.5 * elapsedTime);
        qa = self.q0;
        qb = self.q1;
        qc = self.q2;
        self.q0 += (-qb * gx - qc * gy - self.q3 * gz);
        self.q1 += (qa * gx + qc * gz - self.q3 * gy);
        self.q2 += (qa * gy - qb * gz + self.q3 * gx);
        self.q3 += (qa * gz + qb * gy - qc * gx);

        # Normalise quaternion
        recipNorm = self.invSqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3);
        self.q0 *= recipNorm;
        self.q1 *= recipNorm;
        self.q2 *= recipNorm;
        self.q3 *= recipNorm;

    def setGyroOffset(self, offx, offy, offz):

        self.gyroOffset[0] = offx;
        self.gyroOffset[1] = offy;
        self.gyroOffset[2] = offz;


    def setQuaternions(self, _q0, _q1, _q2, _q3):
        self.q0 = _q0;
        self.q1 = _q1;
        self.q2 = _q2;
        self.q3 = _q3;


    def getEuler(self): #(self, roll, pitch, yaw):
        
        #printf("Q = %lf %lf %lf %lf ",q0,q1,q2,q3);
        roll = np.arctan2(2*(self.q0*self.q1+self.q2*self.q3), 1-2*(self.q1*self.q1+self.q2*self.q2)) * 180.0/np.pi;
        pitch = np.arcsin(2*(self.q0*self.q2-self.q3*self.q1)) * 180.0/np.pi;
        yaw = -np.arctan2(2*(self.q0*self.q3+self.q1*self.q2), 1-2*(self.q2*self.q2+self.q3*self.q3)) * 180.0/np.pi;
        return roll, pitch, yaw

    def invSqrt(self, x):
        return x**(-.5);

    def getW(self):
        return  self.q0;

    def getX(self):
        return  self.q1;

    def getY(self):
        return  self.q2;

    def getZ(self):
        return  self.q3;