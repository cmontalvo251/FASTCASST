import math
import numpy as np

class CONTROLLER():

    def __init__(self,WAYPOINTS,CONTROLMODE):
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
        self.NUMCONTROLS = self.NUMTORQUERS + self.NUMRWS

        # PWM / Signal Constants
        self.CONTROLMODE = abs(CONTROLMODE)
        self.pqr = np.zeros(3)
        self.desired_moments = np.zeros(3)

        self.elapsedTime = 0.0
        self.lastTime = 0.0
        self.Irw = (self.MASSRW / 2.0) * self.RADRW * self.RADRW
        self.setdefaults()                

    def setdefaults(self):
        # First 3 are magnetorquers (magTs)
        # Next ones are reaction wheel speeds
        self.controls = [0.,0.,0.,0.,0.,0.]
        self.defaults = [0.,0.,0.,0.,0.,0.]
        self.color = 'Red'

    def loop(self, RunTime, rcin, gps_llh, rpy, g, m):
        """Main control loop executed at each timestep."""
        # Default Control Signals
        self.elapsedTime = RunTime - self.lastTime
        self.lastTime = RunTime
        self.setdefaults()

        if rcin.autopilot < 1500:
            #Manual control
            self.color = 'Green'
            #Control the reaction wheels
            self.controls[3] = rcin.rollrc
            self.controls[4] = rcin.pitchrc
            self.controls[5] = rcin.yawrc
        elif rcin.autopilot > 1500:
            self.color = 'Blue'
            # Run control loop if active
            if self.CONTROLMODE == 1:
                # Extract PQR in (deg/s) from sense_matrix
                self.pqr[0] = g[0]
                self.pqr[1] = g[1]
                self.pqr[2] = g[2]
                # Convert to (rad/s)
                self.pqr *= math.pi / 180.0

                # Check angular velocity norm for detumbling / reaction wheel spinup
                if np.linalg.norm(self.pqr) < 1.0*np.pi/180.0:
                    # Damping control: M_desired = -KD * (pqr_command - pqr_nav), assuming pqr_command = 0
                    KD = -0.0001
                    desired_moments = self.pqr * KD / self.Irw

                    # Place into control matrix for reaction wheels (indices 3 to 5)
                    self.controls[3] = desired_moments[0]
                    self.controls[4] = desired_moments[1]
                    self.controls[5] = desired_moments[2]
                else:   
                    # Extract Magnetometer Readings
                    # mu_ideal = k * (omega cross B)
                    desired_moments = np.cross(self.pqr,m)
                    desired_moments *= self.KMAG
                    # Convert to currents
                    desired_moments /= self.NUMTURNS * self.AREA
                    # Send to control matrix actuators
                    for i in range(self.NUMTORQUERS):
                        self.controls[i] = desired_moments[i]
            elif self.CONTROLMODE == 0:
                #Then we just set all controls to zero so we just pass
                pass

        #print(self.controls)
            
        return self.controls,self.defaults,self.color