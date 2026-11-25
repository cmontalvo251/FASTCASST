import math
import numpy as np


class FORCES():
    def __init__(self,mass,I):
        self.NUMMOTORS = 4
        self.RHOSLSI = 1.225
        self.GEARTH = 9.81
        self.PI = math.pi

        #Math and inertia
        self.mass = mass
        self.I = I
        
        # Motor arrays
        self.thrust_motors = np.zeros(self.NUMMOTORS)
        self.torque_motors = np.zeros(self.NUMMOTORS)

        # Quadcopter Aerodynamic Parameters
        self.Rrotor = (9.0/ 12.0) / (2.0 * 3.28)
        self.AREA = self.PI * pow(self.Rrotor, 2.0)
        
        #Spin Slope
        self.eta = 0
        omegarads = 1250.0 
        self.spin_slope = omegarads / 2.0

        # Compute ct and cq
        self.ct = 0.0071
        self.cq = pow(self.ct, 3.0 / 2.0) / math.sqrt(2.0)

        # Distance from CG to rotor
        self.rx = 0.133
        self.ry = 0.214
        
        self.length = 1.0  # Used in Dryden model

    def compute_thrust_and_torque(self,commands):
        #print(commands)
        self.eta = 0 
        for i in range(self.NUMMOTORS):
            omega = self.spin_slope * (commands[i] + 1)
            self.eta += omega*self.Rrotor
            thrust = self.RHOSLSI * self.AREA * pow(omega * self.Rrotor, 2.0) * self.ct
            torque = self.RHOSLSI * self.AREA * pow(omega * self.Rrotor, 2.0) * self.Rrotor * self.cq
            self.thrust_motors[i] = thrust
            self.torque_motors[i] = torque
        #print(self.thrust_motors)

    def ForceMoment(self,time,state,commands):
        """Calculates forces and moments for the X8 quadcopter in the body frame."""
        Force = np.asarray([0.,0.,0.])
        Moment = np.asarray([0.,0.,0.])

        # Compute individual motor thrusts and torques
        self.compute_thrust_and_torque(commands)

        #if time > 20:
        #    print(time,commands,self.thrust_motors)

        # Thrust on the body is simply the total thrust (applied along the negative z-axis in body frame)
        thrust = np.sum(self.thrust_motors)
        A1C = 0.01
        dx = 0.01
        A1S = 0.01
        dy = 0.01
        u = state[7]
        v = state[8]
        Force[0] = -thrust*((A1C/self.eta+dx)*u-A1S*v);
        Force[1] = -thrust*((A1S/self.eta)*u+(A1C/self.eta+dy)*v);
        Force[2] = -thrust

        # Extract individual motor forces (Converting 1-based C++ MATLAB indexing to 0-based Python indices)
        motor_upper_left_bottom = self.thrust_motors[0]
        motor_upper_right_bottom = self.thrust_motors[1]
        motor_lower_right_bottom = self.thrust_motors[2]
        motor_lower_left_bottom = self.thrust_motors[3]

        # Extract individual motor torques
        torque_upper_left_bottom = self.torque_motors[0]
        torque_upper_right_bottom = self.torque_motors[1]
        torque_lower_right_bottom = self.torque_motors[2]
        torque_lower_left_bottom = self.torque_motors[3]

        # Compute yaw torques for bottom and top rotors
        yaw_torque_bottom = -torque_upper_left_bottom + torque_upper_right_bottom + torque_lower_left_bottom - torque_lower_right_bottom
        Moment[2] = yaw_torque_bottom

        # Compute roll and pitch torques for bottom rotors
        roll_torque_bottom = (motor_upper_left_bottom + motor_lower_left_bottom) * self.ry - (motor_upper_right_bottom + motor_lower_right_bottom) * self.ry
        pitch_torque_bottom = (motor_upper_left_bottom + motor_upper_right_bottom) * self.rx - (motor_lower_right_bottom + motor_lower_left_bottom) * self.rx

        # Set total roll and pitch moments
        Moment[0] = roll_torque_bottom
        Moment[1] = pitch_torque_bottom

        return Force,Moment