import math
import numpy as np


class FORCES():
    def __init__(self):
        self.NUMMOTORS = 8
        self.RHOSLSI = 1.225
        self.GEARTH = 9.81
        self.PI = math.pi

        #Math and inertia
        self.mass = 0.735 #!mass  (kg)
        Ixx = 0.0146 #!Ixx    (kg-m^2)
        Iyy = 0.0146 #!Iyy    (kg-m^2)
        Izz = 0.0285 #!Izz     (kg-m^2)
        self.I = np.asarray([[Ixx,0,0],[0,Iyy,0],[0,0,Izz]])
        
        # Motor arrays
        self.thrust_motors = np.zeros(self.NUMMOTORS)
        self.torque_motors = np.zeros(self.NUMMOTORS)

        # Quadcopter Aerodynamic Parameters
        thrust_max = 0.4 * self.GEARTH
        self.Rrotor = (9.5 / 12.0) / (2.0 * 3.28)
        self.AREA = self.PI * pow(self.Rrotor, 2.0)
        
        Voltage = 3.7 * 4
        KV = 950.0
        omegaRPM = KV * Voltage
        # Following C++ implementation exactly: omegaRPM * 2.0 * PI / 180.0
        omegarads = omegaRPM * 2.0 * self.PI / 180.0
        self.spin_slope = omegarads / 2.0

        # Compute ct and cq
        self.ct = thrust_max / (0.5 * self.RHOSLSI * self.AREA * pow(self.Rrotor * omegarads, 2.0))
        self.cq = pow(self.ct, 3.0 / 2.0) / math.sqrt(2.0)

        # Distance from CG to rotor
        self.rx = 0.10115
        self.ry = 0.10115
        self.rz = 0.0508
        
        self.length = 1.0  # Used in Dryden model

    def compute_thrust_and_torque(self,commands):
        #print(commands)
        for i in range(self.NUMMOTORS):
            omega = self.spin_slope * (commands[i] + 1)
            thrust = 0.5 * self.RHOSLSI * self.AREA * pow(omega * self.Rrotor, 2.0) * self.ct
            torque = 0.5 * self.RHOSLSI * self.AREA * pow(omega * self.Rrotor, 2.0) * self.Rrotor * self.cq
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
        Force[2] = -thrust

        # Extract individual motor forces (Converting 1-based C++ MATLAB indexing to 0-based Python indices)
        motor_upper_left_bottom = self.thrust_motors[0]
        motor_upper_right_bottom = self.thrust_motors[1]
        motor_lower_right_bottom = self.thrust_motors[2]
        motor_lower_left_bottom = self.thrust_motors[3]
        motor_upper_left_top = self.thrust_motors[4]
        motor_upper_right_top = self.thrust_motors[5]
        motor_lower_right_top = self.thrust_motors[6]
        motor_lower_left_top = self.thrust_motors[7]

        # Extract individual motor torques
        torque_upper_left_bottom = self.torque_motors[0]
        torque_upper_right_bottom = self.torque_motors[1]
        torque_lower_right_bottom = self.torque_motors[2]
        torque_lower_left_bottom = self.torque_motors[3]
        torque_upper_left_top = self.torque_motors[4]
        torque_upper_right_top = self.torque_motors[5]
        torque_lower_right_top = self.torque_motors[6]
        torque_lower_left_top = self.torque_motors[7]

        # Compute yaw torques for bottom and top rotors
        yaw_torque_bottom = -torque_upper_left_bottom + torque_upper_right_bottom + torque_lower_left_bottom - torque_lower_right_bottom
        yaw_torque_top = torque_upper_left_top - torque_upper_right_top - torque_lower_left_top + torque_lower_right_top
        Moment[2] = yaw_torque_top + yaw_torque_bottom

        # Compute roll and pitch torques for top rotors
        roll_torque_top = (motor_upper_left_top + motor_lower_left_top) * self.ry - (motor_upper_right_top + motor_lower_right_top) * self.ry
        pitch_torque_top = (motor_upper_left_top + motor_upper_right_top) * self.rx - (motor_lower_right_top + motor_lower_left_top) * self.rx

        # Compute roll and pitch torques for bottom rotors
        roll_torque_bottom = (motor_upper_left_bottom + motor_lower_left_bottom) * self.ry - (motor_upper_right_bottom + motor_lower_right_bottom) * self.ry
        pitch_torque_bottom = (motor_upper_left_bottom + motor_upper_right_bottom) * self.rx - (motor_lower_right_bottom + motor_lower_left_bottom) * self.rx

        # Set total roll and pitch moments
        Moment[0] = roll_torque_top + roll_torque_bottom
        Moment[1] = pitch_torque_top + pitch_torque_bottom

        return Force,Moment