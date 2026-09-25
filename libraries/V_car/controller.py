import numpy as np

class CONTROLLER():
    def __init__(self,WAYPOINTS):
        self.Kp_steer = 0.008
        self.base_throttle = 0.4
        self.NUMCONTROLS = 2
        self.setdefaults()
        self.WAYPOINTS = WAYPOINTS
        return

    def setdefaults(self):
        self.defaults = [0, 0]   #-1 is minimum and 0 is mid, 1 is maximum
        self.controls = [0, 0]
        self.color = 'Red'

    def loop(self, RunTime, rcin, gps_llh, rpy, g, baro):
        self.setdefaults()
        if rcin.autopilot < 1500:
            ##Manual control
            self.color = 'Green'
            self.controls[0] = rcin.throttlerc
            self.controls[1] = rcin.rollrc
        elif rcin.autopilot > 1500:
            #Autopilot mode
            self.controls = [0, 0]
            self.color = 'Blue'

        ##Saturation
        for i in range(0, self.NUMCONTROLS):
            if self.controls[i] < -1:
                self.controls[i] = -1
            if self.controls[i] > 1:
                self.controls[i] = 1

        #print(self.controls)
        return self.controls, self.defaults, self.color

"""Imported from Gemini

import math
import numpy as np


def delpsi(psi1: float, psi2: float) -> float:
    """Calculates relative angle wrapped between -pi and pi."""
    dpsi = psi2 - psi1
    return math.atan2(math.sin(dpsi), math.cos(dpsi))


class Controller:

    def __init__(
        self,
        num_signals: int = 2,
        stick_min: float = 1000.0,
        stick_mid: float = 1500.0,
        stick_max: float = 2000.0,
        out_min: float = 1000.0,
        out_mid: float = 1500.0,
        out_max: float = 2000.0,
    ):
        # Constants
        self.NUMSIGNALS = num_signals
        self.STICK_MIN = stick_min
        self.STICK_MID = stick_mid
        self.STICK_MAX = stick_max
        self.OUTMIN = out_min
        self.OUTMID = out_mid
        self.OUTMAX = out_max

        # State Variables
        self.control_matrix = np.zeros((self.NUMSIGNALS, 1))
        self.CONTROLLER_FLAG = 0
        self.NUMWAYPOINTS = 0
        self.WAYPOINTS_X = []
        self.WAYPOINTS_Y = []
        self.WAYINDEX = 0

        self.lastTime = 0.0
        self.elapsedTime = 0.0
        self.PRINTER = 0

        # Control State Variables
        self.throttle = self.STICK_MID
        self.aileron = self.STICK_MID
        self.autopilot = 0.0

        self.velocity_command = -99.0
        self.heading_command = -99.0
        self.velocityint = 0.0

    def init(self, in_configuration_matrix: np.ndarray):
        """Initializes controller configuration from matrix input."""
        self.control_matrix = np.zeros((self.NUMSIGNALS, 1))
        self.set_defaults()

        print("Controller Received Configuration Matrix")

        # 1-based index (11,1) translates to 0-based index [10, 0]
        self.CONTROLLER_FLAG = int(in_configuration_matrix[10, 0])

        if in_configuration_matrix.shape[0] >= 12:
            # 1-based index (12,1) translates to 0-based index [11, 0]
            self.NUMWAYPOINTS = int(in_configuration_matrix[11, 0])
            self.WAYPOINTS_X = [0.0] * self.NUMWAYPOINTS
            self.WAYPOINTS_Y = [0.0] * self.NUMWAYPOINTS

            idx = 12  # 0-based starting index for waypoint data
            for i in range(self.NUMWAYPOINTS):
                self.WAYPOINTS_X[i] = float(in_configuration_matrix[idx, 0])
                idx += 1
                self.WAYPOINTS_Y[i] = float(in_configuration_matrix[idx, 0])
                idx += 1

        print(
            f"Controller Setup. CONTROLLER_FLAG = {self.CONTROLLER_FLAG} \n"
        )

    def set_defaults(self):
        """Resets control matrix outputs to mid positions."""
        self.control_matrix[0, 0] = self.OUTMID
        self.control_matrix[1, 0] = self.OUTMID

    def print(self):
        """Prints current control signals."""
        output_str = " ".join(
            str(int(self.control_matrix[i, 0])) for i in range(self.NUMSIGNALS)
        )
        print(output_str)

    def loop(
        self, currentTime: float, rx_array: list, sense_matrix: np.ndarray
    ):
        """Main control loop handling flight/car control modes."""
        self.set_defaults()

        self.elapsedTime = currentTime - self.lastTime
        self.lastTime = currentTime

        motor = self.STICK_MID
        servo = self.STICK_MID

        self.throttle = rx_array[0]
        self.aileron = rx_array[1]
        self.autopilot = rx_array[4]
        icontrol = 0

        # Check control mode determination
        if self.CONTROLLER_FLAG == -1:
            if self.autopilot > self.STICK_MID:
                icontrol = 1
            else:
                icontrol = 0
        else:
            icontrol = self.CONTROLLER_FLAG

        self.velocity_command = -99.0
        self.heading_command = -99.0

        # Implement fall-through cascade logic using explicit conditionals
        if icontrol >= 3:
            self.waypoint_loop(sense_matrix)

        if icontrol >= 2:
            if self.heading_command == -99.0:
                self.heading_command = 45.0
            self.heading_loop(sense_matrix)

        if icontrol >= 1:
            self.velocity_loop(sense_matrix)

        if icontrol >= 0:
            if self.velocity_command == -99.0:
                self.throttle = self.STICK_MID + 0.75 * (
                    self.STICK_MAX - self.STICK_MID
                )
            motor = self.throttle
            servo = self.aileron

        # Saturation Bounds
        motor = min(max(motor, self.STICK_MIN), self.STICK_MAX)
        servo = min(max(servo, self.STICK_MIN), self.STICK_MAX)

        # Output to Control Matrix
        self.control_matrix[0, 0] = motor
        self.control_matrix[1, 0] = servo

    def waypoint_loop(self, sense_matrix: np.ndarray):
        """Waypoint navigation calculations."""
        # MATLAB 1-based (1,1) and (2,1) -> 0-based [0,0] and [1,0]
        X = sense_matrix[0, 0]
        Y = sense_matrix[1, 0]

        DY = self.WAYPOINTS_Y[self.WAYINDEX] - Y
        DX = self.WAYPOINTS_X[self.WAYINDEX] - X

        self.heading_command = math.atan2(DY, DX) * 180.0 / math.pi
        distance = math.sqrt(DY * DY + DX * DX)

        if self.PRINTER == 4 * 100000:
            print(
                f"WAY (X,Y) = ({self.WAYPOINTS_X[self.WAYINDEX]},{self.WAYPOINTS_Y[self.WAYINDEX]}) "
                f"GPS (X,Y) = {X} {Y} HCOMM = {self.heading_command} DIST = {distance}"
            )
            self.PRINTER = 0

        self.PRINTER += 1

        if distance < 50:
            print(
                f"WAY (X,Y) = ({self.WAYPOINTS_X[self.WAYINDEX]},{self.WAYPOINTS_Y[self.WAYINDEX]}) "
                f"GPS (X,Y) = {X} {Y} HCOMM = {self.heading_command} DIST = {distance}"
            )
            self.WAYINDEX += 1
            if self.WAYINDEX > self.NUMWAYPOINTS - 1:
                self.WAYINDEX = 0

    def heading_loop(self, sense_matrix: np.ndarray):
        """Heading control proportional feedback loop."""
        kp = 2.5
        # MATLAB 1-based (6,1) -> 0-based [5,0]
        heading = sense_matrix[5, 0]

        dheading = (
            -delpsi(
                heading * math.pi / 180.0, self.heading_command * math.pi / 180.0
            )
            * 180.0
            / math.pi
        )

        if dheading > 180:
            dheading -= 180
            dheading *= -1
        if dheading < -180:
            dheading += 180
            dheading *= -1

        daileron = kp * dheading
        dpwm = self.STICK_MAX - self.STICK_MID

        # Constrain aileron deflection
        daileron = min(max(daileron, -dpwm), dpwm)
        self.aileron = self.STICK_MID + daileron

    def velocity_loop(self, sense_matrix: np.ndarray):
        """PI speed controller."""
        # MATLAB 1-based (7,1) -> 0-based [6,0]
        u = sense_matrix[6, 0]
        self.velocity_command = 5.0
        velocityerror = self.velocity_command - u

        kp = 60.0
        ki = 20.0

        self.throttle = self.OUTMID + kp * velocityerror + ki * self.velocityint
        self.throttle = min(max(self.throttle, self.OUTMIN), self.OUTMAX)

        # Anti-windup integration
        if self.OUTMIN < self.throttle < self.OUTMAX:
            self.velocityint += self.elapsedTime * velocityerror
"""