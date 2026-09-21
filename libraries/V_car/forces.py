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

    def ForceMoment(self,t,state,pwm_commands):
        Force = np.asarray([0,0,0])
        Moment = np.asarray([0,0,0])
        return Force,Moment

import math
import numpy as np


class Forces:

    def __init__(self, stick_mid=1500.0, stick_max=2000.0):
        # PWM Constants (default values based on C++ macros)
        self.STICK_MID = stick_mid
        self.STICK_MAX = stick_max

        # Initialize 3x1 body force and moment vectors
        self.FB = np.zeros((3, 1))
        self.MB = np.zeros((3, 1))

    def force_moment(
        self,
        time: float,
        state: np.ndarray,
        statedot: np.ndarray,
        pwm_out: np.ndarray,
        env=None,
    ):
        """Calculates body forces and moments for the car model."""
        # Zero out force and moment vectors
        self.FB.fill(0.0)
        self.MB.fill(0.0)

        # Friction Parameters
        d = 0.13335  # (m) - From wheel to center
        Vmax = 6.0  # (m/s) Max speed

        # Extract Actuator Values (Converting 1-based MATLAB indexing to 0-based Python indexing)
        motor = pwm_out[0, 0]
        steering = pwm_out[1, 0]

        # Extract States (Converting 1-based indexing: state.get(1,1) -> state[0,0], etc.)
        x = state[0, 0]
        u = state[7, 0]
        v = state[8, 0]
        w = state[9, 0]
        p = state[10, 0]
        q = state[11, 0]
        r = state[12, 0]

        # Calculate Forces
        force_max = 50.0
        s = 0.007681
        dpwm = motor - self.STICK_MID
        dsteer = steering - self.STICK_MID

        steer_angle = (
            (45.0 * math.pi / 180.0)
            * dsteer
            / (self.STICK_MAX - self.STICK_MID)
        )
        force = (
            math.copysign(1.0, dpwm)
            * force_max
            * (1.0 - math.exp(-s * abs(dpwm)))
        )

        xforce = force - 7.65 * u
        yforce = -10.0 * v + 0.0 * steer_angle

        # Calculate Moments
        Nmoment = 75.0 * (steer_angle - 0.4 * r)

        # Populate Forces
        self.FB[0, 0] += xforce
        self.FB[1, 0] += yforce
        self.FB[2, 0] += 0.0

        # Populate Moments
        self.MB[0, 0] += 0.0
        self.MB[1, 0] += 0.0
        self.MB[2, 0] += Nmoment