import math
import numpy as np

##Literal port of car_forces.cpp::ForceMoment. Module named "forces" (not
##"car_forces") so modeling.py can import it the same generic way fast.py
##already imports a vehicle's "controller" module -- this is the seam a
##future libraries/V_boat/forces.py plugs into for issue #69.

STICK_MID = 1500.0
STICK_MAX = 2016.0
STICK_MIN = 992.0


class FORCES():
    def __init__(self):
        self.FB = np.zeros(3)
        self.MB = np.zeros(3)

    def ForceMoment(self, time, state13, statedot13, pwm_out):
        """pwm_out: [motor, steering] in microseconds. Returns (FB, MB), both
        3-element numpy arrays: body-frame force (N) / moment (N-m)."""
        motor = pwm_out[0]
        steering = pwm_out[1]

        u = state13[7]
        v = state13[8]
        r = state13[12]

        force_max = 50.0
        s = 0.007681
        dpwm = motor - STICK_MID
        dsteer = steering - STICK_MID
        steer_angle = 45 * math.pi / 180.0 * dsteer / (STICK_MAX - STICK_MID)
        force = math.copysign(1.0, dpwm) * force_max * (1 - math.exp(-s * abs(dpwm)))

        xforce = force - 7.65 * u
        yforce = -10.0 * v + 0.0 * steer_angle
        Nmoment = 75 * (steer_angle - 0.4 * r)

        self.FB = np.array([xforce, yforce, 0.0])
        self.MB = np.array([0.0, 0.0, Nmoment])
        return self.FB, self.MB


if __name__ == '__main__':
    f = FORCES()
    state = np.zeros(13)

    ##Motor at mid-stick (no throttle command) -> force should be ~0
    FB, MB = f.ForceMoment(0.0, state, np.zeros(13), [STICK_MID, STICK_MID])
    assert abs(FB[0]) < 1e-9, FB

    ##Full forward throttle -> positive xforce approaching force_max as u stays 0
    FB, MB = f.ForceMoment(0.0, state, np.zeros(13), [STICK_MAX, STICK_MID])
    assert FB[0] > 0, FB
    assert FB[0] < 50.0, FB

    ##Full reverse throttle -> negative xforce
    FB, MB = f.ForceMoment(0.0, state, np.zeros(13), [STICK_MIN, STICK_MID])
    assert FB[0] < 0, FB
    print('forces.py (car) OK')
