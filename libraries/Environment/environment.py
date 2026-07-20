import numpy as np

##Minimal port of environment.cpp: only the flat-earth gravity branch
##(Gravity_Flag==3) and the ground-contact spring/damper model, which is
##all the car's Simulation.txt (Gravity_Flag=3, Magnetic_Flag=0) exercises.
##EGM2008/point-mass/Sun gravity and the EMM2015 magnetic model are
##intentionally not ported here (dead code for this vehicle) -- see the
##plan file for why. gravitymodel()/groundcontactmodel() keep the same
##call signature as the C++ side so a future Gravity_Flag branch can be
##added without changing callers.

GRAVITYSI = 9.81
GEARTH = 9.80665
GNDSTIFF = 112.0
GNDDAMP = 50.0
GNDCOEFF = 0.1


def sat(x, epsilon, scalefactor):
    """Port of mathp.cpp's sat(): saturated linear interpolation."""
    if x > epsilon:
        return scalefactor
    elif x < -epsilon:
        return -scalefactor
    else:
        return scalefactor * x / epsilon


class Environment():
    def __init__(self):
        self.mass = 0.0
        self.Gravity_Flag = 3
        self.FGRAVI = np.zeros(3)
        self.FGNDI = np.zeros(3)
        self.MGNDI = np.zeros(3)

    def setMass(self, m):
        self.mass = m

    def init(self, simulation_data):
        ##simulation_data: flat list from input_files.read_input_file(Simulation.txt)
        self.Gravity_Flag = int(simulation_data[16])  # row 17

    def gravitymodel(self, state13):
        ##Only Gravity_Flag == 3 (flat earth, Z-down) is implemented.
        if self.Gravity_Flag == 3:
            g = np.array([0.0, 0.0, GEARTH])
        elif self.Gravity_Flag == -1:
            g = np.zeros(3)
        else:
            raise NotImplementedError(
                'environment.py only implements Gravity_Flag==3 (flat earth); '
                'got %d' % self.Gravity_Flag)
        self.FGRAVI = g * self.mass

    def groundcontactmodel(self, state13, k):
        z = state13[2]
        xdot, ydot, zdot = k[0], k[1], k[2]
        N = self.mass * GRAVITYSI

        inside_earth = (self.Gravity_Flag == 3) and (z > 0)  # Z is down

        if inside_earth:
            self.FGNDI = np.array([
                -N * GNDCOEFF * sat(xdot, 0.1, 1.0),
                -N * GNDCOEFF * sat(ydot, 0.1, 1.0),
                -z * GNDSTIFF - zdot * GNDDAMP,
            ])
            self.MGNDI = np.zeros(3)
        else:
            self.FGNDI = np.zeros(3)
            self.MGNDI = np.zeros(3)


if __name__ == '__main__':
    env = Environment()
    env.setMass(4.66)
    env.init([0] * 16 + [3])  # Gravity_Flag=3 at row 17 (index 16)
    env.gravitymodel(np.zeros(13))
    assert np.allclose(env.FGRAVI, [0, 0, 4.66 * GEARTH])

    ##A vehicle 1m below the flat ground (z>0, NED convention) with zero velocity
    ##should get a strong upward (negative Z) restoring force.
    state = np.zeros(13)
    state[2] = 1.0
    env.groundcontactmodel(state, np.zeros(13))
    assert env.FGNDI[2] < 0, env.FGNDI
    assert abs(env.FGNDI[2] - (-1.0 * GNDSTIFF)) < 1e-9, env.FGNDI

    ##Above ground -> no contact force
    state[2] = -1.0
    env.groundcontactmodel(state, np.zeros(13))
    assert np.allclose(env.FGNDI, 0)
    print('environment.py OK')
