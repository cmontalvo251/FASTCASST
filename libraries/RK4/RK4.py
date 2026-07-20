import numpy as np


class RK4():
    """Literal port of RK4.cpp: a vehicle-agnostic 4-stage RK4 integrator.

    The caller is responsible for computing the derivative vector k
    (matching integrator.k in the C++ side) before each integrate(i) call.
    """

    def __init__(self):
        pass

    def init(self, numvars, dt):
        self.NUMVARS = numvars
        self.TIMESTEP = dt
        self.State = np.zeros(numvars)
        self.StateDel = np.zeros(numvars)
        self.k = np.zeros(numvars)
        self.k1 = np.zeros(numvars)
        self.k2 = np.zeros(numvars)
        self.k3 = np.zeros(numvars)
        self.k4 = np.zeros(numvars)
        self.phi = np.zeros(numvars)

    def set_ICs(self, x0):
        self.State = np.array(x0, dtype=float).copy()
        self.StateDel = np.array(x0, dtype=float).copy()

    def integrate(self, i):
        dt = self.TIMESTEP
        if i == 1:
            self.k1 = self.k.copy()
            self.StateDel = self.State + self.k1 * (dt / 2.0)
        elif i == 2:
            self.k2 = self.k.copy()
            self.StateDel = self.State + self.k2 * (dt / 2.0)
        elif i == 3:
            self.k3 = self.k.copy()
            self.StateDel = self.State + self.k3 * dt
        elif i == 4:
            self.k4 = self.k.copy()
            self.phi = self.k1 / 6.0 + self.k2 / 3.0 + self.k3 / 3.0 + self.k4 / 6.0
            self.State = self.State + self.phi * dt


if __name__ == '__main__':
    ##Integrate dx/dt = 1 (constant derivative) for 1 second, expect x(1.0) ~= 1.0
    rk = RK4()
    rk.init(1, 0.01)
    rk.set_ICs([0.0])
    t = 0.0
    while t < 1.0 - 1e-9:
        for i in (1, 2, 3, 4):
            rk.k[0] = 1.0
            rk.integrate(i)
        t += 0.01
    assert abs(rk.State[0] - 1.0) < 1e-6, rk.State[0]
    print('RK4.py OK: x(1.0) =', rk.State[0])
