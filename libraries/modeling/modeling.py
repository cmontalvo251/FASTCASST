import sys
import math
import numpy as np

sys.path.append('../RK4')
sys.path.append('../libraries/RK4')
sys.path.append('../Rotation')
sys.path.append('../libraries/Rotation')
sys.path.append('../Environment')
sys.path.append('../libraries/Environment')
sys.path.append('../GPS')
sys.path.append('../libraries/GPS')

from RK4 import RK4
import Rotation3 as rot
from environment import Environment
import gps as gps_module

##Literal port of modeling.cpp's init()/loop()/rk4step()/Derivatives(),
##minus the OpenGL/HIL-specific branches (not applicable in Python) and
##minus its own file logging (fast_simonly.py owns the Datalogger calls,
##since Python's Datalogger works differently from the C++ one -- see the
##plan file). Vehicle-agnostic except for the `forces` module it imports,
##following the same "sys.path.append('../libraries/V_'+VEHICLE)" pattern
##fast.py already uses for `controller` -- this is the seam issue #69 plugs
##a boat forces.py into.

##Truth-row column layout (29 values, matches modeling.cpp's headernames[]
##exactly -- quaternions replaced by Euler-in-degrees, Yaw duplicated at
##the IMU-heading slot). Reused by sensors.py for the sensed-row headers.
HEADERNAMES = [
    'X(m)', 'Y(m)', 'Z(m)', 'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)',
    'U(m/s)', 'V(m/s)', 'W(m/s)', 'P(rad/s)', 'Q(rad/s)', 'R(rad/s)',
    'Mx(Gauss)', 'My(Gauss)', 'Mz(Gauss)',
    'GPS Latitude (deg)', 'GPS Longitude (deg)', 'GPS Altitude (m)',
    'GPS Heading (deg)', 'Yaw (deg)',
    'Analog 1 (V)', 'Analog 2 (V)', 'Analog 3 (V)', 'Analog 4 (V)', 'Analog 5 (V)', 'Analog 6 (V)',
    'Pressure (Pa)', 'Pressure Altitude (m)', 'Temperature (C)',
]

OUTMIN, OUTMID, OUTMAX = 992.0, 1500.0, 1950.0


def convert_z_to_pressure(z):
    """Port of mathp.cpp's ConvertZ2Pressure."""
    altitude = -z
    pascals = 101325.0 * (1.0 - 2.25577e-5 * altitude) ** 5.25588
    return pascals * 0.01


class MODELING():
    def __init__(self, vehicle):
        self.vehicle = vehicle
        sys.path.append('../libraries/V_' + vehicle)
        sys.path.append('../V_' + vehicle)
        import forces as forces_module
        self.extforces = forces_module.FORCES()
        self.env = Environment()
        self.integrator = RK4()
        self.truth_gps = gps_module.GPS()
        self.ok = 1

    def init(self, simulation_data, config_data):
        ##simulation_data/config_data: flat lists from input_files.read_input_file()
        self.TFINAL = simulation_data[0]        # row 1
        self.TIMESTEP = simulation_data[1]      # row 2
        self.integrationTime = 0.0

        ##13 initial conditions: X,Y,Z,q0,q1,q2,q3,u,v,w,p,q,r (rows 3-15)
        ic13 = simulation_data[2:15]

        self.NUMACTUATORS = int(simulation_data[38])  # row 39
        ##Settling times (rows 40..40+N-1) and actuator ICs (rows 40+N..40+2N-1)
        settle_start = 39
        ic_start = 39 + self.NUMACTUATORS
        self.settling_time = simulation_data[settle_start:settle_start + self.NUMACTUATORS]
        actuator_ic = simulation_data[ic_start:ic_start + self.NUMACTUATORS]

        self.NUMINTEGRATIONSTATES = 13 + self.NUMACTUATORS
        ic_full = list(ic13) + list(actuator_ic)

        self.IACTUATORERROR = int(simulation_data[36])   # row 37
        self.ACTUATORPERCENTERROR = simulation_data[37]  # row 38
        self.FORCES_FLAG = int(simulation_data[15])       # row 16

        self.actuatorStates = np.array(actuator_ic, dtype=float)
        self.pwm_out = self.actuatorStates.copy()

        ##Mass / inertia
        self.mass = config_data[11]  # row 12
        Ixx, Iyy, Izz = config_data[12], config_data[13], config_data[14]  # rows 13-15
        Ixy, Ixz, Iyz = (config_data[15], config_data[16], config_data[17]) \
            if len(config_data) > 17 else (0.0, 0.0, 0.0)  # rows 16-18
        self.I = np.array([
            [Ixx, Ixy, Ixz],
            [Ixy, Iyy, Iyz],
            [Ixz, Iyz, Izz],
        ])
        self.Iinv = np.linalg.inv(self.I)

        self.env.setMass(self.mass)
        self.env.init(simulation_data)

        ##GPS origin override (rows 19-20), matching ConvertXYZ2LLH's convention
        self.X_origin = config_data[18] if len(config_data) > 18 else 30.0
        self.Y_origin = config_data[19] if len(config_data) > 19 else -88.0
        self.truth_gps.setOrigin(self.X_origin, self.Y_origin)
        self.xprev, self.yprev = 0.0, 0.0

        self.integrator.init(self.NUMINTEGRATIONSTATES, self.TIMESTEP)
        self.integrator.set_ICs(ic_full)

        print('Modeling Routine initialized')

    def _set_gps(self, state13):
        X, Y, Z = state13[0], state13[1], state13[2]
        lat, lon, alt = self.truth_gps.convertXYZ2LATLON(X, Y, Z)
        heading = math.degrees(math.atan2(Y - self.yprev, X - self.xprev)) if (X, Y) != (self.xprev, self.yprev) else 0.0
        self.xprev, self.yprev = X, Y
        return lat, lon, alt, heading

    def step(self, currentTime, control_pwm_us):
        """Advance the truth state by one TIMESTEP (4-stage RK4). control_pwm_us
        is a length-NUMACTUATORS list/array of actuator commands in microseconds."""
        if currentTime > self.TFINAL:
            self.ok = 0
            return
        for i in (1, 2, 3, 4):
            self._derivatives(currentTime, control_pwm_us)
            self.integrator.integrate(i)
        self.integrationTime += self.TIMESTEP

    def get_truth_row(self):
        """Current truth state as a 29-element row matching HEADERNAMES."""
        state = self.integrator.State
        q0123 = state[3:7]
        ptp = rot.quat2euler(q0123)
        lat, lon, alt, heading = self._set_gps(state)
        pressure = convert_z_to_pressure(state[2])
        pressure_alt = -state[2]
        yaw_deg = math.degrees(ptp[2])
        return [
            state[0], state[1], state[2],
            math.degrees(ptp[0]), math.degrees(ptp[1]), yaw_deg,
            state[7], state[8], state[9],
            state[10], state[11], state[12],
            0.0, 0.0, 0.0,  # Mx,My,Mz -- no magnetic model
            lat, lon, alt, heading, yaw_deg,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Analog 1-6 -- not modeled
            pressure, pressure_alt, 0.0,  # Temperature -- not modeled
        ]

    def _derivatives(self, currentTime, control_pwm_us):
        ##Actuator dynamics -- passthrough unless IACTUATORERROR>1 with a
        ##nonzero settling time (car's Simulation.txt uses IACTUATORERROR=1,
        ##i.e. passthrough; the lag branch is implemented for parity).
        for i in range(self.NUMACTUATORS):
            settle = self.settling_time[i]
            if settle == 0 or self.IACTUATORERROR <= 1:
                self.actuatorStates[i] = control_pwm_us[i]
            else:
                time_constant = 4.0 / settle
                actuatorDot = time_constant * (control_pwm_us[i] - self.actuatorStates[i])
                self.integrator.k[13 + i] = actuatorDot
            val = int(self.actuatorStates[i]) if self.IACTUATORERROR >= 1 else self.actuatorStates[i]
            self.pwm_out[i] = min(max(val, OUTMIN), OUTMAX)

        ##Kinematics
        state = self.integrator.StateDel
        cg = state[0:3]
        q0123 = state[3:7]
        cgdotB = state[7:10]
        pqr = state[10:13]

        cgdotI = rot.rotate_body_to_inertial(cgdotB, q0123)
        self.integrator.k[0:3] = cgdotI

        dq = rot.quat_kinematic_derivative(q0123, pqr)
        self.integrator.k[3:7] = dq

        ##Environment: gravity + ground contact (magnetic field not modeled)
        self.env.gravitymodel(state)
        self.env.groundcontactmodel(state, self.integrator.k)

        ##External forces (vehicle-specific)
        if self.FORCES_FLAG:
            FB, MB = self.extforces.ForceMoment(currentTime, state, self.integrator.k, self.pwm_out)
        else:
            FB, MB = np.zeros(3), np.zeros(3)

        FTOTALI = self.env.FGRAVI.copy()
        FGNDB = rot.rotate_inertial_to_body(self.env.FGNDI, q0123)
        FTOTALB = rot.rotate_inertial_to_body(FTOTALI, q0123)
        FTOTALB = FTOTALB + FGNDB
        ##Car always adds external forces regardless of ground contact
        ##(matches modeling.cpp's "#if defined car || tank" branch).
        FTOTALB = FTOTALB + FB

        ##Translational dynamics: uvwdot = FTOTALB/mass - pqr x cgdotB
        Kuvw_pqr = np.cross(pqr, cgdotB)
        uvwdot = FTOTALB / self.mass - Kuvw_pqr
        self.integrator.k[7:10] = uvwdot

        MGNDB = rot.rotate_inertial_to_body(self.env.MGNDI, q0123)
        MTOTALB = MGNDB + MB

        ##Rotational dynamics: pqrdot = Iinv * (MTOTALB - pqr x (I*pqr))
        I_pqr = self.I @ pqr
        pqrskew_I_pqr = np.cross(pqr, I_pqr)
        pqrdot = self.Iinv @ (MTOTALB - pqrskew_I_pqr)
        self.integrator.k[10:13] = pqrdot


if __name__ == '__main__':
    ##Constant-throttle smoke test against the car: forward speed should
    ##converge toward a plausible steady state under xforce = force - 7.65*u.
    sim_data = [0.0] * 45
    sim_data[0] = 5.0     # TFINAL
    sim_data[1] = 0.001   # TIMESTEP
    sim_data[5] = 1.0     # q0 = 1 (level attitude)
    sim_data[15] = 1      # FORCES_FLAG
    sim_data[16] = 3      # Gravity_Flag = flat earth
    sim_data[36] = 1      # IACTUATORERROR (passthrough)
    sim_data[38] = 2      # NUMACTUATORS
    sim_data[41] = 1500.0  # actuator IC 1 (motor)
    sim_data[42] = 1500.0  # actuator IC 2 (steering)
    cfg_data = [0.0] * 20
    cfg_data[11] = 4.66   # mass
    cfg_data[12], cfg_data[13], cfg_data[14] = 0.056, 0.035, 0.077  # Ixx,Iyy,Izz
    cfg_data[18], cfg_data[19] = 30.0, -88.0

    m = MODELING('car')
    m.init(sim_data, cfg_data)
    t = 0.0
    while t < 5.0:
        m.step(t, [2016.0, 1500.0])  # full forward throttle, straight steering
        t += m.TIMESTEP
    row = m.get_truth_row()
    u_final = row[6]
    assert u_final > 3.0, row  # should have accelerated forward substantially
    assert abs(row[2]) < 0.5, row  # Z should stay near ground (ground contact holding it)
    print('modeling.py OK: U(5s) =', u_final, 'Z =', row[2])
