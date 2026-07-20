import sys
import random

sys.path.append('../GPS')
sys.path.append('../libraries/GPS')
sys.path.append('../modeling')
sys.path.append('../libraries/modeling')
import gps as gps_module

from modeling import convert_z_to_pressure

##Self-contained port of sensors.cpp's send()/poll()/populate()/pollute(),
##taking a truth row (modeling.MODELING.get_truth_row()) in and producing a
##sensed row out. Deliberately does NOT run libraries/AHRS/AHRS.py's Mahony
##filter -- sensors.cpp force-seeds its AHRS state from the truth quaternion
##(+ noise) every call anyway (orientation.ahrs.q0..q3 = polluted truth),
##so the net effect for SIL/SIMONLY is that sensed roll/pitch/yaw are the
##truth Euler angles plus the configured angle bias/noise. Pollution is
##applied directly to the already-computed truth Euler angles (equivalent
##to sensors.cpp's quat->euler->pollute->quat->[reseed AHRS]->euler round
##trip, without needing to reimplement the AHRS class). Position/pressure/
##gyro pollution matches sensors.cpp exactly (pollute(bias,noise) =
##bias + uniform(-noise,noise), bias drawn once per run).
##
##Two independently-noised altitude channels are kept, matching the C++
##side: GPS altitude (bias_pos/noise_pos, feeds into satellites.Z) and
##barometer altitude (bias_pressure/noise_pressure, feeds "Sense Z(m)").

HEADERNAMES = [
    'Sense X(m)', 'Sense Y(m)', 'Sense Z(m)', 'Sense Roll (deg)', 'Sense Pitch (deg)', 'Sense Compass (deg)',
    'Sense U(m/s)', 'Sense V(m/s)', 'Sense W(m/s)', 'Sense P(rad/s)', 'Sense Q(rad/s)', 'Sense R(rad/s)',
    'Sense Mx(Gauss)', 'Sense My(Gauss)', 'Sense Mz(Gauss)',
    'Sense GPS Latitude (deg)', 'Sense GPS Longitude (deg)', 'Sense GPS Altitude (m)',
    'Sense GPS Heading (deg)', 'Sense IMU Heading (deg)',
    'Sense Analog 1 (V)', 'Sense Analog 2 (V)', 'Sense Analog 3 (V)', 'Sense Analog 4 (V)', 'Sense Analog 5 (V)', 'Sense Analog 6 (V)',
    'Sense Pressure (Pa)', 'Sense Pressure Altitude (m)', 'Sense Temperature (C)',
]


def _randnum(start, end):
    """Port of mathp.cpp's randnum(start,end)."""
    return random.uniform(start, end)


class SENSORS():
    def __init__(self):
        self.sense_gps = gps_module.GPS()
        self.next_gps_time = 0.0
        self.last_gps_poll_time = None

    def _setbias(self, bias, std):
        return [bias + _randnum(-std, std) for _ in range(3)]

    def init(self, config_data, simulation_data):
        self.GPS_RATE = config_data[4]  # row 5

        self.IERROR = int(simulation_data[20])  # row 21
        bias_pos, std_pos, self.noise_pos = simulation_data[21:24]       # rows 22-24
        bias_angle, std_angle, self.noise_angle = simulation_data[24:27]  # rows 25-27
        bias_pressure, std_pressure, self.noise_pressure = simulation_data[27:30]  # rows 28-30
        bias_gyro, std_gyro, self.noise_gyro = simulation_data[30:33]     # rows 31-33

        self.bias_pos = self._setbias(bias_pos, std_pos)
        self.bias_angle = self._setbias(bias_angle, std_angle)
        self.bias_gyro = self._setbias(bias_gyro, std_gyro)
        self.bias_pressure_val = bias_pressure + _randnum(-std_pressure, std_pressure)

        X_origin = config_data[18] if len(config_data) > 18 else 30.0
        Y_origin = config_data[19] if len(config_data) > 19 else -88.0
        self.sense_gps.setOrigin(X_origin, Y_origin)

    def pollute(self, bias, noise):
        return bias + _randnum(-noise, noise)

    def send_and_poll(self, currentTime, truth_row):
        """truth_row: modeling.MODELING.get_truth_row() output (29 values).
        Returns a sensed row in the same 29-value layout (HEADERNAMES)."""
        X, Y, Z = truth_row[0], truth_row[1], truth_row[2]
        roll, pitch, yaw = truth_row[3], truth_row[4], truth_row[5]
        p, q, r = truth_row[9], truth_row[10], truth_row[11]

        if self.IERROR:
            Xs = X + self.pollute(self.bias_pos[0], self.noise_pos)
            Ys = Y + self.pollute(self.bias_pos[1], self.noise_pos)
            Zgps = Z + self.pollute(self.bias_pos[2], self.noise_pos)
            Zbaro = Z + self.pollute(self.bias_pressure_val, self.noise_pressure)
            roll += self.pollute(self.bias_angle[0], self.noise_angle)
            pitch += self.pollute(self.bias_angle[1], self.noise_angle)
            yaw += self.pollute(self.bias_angle[2], self.noise_angle)
            p += self.pollute(self.bias_gyro[0], self.noise_gyro)
            q += self.pollute(self.bias_gyro[1], self.noise_gyro)
            r += self.pollute(self.bias_gyro[2], self.noise_gyro)
        else:
            Xs, Ys, Zgps, Zbaro = X, Y, Z, Z

        if yaw > 180:
            yaw -= 360
        if yaw < -180:
            yaw += 360

        if currentTime >= self.next_gps_time:
            self.sense_gps.elapsedTime = (currentTime - self.last_gps_poll_time
                                           if self.last_gps_poll_time is not None
                                           else self.sense_gps.GPSNEXT)
            lat, lon, alt = self.sense_gps.convertXYZ2LATLON(Xs, Ys, Zgps)
            self.sense_gps.latitude, self.sense_gps.longitude, self.sense_gps.altitude = lat, lon, alt
            self.sense_gps.compute_heading_velocity()
            self.last_gps_poll_time = currentTime
            self.next_gps_time = currentTime + self.GPS_RATE

        speed = self.sense_gps.speed if self.sense_gps.speed != -99 else 0.0
        gps_heading = self.sense_gps.heading if self.sense_gps.heading not in (-99, -999) else 0.0

        pressure = convert_z_to_pressure(Zbaro)

        return [
            Xs, Ys, -Zbaro, roll, pitch, yaw,
            speed, 0.0, 0.0,  # U(GPS speed), V(sideslip, not modeled), W(vertical speed, not modeled)
            p, q, r,
            0.0, 0.0, 0.0,  # Mx,My,Mz -- no magnetic model
            self.sense_gps.latitude, self.sense_gps.longitude, self.sense_gps.altitude,
            gps_heading, yaw,  # GPS heading, IMU heading (dup of polluted yaw)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Analog 1-6 -- not modeled
            pressure, -Zbaro, 0.0,  # Pressure, Pressure Altitude, Temperature
        ]


if __name__ == '__main__':
    import numpy as np
    s = SENSORS()
    sim_data = [0.0] * 33
    sim_data[20] = 1     # IERROR on
    sim_data[21:24] = [2.0, 0.5, 1.0]   # bias/std/noise pos
    sim_data[24:27] = [0.5, 0.5, 0.5]   # bias/std/noise angle
    sim_data[27:30] = [2.0, 2.0, 0.5]   # bias/std/noise pressure
    sim_data[30:33] = [0.05, 0.1, 0.1]  # bias/std/noise gyro
    cfg_data = [0.0] * 20
    cfg_data[4] = 1.0  # GPS_RATE
    cfg_data[18], cfg_data[19] = 30.0, -88.0
    s.init(cfg_data, sim_data)

    truth = [0.0] * 29
    truth[15], truth[16], truth[17] = 30.0, -88.0, 0.0  # not used directly (recomputed from X,Y)
    sensed = s.send_and_poll(0.0, truth)
    assert len(sensed) == 29
    ##Position bias/noise should have moved X/Y away from exactly 0
    assert sensed[0] != 0.0 or sensed[1] != 0.0
    print('sensors.py OK:', sensed[:3])
