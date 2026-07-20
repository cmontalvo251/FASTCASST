#############################################
#
#  FASTCASSTPy SIMONLY - fixed-timestep 6DOF physics simulation driver
#
#  Issue #68: fast.py only supported AUTO (real-hardware) mode. This is a
#  separate driver (not a mode flag inside fast.py) that mirrors fast.cpp's
#  SIMONLY loop structure instead: fixed-step clock, a truth physics engine
#  (libraries/modeling/modeling.py) feeding a noisy sense layer
#  (libraries/sensors/sensors.py), running the *unmodified* vehicle
#  controller against the sensed state, and logging both a truth and a
#  sensed CSV trail (logs/N.csv, data/N.csv) for plot_simonly.py.
#
#  Vinicius da Luz, Summer 2026
#
################################################

import sys
import types
import numpy as np

VEHICLE = 'car'

sys.path.append('../libraries/Util')
sys.path.append('../libraries/RK4')
sys.path.append('../libraries/Rotation')
sys.path.append('../libraries/Environment')
sys.path.append('../libraries/modeling')
sys.path.append('../libraries/sensors')
sys.path.append('../libraries/GPS')
sys.path.append('../libraries/Datalogger')
sys.path.append('../libraries/V_' + VEHICLE)

import input_files
import modeling
import sensors
import controller
import datalogger

STICK_MIN, STICK_MID, STICK_MAX = 992.0, 1500.0, 2016.0


def controls_to_pwm_us(controls):
    """Car controller returns [throttle, steering] in [-1,1]; convert to
    actuator PWM microseconds, matching car_forces.py's STICK_* convention."""
    pwm = []
    for c in controls:
        c = max(-1.0, min(1.0, c))
        if c >= 0:
            pwm.append(STICK_MID + c * (STICK_MAX - STICK_MID))
        else:
            pwm.append(STICK_MID + c * (STICK_MID - STICK_MIN))
    return pwm


def main():
    print('FASTCASST SIMONLY (Python) -- vehicle =', VEHICLE)

    simulation_data = input_files.read_input_file('../libraries/V_' + VEHICLE + '/Input_Files/Simulation.txt')
    config_data = input_files.read_input_file('../libraries/V_' + VEHICLE + '/Input_Files/Config.txt')

    model = modeling.MODELING(VEHICLE)
    model.init(simulation_data, config_data)

    sense = sensors.SENSORS()
    sense.init(config_data, simulation_data)

    vehicle = controller.CONTROLLER()

    ##Real telemetry-triggered mission wiring is issue #69's job -- this is
    ##verification-only scaffolding, matching the box route used to validate
    ##the C++ SIL/SIMONLY car controller earlier in this project.
    start_lat, start_lon, _ = model.truth_gps.convertXYZ2LATLON(0.0, 0.0, 0.0)
    box_waypoints = []
    for X, Y in [(500.0, 0.0), (500.0, 500.0), (0.0, 500.0)]:
        lat, lon, _ = model.truth_gps.convertXYZ2LATLON(X, Y, 0.0)
        box_waypoints.append((lat, lon))
    vehicle.set_mission(box_waypoints, start_lat, start_lon)

    ##Minimal RC stub -- bypasses RCInput.py's SIL ms/us unit bug (flagged
    ##as a follow-up, not fixed here). autopilot>1500 forces the car
    ##controller's GPS-autopilot branch regardless of any real receiver.
    rc_stub = types.SimpleNamespace(autopilot=1600.0, throttlerc=0.0, rollrc=0.0, signal_lost=False)

    LOGRATE = config_data[1]  # row 2
    TIMESTEP = simulation_data[1]  # row 2
    TFINAL = simulation_data[0]  # row 1
    NUMACTUATORS = int(simulation_data[38])

    rc_names = ['RC Channel #%d' % i for i in range(1, 6)]
    pwm_names = ['PWM Model %d' % i for i in range(1, NUMACTUATORS + 1)]
    header = ['Time (sec)'] + modeling.HEADERNAMES + rc_names + pwm_names
    sense_header = ['Time (sec)'] + sensors.HEADERNAMES + rc_names + pwm_names

    sense_logger = datalogger.Datalogger(len(header), directory='../data/', extension='.csv')
    sense_logger.writeheader(sense_header)
    truth_logger = datalogger.Datalogger(len(header), directory='../logs/', extension='.csv')
    truth_logger.writeheader(header)

    rc_placeholder = [992, 1500, 1500, 1500, 992]

    currentTime = 0.0
    nextLogTime = 0.0
    while currentTime <= TFINAL:
        truth_row = model.get_truth_row()
        sensed_row = sense.send_and_poll(currentTime, truth_row)

        gps_llh = types.SimpleNamespace(latitude=sensed_row[15], longitude=sensed_row[16])
        rpy_ahrs = [sensed_row[3], sensed_row[4], sensed_row[19]]
        gdegs = sensed_row[9:12]

        controls, defaults, color = vehicle.loop(currentTime, rc_stub, gps_llh, rpy_ahrs, gdegs, None)
        pwm_us = controls_to_pwm_us(controls)

        if currentTime >= nextLogTime:
            sense_logger.outdata = [currentTime] + sensed_row + rc_placeholder + list(model.pwm_out)
            sense_logger.println()
            truth_logger.outdata = [currentTime] + truth_row + rc_placeholder + list(model.pwm_out)
            truth_logger.println()
            nextLogTime = currentTime + LOGRATE

        model.step(currentTime, pwm_us)
        if not model.ok:
            break
        currentTime += TIMESTEP

    sense_logger.close()
    truth_logger.close()
    print('Main Loop End')


if __name__ == '__main__':
    main()
