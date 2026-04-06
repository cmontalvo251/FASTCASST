import sys
import time
import math

sys.path.append('../Util')
sys.path.append('../libraries/Util')
import util

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print('WARNING: pymavlink not installed. Run: sudo pip3 install pymavlink')


class PIXHAWK():
    def __init__(self, port='/dev/ttyACM0', baud=115200):
        ##GPS state
        self.latitude     = -99
        self.longitude    = -99
        self.altitude     = -99
        self.speed        = -99
        self.fix_quality  = 0
        self.num_satellites = 0
        self.has_fix      = False

        ##Attitude state (degrees)
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0

        ##Barometer state
        self.ALT  = 0.0
        self.PRES = 0.0

        ##Fake imu-style outputs so fast.py can use the same interface
        ##  a    = [ax, ay, az]  (m/s^2)  — not available via MAVLink basic stream
        ##  gdegs= [gx, gy, gz] (deg/s)
        ##  m    = [mx, my, mz] (uT)
        ##  rpy  = [roll, pitch, yaw] raw (deg)
        ##  rpy_ahrs = [roll, pitch, yaw] AHRS-filtered (deg)
        ##  temp = temperature (C)
        self.a        = [0.0, 0.0, 0.0]
        self.gdegs    = [0.0, 0.0, 0.0]
        self.m        = [0.0, 0.0, 0.0]
        self.rpy      = [0.0, 0.0, 0.0]
        self.rpy_ahrs = [0.0, 0.0, 0.0]
        self.temp     = 0.0

        ##Polling timing
        self.POLL_RATE = 0.05   # 20 Hz
        self.last_poll = -self.POLL_RATE

        self.port  = port
        self.baud  = baud
        self._mav  = None

        self.SIL = util.isSIL()
        self.initialize()

    def initialize(self):
        if self.SIL:
            print('Running in SIL mode — emulating Pixhawk')
            return

        if not MAVLINK_AVAILABLE:
            print('ERROR: pymavlink not available. Run: sudo pip3 install pymavlink')
            return

        print(f'Connecting to Pixhawk on {self.port} at {self.baud} baud...')
        try:
            self._mav = mavutil.mavlink_connection(self.port, baud=self.baud)
            print('Waiting for heartbeat...')
            self._mav.wait_heartbeat(timeout=10)
            print(f'Pixhawk connected! System: {self._mav.target_system}')

            ##Request attitude at 20 Hz
            self._mav.mav.request_data_stream_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                20, 1
            )
            ##Request GPS at 5 Hz
            self._mav.mav.request_data_stream_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                5, 1
            )
            ##Request VFR_HUD (speed, baro alt) at 10 Hz
            self._mav.mav.request_data_stream_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                10, 1
            )
        except Exception as e:
            print(f'ERROR: Could not connect to Pixhawk: {e}')
            self._mav = None

    # -------------------------------------------------------------------------
    # POLLING
    # -------------------------------------------------------------------------

    def poll(self, RunTime):
        if (RunTime - self.last_poll) > self.POLL_RATE:
            self.last_poll = RunTime
            self.update()

    def update(self):
        if self.SIL:
            self.latitude     = 30.69
            self.longitude    = -88.10
            self.altitude     = 0.0
            self.speed        = 0.0
            self.has_fix      = True
            self.fix_quality  = 1
            self.num_satellites = 6
            self.rpy_ahrs     = [0.0, 0.0, 0.0]
            self.rpy          = [0.0, 0.0, 0.0]
            self.ALT          = 0.0
            return

        if self._mav is None:
            return

        ##Drain all pending messages in one pass
        while True:
            msg = self._mav.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()

            if mtype == 'ATTITUDE':
                self.roll      = math.degrees(msg.roll)
                self.pitch     = math.degrees(msg.pitch)
                self.yaw       = math.degrees(msg.yaw)
                self.rpy       = [self.roll, self.pitch, self.yaw]
                self.rpy_ahrs  = [self.roll, self.pitch, self.yaw]
                self.gdegs     = [math.degrees(msg.rollspeed),
                                  math.degrees(msg.pitchspeed),
                                  math.degrees(msg.yawspeed)]

            elif mtype == 'GPS_RAW_INT':
                self.fix_quality    = msg.fix_type
                self.num_satellites = msg.satellites_visible
                self.has_fix        = msg.fix_type >= 3
                if self.has_fix:
                    self.latitude  = msg.lat / 1e7
                    self.longitude = msg.lon / 1e7
                    self.altitude  = msg.alt / 1000.0   # mm -> m

            elif mtype == 'VFR_HUD':
                self.speed = msg.groundspeed   # m/s
                self.ALT   = msg.alt           # barometric altitude (m)

            elif mtype == 'SCALED_PRESSURE':
                self.PRES = msg.press_abs      # hPa

    # -------------------------------------------------------------------------
    # IMU-STYLE INTERFACE (drop-in for mpu9250.getALL)
    # -------------------------------------------------------------------------

    def getALL(self, elapsedTime):
        """
        Match the mpu9250.MPU9250.getALL() return signature so fast.py
        can call pixhawk.getALL(dt) without changes to the main loop.

        Returns: a, gdegs, m, rpy, rpy_ahrs, temp
        """
        self.update()
        return (self.a, self.gdegs, self.m,
                self.rpy, self.rpy_ahrs, self.temp)

    # -------------------------------------------------------------------------
    # BAROMETER-STYLE INTERFACE (drop-in for ms5611)
    # -------------------------------------------------------------------------

    def calibrate(self):
        """No-op — Pixhawk handles baro calibration internally."""
        print('[PIXHAWK] Barometer managed by Pixhawk — no calibration needed.')

    def baro_poll(self, RunTime):
        """Alias so fast.py can call pixhawk.baro_poll() instead of baro.poll()."""
        self.poll(RunTime)
