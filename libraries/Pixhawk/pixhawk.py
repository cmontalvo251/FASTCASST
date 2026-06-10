import sys
import time
import math
import struct

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
    """
    Compass-only MAVLink wrapper for Pixhawk connected via USB.
    GPS, IMU, and baro come from Navio2 hardware directly.
    Telemetry is injected to the SiK radio on TELEM1 via SERIAL_CONTROL.
    """

    def __init__(self, port='/dev/ttyACM0', baud=115200):
        ##Compass / attitude state (degrees)
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0   # compass heading

        ##Gyro rates (deg/s) — still available from ATTITUDE message
        self.gdegs = [0.0, 0.0, 0.0]

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
            print('Running in SIL mode — emulating Pixhawk compass')
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

            ##Request attitude at 20 Hz (compass heading)
            self._mav.mav.request_data_stream_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                20, 1
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
            self.yaw   = 0.0
            self.roll  = 0.0
            self.pitch = 0.0
            return

        if self._mav is None:
            return

        ##Drain all pending ATTITUDE messages
        while True:
            msg = self._mav.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() == 'ATTITUDE':
                self.roll  = math.degrees(msg.roll)
                self.pitch = math.degrees(msg.pitch)
                self.yaw   = (math.degrees(msg.yaw) + 360) % 360
                self.gdegs = [math.degrees(msg.rollspeed),
                              math.degrees(msg.pitchspeed),
                              math.degrees(msg.yawspeed)]

    # -------------------------------------------------------------------------
    # TELEMETRY — inject FASTCASST packet to SiK radio via Pixhawk TELEM1
    # -------------------------------------------------------------------------

    def _float_to_hex(self, num):
        """Convert float to 8-char IEEE 754 hex string."""
        bin_str = ''.join('{:0>8b}'.format(c) for c in struct.pack('!f', num))
        int_val = int(bin_str, 2)
        if int_val == 0:
            return '00000000'
        return hex(int_val).replace('0x', '')

    def send_to_radio(self, number_array, baud=57600):
        """
        Send a FASTCASST telemetry packet to the SiK radio on Pixhawk TELEM1
        using MAVLink SERIAL_CONTROL messages (raw byte injection).
        Packet format matches Comms.SerialSendArray: "index:hexvalue\r" per slot.
        """
        if self._mav is None:
            return

        ##Build the packet string
        packet = ''
        for i, n in enumerate(number_array):
            packet += f'{i}:{self._float_to_hex(n)}\r'

        ##Send in 70-byte chunks (SERIAL_CONTROL data field limit)
        data_bytes = packet.encode('ascii')
        chunk_size = 70
        for offset in range(0, len(data_bytes), chunk_size):
            chunk = data_bytes[offset:offset + chunk_size]
            padded = list(chunk) + [0] * (70 - len(chunk))
            try:
                self._mav.mav.serial_control_send(
                    device=0,       # SERIAL_CONTROL_DEV_TELEM1 = 0
                    flags=0,
                    timeout=0,
                    baudrate=baud,
                    count=len(chunk),
                    data=padded
                )
            except Exception as e:
                print(f'[PIXHAWK] send_to_radio error: {e}')
