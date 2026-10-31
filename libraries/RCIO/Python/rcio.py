import os
import sys
import numpy as np
import json
import struct
import math

class RCIO():
    def __init__(self,NUMPWM,mode,json_file):
        self.SERVO_MIN = 0.995 #ms
        self.SERVO_MID = 1.504 #ms
        self.SERVO_MAX = 2.010 #ms
        B = 32768
        if json_file == 'XBOX_config.json':
            B = 32768
        elif json_file == 'RCTECH_config.json':
            B = 20000
        json_file_name = 'libraries/RCInput/Calibration/' + json_file
        self.rcin = RCInput(mode,1000*self.SERVO_MIN,1000*self.SERVO_MAX,json_file_name,num_channels=6,BIT_RANGE=B)
        self.rcout = []
        self.NUMPWM = NUMPWM
        self.pwm_commands = np.zeros(NUMPWM)
        for i in range(0,NUMPWM):
            print('Setting up Pin = ',i)
            self.rcout.append(PWM(mode,i))
            print('Pin ',i,' appended')
            self.rcout[i].initialize()
            self.rcout[i].set_period(50)
            self.rcout[i].enable()

    def set_commands(self,commands):
        for i in range(0,self.NUMPWM):
            ##Compute command from -1 to 1
            command = commands[i]*(self.SERVO_MAX-self.SERVO_MID)+self.SERVO_MID
            #Saturation block
            if(command < self.SERVO_MIN):
                command = self.SERVO_MIN
            if(command > self.SERVO_MAX):
                command = self.SERVO_MAX
            self.pwm_commands[i] = command
            self.rcout[i].set_duty_cycle(command)

class PWM():
    SYSFS_PWM_PATH_BASE = "/sys/class/pwm/pwmchip0/"
    SYSFS_PWM_EXPORT_PATH = "/sys/class/pwm/pwmchip0/export"
    SYSFS_PWM_UNEXPORT_PATH = "/sys/class/pwm/pwmchip0/unexport"

    def __init__(self,mode,channel):
        self.channel = channel
        self.channel_path = self.SYSFS_PWM_PATH_BASE + "pwm{}/".format(self.channel)
        self.is_initialized = False
        self.is_enabled = False
        self.MODE = mode

    def __enter__(self):
        self.initialize()
        return self

    def __exit__(self, *args):
        self.deinitialize()

    def deinitialize(self):
        if self.is_enabled:
            self.set_period(1)
            self.disable()
        with open(self.SYSFS_PWM_UNEXPORT_PATH, "w") as pwm_unexport:
            pwm_unexport.write(str(self.channel))

    def initialize(self):
        #These will work in HIL mode and AUTO
        if self.MODE != 'AUTO' and self.MODE != 'HIL':
            print('Emulating PWM signal. Initializing pin.....',self.channel)
        else:
            print('Initializing pin.....',self.channel)
            if not os.path.exists(self.SYSFS_PWM_PATH_BASE):
                print("Looking for rcio_pwm module in PATH = ",self.SYSFS_PWM_PATH_BASE,"....couldn't find it")
                raise OSError("rcio_pwm module wasn't loaded")
            if not os.path.exists(self.channel_path):
                with open(self.SYSFS_PWM_EXPORT_PATH, "w") as pwm_export:
                    pwm_export.write(str(self.channel))
        self.is_initialized = True

    def enable(self):
        if self.MODE != 'AUTO' and self.MODE !=' HIL':
            print('Emulating PWM signal. Enabling pin.....',self.channel)
        else:
            print('Enabling pin.....',self.channel)
            with open(self.channel_path + "enable", "w") as pwm_enable:
                pwm_enable.write("1")
        self.is_enabled = True

    def disable(self):
        with open(self.channel_path + "enable", "w") as pwm_enable:
            pwm_enable.write("0")
            self.is_enabled = False

    def set_period(self, freq):
        if not self.is_initialized:
            raise RuntimeError("PWM not initialized. Call initialize first")

        period_ns = int(1e9/freq)
        if self.MODE != 'AUTO' and self.MODE != 'HIL':
            print('Emulating PWM signal. Setting frequency for pin....',freq,self.channel)
        else:
            print('Setting frequency for pin....',freq,self.channel)
            with open(self.channel_path + "period",  "w") as pwm_period:
                pwm_period.write(str(period_ns))

    def set_duty_cycle(self, period):
        if not self.is_initialized:
            raise RuntimeError("PWM not initialized. Call initialize first")
        if self.MODE == 'AUTO' or self.MODE == 'HIL':
            period_ns = int(period*1e6)
            with open(self.channel_path + "duty_cycle", "w") as pwm_duty:
                pwm_duty.write(str(period_ns))

class RCInput():
    #Note for the FS-TH9X receiver you need to make sure you setup the channels
    #properly into the right order.  The order should be: TAER P1 3POS where
    #P1 is a potentiometer for arming because one of the switches broke
    #and then 3POS is the 3 position switch. Again the 2 auxiliary channels
    #can be whatever you want but this code operates under the assumption
    #that channel 5 is arming and channel 6 is autopilot mode
    #the first 4 channels are then the standard TAER

    def __init__(self,mode,STICK_MIN,STICK_MAX,json_file_name,num_channels=6,BIT_RANGE=32768):
        print('Initializing RCInput....')
        self.channels = []
        self.STICK_MAX = STICK_MAX
        self.STICK_MIN = STICK_MIN
        self.STICK_MID = (STICK_MAX + STICK_MIN)/2.0
        self.BIT_RANGE = BIT_RANGE
        self.num_channels = num_channels
        self.num_of_axis = num_channels
        self.rcsignals = [0]*num_channels
        self.MODE = mode
        self.ARMED = False #set armed to false
        self.color = 'Red' #for the LED

        if self.MODE == 'SIL':
            print("Setting up RCIO SIL System")
            #Create joystick object
            self.has_json_config = False
            self.axis_mappings = {}
            self.joy_fd = -1
            for i in range(5):
                self.axis_mappings[i] = {"axis": -1,"invert": False,"min": -1.0,"max": 1.0}                  
            self.config_file = json_file_name
            
            # Open joystick device safely across Linux and Windows
            joystick_dev = os.environ.get("JOY_DEV", "/dev/input/js0")
            try:
                print("Initializing joystick")
                flags = os.O_RDONLY
                if hasattr(os, 'O_NONBLOCK'):
                    flags |= os.O_NONBLOCK

                self.joy_fd = os.open(joystick_dev, flags)
                print("Getting information from the joystick")
                self.name_of_joystick = "USB Controller"
                self.joycomm = [self.STICK_MID]*max(num_channels,16)
                self.joyraw  = [self.STICK_MID]*max(num_channels,16)
                self.num_of_buttons = 32
                self.button = [0] * self.num_of_buttons
                print(f"joy_fd = {self.joy_fd}")
                print("Allocating Axes")
                print(f"Number of Axes = {self.num_of_axis}")
                print("Success!!!! Setting non-blocking mode")
            except (OSError, AttributeError):
                print("Couldn't open joysticks")
                print("Defaulting to Stick min")
                print(f"joy_fd = {self.joy_fd}")

            print(f"Using RX config file: {self.config_file}")
            f_data = None
            if self.config_file and os.path.exists(self.config_file):
                try:
                    with open(self.config_file, "r") as file_obj:
                        f_data = json.load(file_obj)
                except Exception as e:
                    print(f"Error parsing {self.config_file}: {e}")
            if f_data:
                try:
                    mapping = f_data.get("mapping", {})
                    controls = ["throttle","roll","pitch","yaw","arming_switch","autopilot_switch"]
                    for i, ctrl_name in enumerate(controls):
                        if ctrl_name in mapping:
                            ctrl = mapping[ctrl_name]
                            if "axis" in ctrl:
                                self.axis_mappings[i] = {
                                    "axis": ctrl["axis"],
                                    "invert": ctrl.get("invert", False),
                                    "min": ctrl.get("min", -1.0),
                                    "max": ctrl.get("max", 1.0),
                                }
                    self.has_json_config = True
                    print(f"Loaded JSON config from {self.config_file}")
                except Exception as e:
                    print(f"Error parsing JSON content: {e}")
                    sys.exit(1)
            else:
                print(f"Could not open JSON config file: {self.config_file}")
                sys.exit(1)
            print("Done")

        if self.MODE == 'AUTO' or self.MODE == 'HIL':
            print("Setting up RCIO AUTO or HIL System")
            for i in range(self.num_channels):
                #Read from PPM receiver
                try:
                    f = open("/sys/kernel/rcio/rcin/ch%d" % i, "r")
                    print('Opening RCinput channel = ',i)
                except: 
                    print ("Can't open file /sys/kernel/rcio/rcin/ch%d" % i)
                    #Fall back on emulation mode
                    f = i
            self.channels.append(f)
            print('RCInput HW initialized')

    def bit2PWM(self, val: int) -> int:
        outval = int((self.STICK_MAX - self.STICK_MID) * val / self.BIT_RANGE + self.STICK_MID)
        if outval > self.STICK_MAX:
            outval = self.STICK_MAX
        elif outval < self.STICK_MIN:
            outval = self.STICK_MIN
        return outval

    def invert(self, val: int) -> int:
        if val != 0:
            delta = val - self.STICK_MID
            inverse = -delta
            out = self.STICK_MID + inverse
        else:
            out = 0
        return out

    def mapjoy2rx(self):
        for i in range(self.num_of_axis):
            self.rcsignals[i] = self.joycomm[i]
        if self.has_json_config:
            for i in range(6):
                ax = self.axis_mappings[i]["axis"]
                if 0 <= ax < len(self.joycomm):
                    val = self.joycomm[ax]
                    if self.axis_mappings[i]["invert"]:
                        val = self.invert(val)
                    self.rcsignals[i] = val

    def setStickNeutral(self):
        for i in range(self.num_of_axis):
            if i == 0:
                self.rcsignals[i] = self.STICK_MIN
                #elif i == 3:
            #    self.rcsignals[i] = self.STICK_MAX
            else:
                self.rcsignals[i] = self.STICK_MID

    def readALL(self):
        if self.MODE == 'SIL':
            if self.joy_fd != -1:
                try:
                    while True:
                        try:
                            event_data = os.read(self.joy_fd, 8)
                            if len(event_data) == 8:
                                time_val, value, ev_type, number = struct.unpack("IhBB", event_data)
                                actual_type = ev_type & ~0x80
                                if actual_type == 2:  # JS_EVENT_AXIS
                                    if number < len(self.joycomm):
                                        #if number == 4:
                                        #    print(value)
                                        self.joycomm[number] = self.bit2PWM(value)
                                        self.joyraw[number] = value
                                elif actual_type == 1:  # JS_EVENT_BUTTON
                                    if number < len(self.button):
                                        self.button[number] = value
                        except BlockingIOError:
                            break
                    self.mapjoy2rx()
                    #print(self.joyraw)
                except Exception as e:
                    print(f'Exception in readALL: {e}')
                    self.setStickNeutral()
                    #print('Setting stick neutral because joy_fd = -1')
            else:
                self.setStickNeutral()
                #print('RCIO',self.rcsignals)
        else:
            for i in range(self.num_channels):
                value = self.read(i)
                self.rcsignals[i] = value
        ##Map rcsignals to TAER
        self.throttle = float(self.rcsignals[0])
        self.roll = float(self.rcsignals[1])
        self.pitch = float(self.rcsignals[2])
        self.yaw = float(self.rcsignals[3])
        self.armswitch = float(self.rcsignals[4])
        self.autopilot = float(self.rcsignals[5])
        #print('RCIO',self.rcsignals)
        #Turn TAER commands to floats between -1 and 1
        
        self.throttlerc = self.convert(self.throttle)
        self.rollrc = self.convert(self.roll)
        self.pitchrc = self.convert(self.pitch)
        self.yawrc = self.convert(self.yaw)
        ##Signal loss detection: valid PWM is always >= 995 us.
        ##If throttle reads below 800, the receiver has lost the transmitter.
        self.signal_lost = (self.throttle < 800)
        
        if not self.ARMED:
            #system is not armed. Let's try and arm it
            #Independent Safety precautions to arm system
            if (self.armswitch > self.STICK_MID) and (self.throttle < self.STICK_MIN*1.1):
                self.ARMED = True
                self.color = 'Green' #for go
            #Let's say armswitch is is >1500 but throttle is not down
            if (self.armswitch > self.STICK_MID) and (self.throttle > self.STICK_MIN*1.1):
                #we aren't going to arm the system but....
                #we are going to tell the user that you need to have throttle down
                self.color = 'Yellow'
                #No matter what though
        if self.armswitch < self.STICK_MID:
            self.ARMED = False
            self.color = 'Red'
        return self.ARMED,self.color

    def convert(self,signal):
        #y2 - y1 = m*(x2 - x1)
        #y2 = m*(x2-x1) + y1
        #rc = m*(signal - self.STICK_MIN) + -1
        #m = (1 - -1) / (self.STICK_MAX - self.STICK_MIN)
        return 2.0/(self.STICK_MAX-self.STICK_MIN) * (signal - self.STICK_MIN) - 1.0
        #return (signal - self.STICK_MID)/((self.STICK_MAX-self.STICK_MIN)/2)
        
    def read(self, ch):
        if self.MODE == 'AUTO' or self.MODE == 'HIL':
            value = self.channels[ch].read()
            position = self.channels[ch].seek(0, 0)
            return value[:-1]
        else:
            if ch == 0:
                return self.STICK_MIN
            else:
                return self.STICK_MID
            
