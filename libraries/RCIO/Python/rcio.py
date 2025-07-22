import util
import os.path

class RCIO():
	SERVO_MIN = 0.995 #ms
	SERVO_MID = 1.504 #ms
	SERVO_MAX = 2.010 #ms
	def __init__(self,NUMRX,NUMPWM):
		self.rcin = RCInput(self.SERVO_MIN,self.SERVO_MID,self.SERVO_MAX,NUMRX)
		self.pwms = []
		for i in range(0,NUMPWM):
			self.pwms.append(PWM(i))
			self.pwms[i].initialize()
			self.pwms[i].set_period(50)
			self.pwms[i].enable()

class CHANNELS():
    def __init__(self,num_outputs):
        channels = []
        for i in range(0,num_outputs):
            channels.append(PWM(i))
            channels[i].initialize()
            channels[i].set_period(50)
            channels[i].enable()

class PWM():
    SYSFS_PWM_PATH_BASE = "/sys/class/pwm/pwmchip0/"
    SYSFS_PWM_EXPORT_PATH = "/sys/class/pwm/pwmchip0/export"
    SYSFS_PWM_UNEXPORT_PATH = "/sys/class/pwm/pwmchip0/unexport"

    def __init__(self, channel):
        self.channel = channel
        self.channel_path = self.SYSFS_PWM_PATH_BASE + "pwm{}/".format(self.channel)
        self.is_initialized = False
        self.is_enabled = False
        self.SIL = util.isSIL()

    def __enter__(self):
        self.initialize()
        return self

    def __exit__(self, *args):
        self.deinitialize()

    def deinitialize(self):
        if self.is_enabled:
            self.set_period(1)
            self.disable()
        with open(self.SYSFS_PWM_UNEXPORT_PATH, "a") as pwm_unexport:
            pwm_unexport.write(str(self.channel))

    def initialize(self):
        if not os.path.exists(self.SYSFS_PWM_PATH_BASE):
            raise OSError("rcio_pwm module wasn't loaded")
        if self.SIL:
            print('Emulating PWM signal. Initializing pin.....',self.channel)
        else:
            if not os.path.exists(self.channel_path):
                with open(self.SYSFS_PWM_EXPORT_PATH, "a") as pwm_export:
                    pwm_export.write(str(self.channel))
            print('Initializing pin.....',self.channel)
        self.is_initialized = True

    def enable(self):
        if self.SIL:
            print('Emulating PWM signal. Enabling pin.....',self.channel)
        else:
            with open(self.channel_path + "enable", "w") as pwm_enable:
                pwm_enable.write("1")
            print('Enabling pin.....',self.channel)
        self.is_enabled = True

    def disable(self):
        with open(self.channel_path + "enable", "w") as pwm_enable:
            pwm_enable.write("0")
            self.is_enabled = False

    def set_period(self, freq):
        if not self.is_initialized:
            raise RuntimeError("PWM not initialized. Call initialize first")

        period_ns = int(1e9/freq)
        if self.SIL:
            print('Emulating PWM signal. Setting frequency for pin....',freq,self.channel)
        else:
            with open(self.channel_path + "period",  "w") as pwm_period:
                pwm_period.write(str(period_ns))
            print('Setting frequency for pin....',freq,self.channel)

    def set_duty_cycle(self, period):
        if not self.is_initialized:
            raise RuntimeError("PWM not initialized. Call initialize first")

        period_ns = int(period*1e6)
        if not self.SIL:
            with open(self.channel_path + "duty_cycle", "w") as pwm_duty:
                pwm_duty.write(str(period_ns))

class RCInput():
    CHANNEL_COUNT = 14
    channels = []

    def __init__(self,SERVO_MIN,SERVO_MID,SERVO_MAX,num_channels=9):
        print('Initializing RCInput....')
        self.SERVO_MID = SERVO_MID
        self.SERVO_MAX = SERVO_MAX
        self.SERVO_MIN = SERVO_MIN
        self.num_channels = num_channels
        self.rcsignals = [0]*num_channels
        self.SIL = util.isSIL()
        for i in range(0, self.CHANNEL_COUNT):
            try:
                if self.SIL:
                    f = i
                    print('Running in SIL mode....emulating RCinput = ',f)
                else:
                    f = open("/sys/kernel/rcio/rcin/ch%d" % i, "r")
                    print('Opening RCinput channel = ',f)
                self.channels.append(f)
            except: 
                print ("Can't open file /sys/kernel/rcio/rcin/ch%d" % i)
        print('RCInput initialized')

    def readALL(self):
        for i in range(self.num_channels):
            value = self.read(i)
            self.rcsignals[i] = value
        #Turn receiver commands to floats
        self.rollrc = float(self.rcsignals[0])/1000.
        self.pitchrc = float(self.rcsignals[1])/1000.
        self.throttlerc = float(self.rcsignals[2])/1000.
        self.yawrc = float(self.rcsignals[3])/1000.
        self.armswitch = float(self.rcsignals[4])/1000.
        self.autopilot = float(self.rcsignals[6])/1000.
        #print(throttlerc,rollrc,pitchrc,yawrc,armswitch)
    
    def read(self, ch):
        if not self.SIL:
            value = self.channels[ch].read()
            position = self.channels[ch].seek(0, 0)
            return value[:-1]
        else:
            if ch == 0:
                return self.SERVO_MIN
            else:
                return self.SERVO_MID
            
