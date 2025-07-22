import util

class RCInput():
    CHANNEL_COUNT = 14
    channels = []
    SERVO_MIN = 995 #ms
    SERVO_MID = 1504 #ms
    SERVO_MAX = 2010 #ms

    def __init__(self,num_channels=9):
        print('Initializing RCInput....')
        self.num_channels = num_channels
        self.period = [0]*num_channels
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
            
