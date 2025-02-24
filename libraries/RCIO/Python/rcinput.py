class RCInput():
    CHANNEL_COUNT = 14
    channels = []

    def __init__(self,num_channels=9):
        self.num_channels = num_channels
        self.period = [0]*num_channels
        for i in range(0, self.CHANNEL_COUNT):
            try:
                f = open("/sys/kernel/rcio/rcin/ch%d" % i, "r")
                self.channels.append(f)
            except: 
                print ("Can't open file /sys/kernel/rcio/rcin/ch%d" % i)
    
    def read(self, ch):
        value = self.channels[ch].read()
        position = self.channels[ch].seek(0, 0)
        return value[:-1]
        
