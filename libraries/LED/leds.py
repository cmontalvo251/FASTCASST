ON = 0
OFF = 1
import util

class Pin():
    def __init__(self,mode,folder_name,color):
        self.pin = folder_name
        self.color = color
        self.MODE=mode
    
    def write(self, value):
        if self.MODE != 'AUTO':
            if value == 1:
                status = 'OFF'
            else:
                status = 'ON'
            #print('Running in emulation mode so printing color and mode....',self.color,status)
        else:
            with open("/sys/class/leds/%s/brightness" % self.pin, "w") as value_file:
                value_file.write(str(value))
            #print('Changing LED color = ',self.color,mode)

class Led():

    def __init__(self,mode):
        print('Initializing LEDs....')
        self.ledR = Pin(mode,"rgb_led0","red")
        self.ledB = Pin(mode,"rgb_led1","blue")
        self.ledG = Pin(mode,"rgb_led2","green")

        self.ledR.write(OFF) 
        self.ledG.write(OFF) 
        self.ledB.write(OFF) 
        #initialize to yellow
        self.setColor('Yellow')
        print('LEDs Initialized')
    
    def setColor(self, color):
        self.ledR.write(self.gamma[color][0]) 
        self.ledG.write(self.gamma[color][1]) 
        self.ledB.write(self.gamma[color][2])
    
    gamma = {
        'Black':    (OFF, OFF, OFF),
        'Red':      (ON, OFF, OFF),
        'Green':    (OFF, ON, OFF),
        'Blue':     (OFF, OFF, ON),
        'Cyan':     (OFF, ON, ON),
        'Magenta':  (ON, OFF, ON),
        'Yellow':   (ON,ON,OFF),
        'White':    (ON, ON, ON)
    }
