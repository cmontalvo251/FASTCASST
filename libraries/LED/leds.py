ON = 0
OFF = 1
import util

class Pin():
    def __init__(self, folder_name,color):
        self.pin = folder_name
        self.color = color
        self.SIL = util.isSIL()
    
    def write(self, value):
        if self.SIL:
            if value == 1:
                mode = 'OFF'
            else:
                mode = 'ON'
            print('Running in SIL mode so printing color and mode....',self.color,mode)
        else:
            with open("/sys/class/leds/%s/brightness" % self.pin, "w") as value_file:
                value_file.write(str(value))

class Led():

    def __init__(self):
        self.ledR = Pin("rgb_led0","red")
        self.ledB = Pin("rgb_led1","blue")
        self.ledG = Pin("rgb_led2","green")

        self.ledR.write(OFF) 
        self.ledG.write(OFF) 
        self.ledB.write(OFF) 
    
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
