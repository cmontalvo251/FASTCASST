#!/usr/bin/env python3
import serial
from serial import Serial
import time
from random import *

Counter = 0
#Counter = Counter + 1

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyAMA0', 57600, timeout=1)
    ser.flush()
    
    while True:
        #Num1 = random()
        #String1 = str(Num1)
        #Counter = 0
        Counter = Counter + 1
        String1 = str(Counter)
        String2 = String1 + "\n"
        ser.write(String2.encode('utf-8'))
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        time.sleep(1)
