#!usr/bin/env python3
import serial
from serial import Serial
import time
from random import *

Counter = 0

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
    ser.flush()
    
    while True:
        #ser.write("255\n".encode('utf-8'))
        RPi = ser.readline().decode('utf-8').rstrip()
        print(RPi)
        #Num1 = random()
        #String1 = str(Num1)
        #Counter = 0
        String1 = str(Counter)
        String2 = RPi + " " + String1  + "\n"
        Counter = Counter + 1
        #Response = RPi + " 255\n"
        #print(String2)
        ser.write(String2.encode('utf-8'))
        time.sleep(1)