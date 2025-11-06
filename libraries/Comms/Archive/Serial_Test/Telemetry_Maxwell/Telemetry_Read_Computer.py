#!/usr/bin/env python3
import serial
from serial import Serial
import time

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
    ser.flush()
    
    while True:
        if ser.in_waiting > 0:
        	print('Reading char =')
        	line = ser.readline().decode('utf-8').rstrip()
        	print(line)
        	print('\n')