#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import time
import os
import struct
import datetime
import sys
sys.path.append('../libraries/')
from UART.uart import UART as U

fastkit_packet = np.zeros(6)

##Open Serial Window
print('Opening Serial port')
print('All available serial ports...')
os.system('ls /dev/ttyUSB*')
ser = U(57600,"/dev/ttyUSB0",period=1.0) #Set the baudrate, port and period in seconds
##Initialize Filenumber at zero
while True:
    position = -1
    #print('Reading Serial....')
    value,position,bytestring = ser.SerialGetNumber(0)
    #print('Value Received, Position, Bytes = ',value,position,bytestring)
    if position >= 0:
        fastkit_packet[position] = value
        #Print
        if position == 0:
                st = ''
                for i in fastkit_packet:
                        st+=(str(i)+',')
                print('Packets Received = ',st)
    #Then sleep for 0.1 seconds 
    time.sleep(0.1)