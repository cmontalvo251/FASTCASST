#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import time
import struct

def bitsToFloat(b):
	if (b > 2147483647):
		b = -2147483648 + (b - 2147483647)
	s = struct.pack('>l', b)
	return struct.unpack('>f', s)[0]

def lineToFloat(line):
	##Ok this loop works. Time to unpack properly
	#Remove first 2 chars because that's just the number
	#followed by a colon (Also remove the trailing two characters. They are \r\n I think)
	hexdata = line[2:-2]
	#Ok now what we do is convert the hex number to an int
	integer = int(hexdata,16)
	#Then finally we convert the int bits to float
	value = bitsToFloat(integer)
	#And print it for debugging
	print('0x = ',hexdata,' intbits = ',integer,' float = ',value)
	return value

def readFile(filename):
	##Alright when running in SIL mode we will need to read the csv files.
	#First we are expecting 6 packets from FASTKit
	telemetry_packet = np.zeros(6)
	file = open(filename)
	counter = 0
	for line in file:
		telemetry_packet[counter] = lineToFloat(line)
		counter+=1
	file.close()
	return telemetry_packet

#Print the received telemetry packet
filenumber = 0
while True:
	time.sleep(0.1)
	filename = '../' + str(filenumber) + '.csv'
	telemetry_packet = readFile(filename)
	filenumber+=1
	print(telemetry_packet)
