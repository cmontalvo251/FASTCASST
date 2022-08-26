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

##Alright when running in SIL mode we will need to read the csv files.
##For now let's just read 0.csv and see if we can unpack what's in it
file = open('../1.csv')
##Ok this loop works. Time to unpack properly
#First we are expecting 6 packets from FASTKit
telemetry_packet = np.zeros(6)
counter = 0
for line in file:
	#Remove first 2 chars because that's just the number
	#followed by a colon (Also remove the trailing two characters. They are \r\n I think)
	hexdata = line[2:-2]
	#Ok now what we do is convert the hex number to an int
	integer = int(hexdata,16)
	#Then finally we convert the int bits to float
	value = bitsToFloat(integer)
	#And print it for debugging
	print('0x = ',hexdata,' intbits = ',integer,' float = ',value)
	#Then put it in the telemetry_packet
	telemetry_packet[counter] = value
	counter+=1
file.close()
#Print the received telemetry packet
print(telemetry_packet)