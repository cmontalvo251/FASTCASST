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
	#print('0x = ',hexdata,' intbits = ',integer,' float = ',value)
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

def getFileName(filenumber):
	return '../' + str(filenumber) + '.csv'

def maxFile(filenumber):
	searching = 1
	while searching:
		try:
			file = open(getFileName(filenumber))
			file.close()
			filenumber+=1
		except:
			#If you throw an error it means that
			#you found the last file
			searching = 0
	return filenumber-1

###Alright infinite while loop
filenumber = 0 #Initialize filenumber at 0
while True:
	##What we need to do first is find the largest csv file in the directory
	##So we basically loop until we find the maximum filenumber
	filenumber = maxFile(filenumber)
	print('FastKit at = ',filenumber,' Python Time = ',time.monotonic())
	#Now this means though we need to read the file before
	if filenumber > 0:
		print('Reading File = ',filenumber-1)
		telemetry_packet = readFile(getFileName(filenumber-1))	
		print(telemetry_packet)
	#Then sleep for 0.1 seconds
	time.sleep(0.1)
