import sys
import Util.util
import numpy as np
import datetime

class Datalogger():
	def __init__(self,directory,NUMOUTPUTS):
		self.number = 0		
		self.directory = directory
		self.setfilename()
		self.open()
		#create an array for data
		self.NUMOUTPUTS = NUMOUTPUTS
		self.outdata = np.zeros(self.NUMOUTPUTS)
		self._write_count = 0

	def setfilename(self,extension='.txt'):
		##Use datetime name only if the system clock looks valid (year >= 2024).
		##Without NTP or an RTC the Pi boots to a wrong date, so fall back to
		##incremental numbering (0.txt, 1.txt, …) instead.
		if datetime.datetime.now().year >= 2024:
			timestamp = datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
			self.filename = self.directory + timestamp + extension
			print("Log file = " + self.filename)
		else:
			print("System clock not synced — using incremental filename.")
			number = 0
			while True:
				self.filename = self.directory + str(number) + extension
				try:
					open(self.filename, 'r').close()
					number += 1
				except FileNotFoundError:
					break
			print("Log file = " + self.filename)

	def writeheader(self,headers,TYPE):		
		self.outfile.write(headers)
		NUMCONTROLS = self.NUMOUTPUTS - 36
		for i in range(0,NUMCONTROLS):
			s = ',     PWM ' + TYPE + ' Out ' + str(i+1) + ' (-1/1)'
			self.outfile.write(s)
		self.outfile.write('\n')

	def open(self):
		print("Attempting to open " + self.filename);
		self.outfile = open(self.filename,"w");
		if not self.outfile:
			print("File not opened properly = " + self.filename);
		else:
			print("File " + self.filename + " opened successfully")

	def println(self):
		ctr = 0
		out = self.outdata
		for o in out:
			s = str(o)
			if ctr != len(out)-1:
				s+=","
			ctr+=1
			self.outfile.write(s)
		self.outfile.write("\n")
		self._write_count += 1
		##Flush to SD card every 50 writes (~5 sec at 10 Hz) instead of every write.
		##Flushing every write causes 50-300 ms SD card stalls that block RC and PWM.
		if self._write_count % 50 == 0:
			self.outfile.flush()

	#Close function
	def close(self):
		print("Closing File");
		try:
			self.outfile.close()
			print("File closed");
		except AttributeError:
	  		print('You have no file to close')