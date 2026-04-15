import sys
import util
import numpy as np
import datetime

class Datalogger():
	def __init__(self,NUMOUTPUTS):
		self.number = 0
		self.SIL = util.isSIL()
		print('Input arguments = ',sys.argv)
		if len(sys.argv) > 1:
			print('Using Directory = ',sys.argv[1])
		else:
		    sys.exit('No input argument given for datalogging directory')
		self.setfilename(sys.argv[1])
		self.open()
		#create an array for data
		self.outdata = np.zeros(NUMOUTPUTS)
		self._write_count = 0

	def setfilename(self,directory,extension='.txt'):
		timestamp = datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
		self.filename = directory + timestamp + extension
		print("Log file = " + self.filename)

	def open(self):
		print("Attempting to open" + self.filename);
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