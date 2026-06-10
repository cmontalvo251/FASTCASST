import sys
import util
import numpy as np

class Datalogger():
	def __init__(self,NUMOUTPUTS):
		self.number = 0
		self.SIL = util.isSIL()
		print('Input arguments = ',sys.argv)
		if len(sys.argv) > 1:
			print('Using Directory = ',sys.argv[1])
		else:
		    sys.exit('No input argument given for datalogging directory')
		self.findfile(sys.argv[1])
		self.open()
		#create an array for data
		self.outdata = np.zeros(NUMOUTPUTS)

	def findfile(self,directory,extension='.txt'):
		found = 0
		if self.SIL:
			print('rm '+directory+'*'+extension)
		while not found:
			self.filename = directory + str(self.number) + extension
			print("Attempting to check for file: " + self.filename);
			try:
				fileout = open(self.filename,"r");
				fileout.close()
				print("File exists. Skipping");
				self.number+=1; #//Number is global in header file
			except FileNotFoundError:
				found = 1
				print("File found for writing = " + self.filename)

	def open(self):
		#If you want the date in the filename use this
		#datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
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
		self.outfile.flush()

	#Close function
	def close(self):
  		print("Closing File");
  		try:
  			self.outfile.close()
	  		print("File closed");
	  	except AttributeError:
	  		print('You have no file to close')