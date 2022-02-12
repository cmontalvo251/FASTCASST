#!/usr/bin/python3

import matplotlib.pyplot as plt
plt.rcParams.update({'figure.max_open_warning': 0})
import sys
import os
import numpy as np

try:
    from pdf import *
    import sixdof as dof
except:
    print('You need pdf and sixdof from Python.git This is on my Github just git clone that repo and put pdf.py and sixdof.py in this root or add to pythonpath')
    sys.exit()

#Run code
os.system('./clean_logs')
os.system('make simonly')
os.system('./simonly.exe')
##Create PDF Handle
pp = PDF(0,plt)
#Open File
datafile = open('data/0.csv','r')
headers = datafile.readline().split(',')
numVars = len(headers)
print('Number of Vars = ',numVars)
print(headers)
#Grab entire data file
data = []
for line in datafile:
    row = line.split(',')
    numarray = [np.float(x) for x in row]
    data.append(numarray)
data = np.array(data)
#Plot everything
time = data[:,0]
for x in range(1,numVars):
    fig = plt.figure()
    plti = fig.add_subplot(1,1,1)
    plti.plot(time,data[:,x])
    plti.set_xlabel('Time (sec)')
    plti.set_ylabel(headers[x])
    print(headers[x])
    plti.grid()
    plti.get_yaxis().get_major_formatter().set_useOffset(False)
    plt.gcf().subplots_adjust(left=0.18)
    pp.savefig()
    
#Close file
datafile.close()
#Close PDF
pp.close()
