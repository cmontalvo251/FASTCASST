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
os.system('make sil MODEL="quadcopter RX="XBOX"')
os.system('./sil.exe quadcopter/')
##Create PDF Handle
pp = PDF(0,plt)
#Open File
datafile = open('data/0.csv','r')
logfile = open('logs/0.csv','r')
dataheaders = datafile.readline().split(',')
logheaders = logfile.readline().split(',')
numVars = len(logheaders)
print('Number of Vars = ',numVars)
print(logheaders)
#Grab entire data file
sense_data = []
model_data = []
for line in datafile:
    row = line.split(',')
    numarray = [np.float(x) for x in row]
    sense_data.append(numarray)
sense_data = np.array(sense_data)
for line in logfile:
    row = line.split(',')
    numarray = [np.float(x) for x in row]
    model_data.append(numarray)
model_data = np.array(model_data)
#Plot everything
sense_time = sense_data[:,0]
model_time = model_data[:,0]
for x in range(1,numVars):
    fig = plt.figure()
    plti = fig.add_subplot(1,1,1)
    plti.plot(sense_time,sense_data[:,x],label=dataheaders[x])
    plti.plot(model_time,model_data[:,x],label=logheaders[x])
    plti.set_xlabel('Time (sec)')
    plti.set_ylabel(logheaders[x])
    print(logheaders[x])
    plti.grid()
    plti.legend()
    plti.get_yaxis().get_major_formatter().set_useOffset(False)
    plt.gcf().subplots_adjust(left=0.18)
    pp.savefig()
    
#Close file
datafile.close()
#Close PDF
pp.close()
