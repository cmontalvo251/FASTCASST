#!/usr/bin/python3

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
plt.rcParams.update({'figure.max_open_warning': 0})
import sys
import os
import numpy as np

try:
    from pdf import *
    #import sixdof as dof
except:
    print('You need pdf and sixdof from Python.git This is on my Github just git clone that repo and put pdf.py and sixdof.py in this root or add to pythonpath')
    sys.exit()

##TRUNCATION START AND END TIME of flight (Set to negative to turn off)
tstart = 10
tend = -99

##Create PDF Handle
pp = PDF(0,plt)
#Open File
#datafile = open('data/RASPI_TABLE_DATA_4_25_2023.csv','r')
datafile = open('data/ARDUINO_TABLE_DATA_4_25_2023.csv','r')
dataheaders = datafile.readline().split(',')
numVars = len(dataheaders)
print('Number of Vars = ',numVars)
print(dataheaders)
#Grab entire data file
sense_data = []
for line in datafile:
    row = line.split(',')
    #print(row,len(row))
    if len(row) > numVars/2.0:
        numarray = [np.float(x) for x in row]
        sense_data.append(numarray)
sense_data = np.array(sense_data)
sense_time = sense_data[:,0]

#Flight Time Truncation
if tstart > 0:
    istart_sense = np.where(sense_time>tstart)[0][0]
else:
    istart_sense = 0
if tend > 0:
    iend_sense = np.where(sense_time>tend)[0][0]
else:
    iend_sense = -1
sense_flighttime = sense_time[istart_sense:iend_sense]
sense_flightdata = []
for x in range(1,numVars):
    sense_trunc = sense_data[istart_sense:iend_sense,x]
    sense_flightdata.append(sense_trunc)
sense_flightdata = np.array(sense_flightdata)

#Plot All Flight Data vs Time
for x in range(0,numVars-1):
    fig = plt.figure()
    plti = fig.add_subplot(1, 1, 1)
    plti.plot(sense_flighttime,sense_flightdata[x])
    plti.set_xlabel('Time (sec)')
    plti.set_ylabel(dataheaders[x+1])
    print(dataheaders[x+1],np.mean(sense_flightdata[x]),np.std(sense_flightdata[x]))
    plti.grid()
    plti.get_yaxis().get_major_formatter().set_useOffset(False)
    plti.get_xaxis().get_major_formatter().set_useOffset(False)
    plt.gcf().subplots_adjust(left=0.18)
    pp.savefig()

#Close file
datafile.close()
#Close PDF
pp.close()
