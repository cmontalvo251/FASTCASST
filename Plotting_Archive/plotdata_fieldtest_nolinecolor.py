#!/usr/bin/python3

import matplotlib.pyplot as plt
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

##TRUNCATION START AND END TIME (Set to negative to turn off)
tstart = -99
tend = -99

##Create PDF Handle
pp = PDF(0,plt)
#Open File
datafile = open('data/0.csv','r')
dataheaders = datafile.readline().split(',')
numVars = len(dataheaders)
print('Number of Vars = ',numVars)
print(dataheaders)
#Grab entire data file
sense_data = []
for line in datafile:
    row = line.split(',')
    numarray = [np.float(x) for x in row]
    sense_data.append(numarray)
sense_data = np.array(sense_data)

#Plot everything
sense_time = sense_data[:,0]
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
    #print(x)
    sense_trunc = sense_data[istart_sense:iend_sense,x]
    sense_flightdata.append(sense_trunc)
sense_flightdata = np.array(sense_flightdata)

#Plot All Flight Data vs Time
for x in range(0,numVars-1):
    flightdata = sense_flightdata[x]
    fig = plt.figure()
    plti = fig.add_subplot(1,1,1)
    plti.plot(sense_flighttime,flightdata,'b',label=dataheaders[x])
    plti.set_xlabel('Time (sec)')
    plti.set_ylabel(dataheaders[x+1])
    print(dataheaders[x+1])
    plti.grid()
    #plti.legend()
    plti.get_xaxis().get_major_formatter().set_useOffset(False)
    plti.get_yaxis().get_major_formatter().set_useOffset(False)
    plt.gcf().subplots_adjust(left=0.18)
    pp.savefig()

X_Pos = sense_flightdata[0]
Y_Pos = sense_flightdata[1]
fig = plt.figure()    
plti = fig.add_subplot(1,1,1)
plti.plot(X_Pos,Y_Pos,'b')
plti.set_xlabel('X (m)')
plti.set_ylabel('Y (m)')
plti.grid()
#plti.legend()
plti.get_yaxis().get_major_formatter().set_useOffset(False)
plti.get_xaxis().get_major_formatter().set_useOffset(False)
plt.gcf().subplots_adjust(left=0.18)
pp.savefig()

Lat = sense_flightdata[15]
Lon = sense_flightdata[16]
fig = plt.figure()    
plti = fig.add_subplot(1,1,1)
plti.plot(Lat,Lon,'b')
plti.set_ylabel('Longitude (deg)')
plti.set_xlabel('Latitude (deg)')
plti.grid()
#plti.legend()
plti.get_yaxis().get_major_formatter().set_useOffset(False)
plti.get_xaxis().get_major_formatter().set_useOffset(False)
plt.gcf().subplots_adjust(left=0.18)
pp.savefig()

#Close file
datafile.close()
#Close PDF
pp.close()
