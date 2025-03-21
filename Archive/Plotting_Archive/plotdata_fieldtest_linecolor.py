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
tstart = 110
tend = 250
##TRUNCATION START AND END TIME of control (Set to negative to turn off)
cstart = 130
cend = 165

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
    #print(x)
    sense_trunc = sense_data[istart_sense:iend_sense,x]
    sense_flightdata.append(sense_trunc)
sense_flightdata = np.array(sense_flightdata)

#Control Time Truncation
if cstart > 0:
    control_start = np.where(sense_flighttime>cstart)[0][0]
else:
    control_start = 0
if cend > 0:
    control_end = np.where(sense_flighttime>cend)[0][0]
else:
    control_end = -1
flighttime_Start = sense_flighttime[0:control_start]
flighttime_Control = sense_flighttime[control_start-10:control_end]
flighttime_End = sense_flighttime[control_end-10:-1]

#Plot All Flight Data vs Time
for x in range(0,numVars-1):
    flightdata = sense_flightdata[x]
    fig = plt.figure()
    plti = fig.add_subplot(1, 1, 1)
    plti.plot(flighttime_Start,flightdata[0:control_start],'b',label='Manual Flight')
    plti.plot(flighttime_Control,flightdata[control_start-10:control_end],'r',label='Controlled Flight')
    plti.plot(flighttime_End,flightdata[control_end-10:-1],'b')
    plti.set_xlabel('Time (sec)')
    plti.set_ylabel(dataheaders[x+1])
    print(dataheaders[x+1])
    plti.grid()
    plti.legend()
    plti.get_yaxis().get_major_formatter().set_useOffset(False)
    plti.get_xaxis().get_major_formatter().set_useOffset(False)
    plt.gcf().subplots_adjust(left=0.18)
    pp.savefig()

#Plot Barometer vs GPS Altitude
Baro = sense_flightdata[2]*-1
GPSAlt = sense_flightdata[17]
Baro_Start = Baro[0:control_start]
Baro_Control = Baro[control_start-10:control_end]
Baro_End = Baro[control_end-10:-1]
GPSAlt_Start = GPSAlt[0:control_start]
GPSAlt_Control = GPSAlt[control_start-10:control_end]
GPSAlt_End = GPSAlt[control_end-10:-1]
fig = plt.figure()
plti = fig.add_subplot(1, 1, 1)
plti.plot(flighttime_Start,Baro_Start,'g',label='Baro(Manual)')
plti.plot(flighttime_Control,Baro_Control,'m',label='Baro(Controlled)')
plti.plot(flighttime_End,Baro_End,'g')
plti.plot(flighttime_Start,GPSAlt_Start,'b',label='GPS(Manual)')
plti.plot(flighttime_Control,GPSAlt_Control,'r',label='GPS(Controlled)')
plti.plot(flighttime_End,GPSAlt_End,'b')
plti.set_xlabel('Time (sec)')
plti.set_ylabel('Altitude (m)')
plti.grid()
plti.legend() 
plti.get_yaxis().get_major_formatter().set_useOffset(False)
plti.get_xaxis().get_major_formatter().set_useOffset(False)
plt.gcf().subplots_adjust(left=0.18)
pp.savefig()

#Plot X vs Y Position
X_Pos = sense_flightdata[0]
Y_Pos = sense_flightdata[1]
X_Pos_Start = X_Pos[0:control_start]
X_Pos_Control = X_Pos[control_start-10:control_end]
X_Pos_End = X_Pos[control_end-10:-1]
Y_Pos_Start = Y_Pos[0:control_start]
Y_Pos_Control = Y_Pos[control_start-10:control_end]
Y_Pos_End = Y_Pos[control_end-10:-1]
fig = plt.figure()
plti = fig.add_subplot(1, 1, 1)
plti.plot(X_Pos_Start,Y_Pos_Start,'b',label='Manual Flight')
plti.plot(X_Pos_Control,Y_Pos_Control,'r',label='Controlled Flight')
plti.plot(X_Pos_End,Y_Pos_End,'b')
plti.set_xlabel('X (m)')
plti.set_ylabel('Y (m)')
plti.grid()
plti.legend()
plti.get_yaxis().get_major_formatter().set_useOffset(False)
plti.get_xaxis().get_major_formatter().set_useOffset(False)
plt.gcf().subplots_adjust(left=0.18)
pp.savefig()

#Plot Latitude vs Longitude
Lat = sense_flightdata[15]
Lon = sense_flightdata[16]
Lat_Start = Lat[0:control_start]
Lat_Control = Lat[control_start-10:control_end]
Lat_End = Lat[control_end-10:-1]
Lon_Start = Lon[0:control_start]
Lon_Control = Lon[control_start-10:control_end]
Lon_End = Lon[control_end-10:-1]
fig = plt.figure()
plti = fig.add_subplot(1, 1, 1)
plti.plot(Lat_Start,Lon_Start,'b',label='Manual Flight')
plti.plot(Lat_Control,Lon_Control,'r',label='Controlled Flight')
plti.plot(Lat_End,Lon_End,'b')
plti.set_ylabel('Longitude (deg)')
plti.set_xlabel('Latitude (deg)')
plti.grid()
plti.legend()
plti.get_yaxis().get_major_formatter().set_useOffset(False)
plti.get_xaxis().get_major_formatter().set_useOffset(False)
plt.gcf().subplots_adjust(left=0.18)
pp.savefig()

#Close file
datafile.close()
#Close PDF
pp.close()
