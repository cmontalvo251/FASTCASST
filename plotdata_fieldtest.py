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

sense_flightcontroller = sense_flightdata[29]
controllerON = np.where(sense_flightcontroller>1000)
controlON = np.array(controllerON)
controlON = np.matrix.transpose(controlON)
controlEnd = []
counter = controllerON[0][0]
for x in range(0,len(controlON)):
    if controllerON[0][x] != counter:
        controlEnd.append(controllerON[0][x-1]+1)
        counter = controllerON[0][x]
    counter += 1
controlEnd.append(controllerON[0][-1]+1)
#print(controlEnd)

controllerOFF = np.where(sense_flightcontroller<1000)
controlOFF = np.array(controllerOFF)
controlOFF = np.matrix.transpose(controlOFF)
controlBegin = []
counter = controllerOFF[0][0]
for x in range(0,len(controlOFF)):
    if controllerOFF[0][x] != counter:
        controlBegin.append(controllerOFF[0][x-1]-1)
        counter = controllerOFF[0][x]
    counter += 1
#print(controlBegin)

n = len(controlBegin)
controllists = [[] for _ in range(n)]
nocontrollists = [[] for _ in range(n-1)]
flighttime_Start = sense_flighttime[0:controlBegin[0]+1]
flighttime_End = sense_flighttime[controlEnd[-1]-1:iend_sense]
#print(flighttime_Start[0],flighttime_Start[-1],flighttime_End[0],flighttime_End[-1])
for x in range(0,n):
    controllists[x] = sense_flighttime[controlBegin[x]:controlEnd[x]]
for x in range(0,n-1):
    if x<n:
        nocontrollists[x] = sense_flighttime[controlEnd[x]-1:controlBegin[x+1]+1]
#print(controllists[3])

#Plot All Flight Data vs Time
for x in range(0,numVars-1):
    flightdata = sense_flightdata[x]
    fig = plt.figure()
    plti = fig.add_subplot(1, 1, 1)
    plti.plot(flighttime_Start,flightdata[0:controlBegin[0]+1],'b',label='Manual Flight')
    for i in range(0,n):
        controlTime = controllists[i]
        if i<1:
            plti.plot(controlTime,flightdata[controlBegin[i]:controlEnd[i]],'r',label='Controlled Flight')
        else:
            plti.plot(controlTime,flightdata[controlBegin[i]:controlEnd[i]],'r')
    for ii in range(0,n-1):
        nonControl = nocontrollists[ii]
        plti.plot(nonControl,flightdata[controlEnd[ii]-1:controlBegin[ii+1]+1],'b')
    plti.plot(flighttime_End,flightdata[controlEnd[-1]-1:iend_sense],'b')
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
fig = plt.figure()
plti = fig.add_subplot(1, 1, 1)
plti.plot(flighttime_Start,Baro[0:controlBegin[0]+1],'g',label='Baro Manual Flight')
plti.plot(flighttime_End,Baro[controlEnd[-1]-1:iend_sense],'g')
plti.plot(flighttime_Start,GPSAlt[0:controlBegin[0]+1],'b',label='GPS Manual Flight')
plti.plot(flighttime_End,GPSAlt[controlEnd[-1]-1:iend_sense],'b')
for i in range(0,n):
    controlTime = controllists[i]
    if i<1:
        plti.plot(controlTime,Baro[controlBegin[i]:controlEnd[i]],'m',label='Baro Controlled Flight')
    else:
        plti.plot(controlTime,Baro[controlBegin[i]:controlEnd[i]],'m')
for ii in range(0,n-1):
    nonControl = nocontrollists[ii]
    plti.plot(nonControl,Baro[controlEnd[ii]-1:controlBegin[ii+1]+1],'g')
for i in range(0,n):
    controlTime = controllists[i]
    if i<1:
        plti.plot(controlTime,GPSAlt[controlBegin[i]:controlEnd[i]],'r',label='GPS Controlled Flight')
    else:
        plti.plot(controlTime,GPSAlt[controlBegin[i]:controlEnd[i]],'r')
for ii in range(0,n-1):
    nonControl = nocontrollists[ii]
    plti.plot(nonControl,GPSAlt[controlEnd[ii]-1:controlBegin[ii+1]+1],'b')
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
fig = plt.figure()
plti = fig.add_subplot(1, 1, 1)
plti.plot(X_Pos[0:controlBegin[0]+1],Y_Pos[0:controlBegin[0]+1],'b',label='Manual Flight')
plti.plot(X_Pos[controlEnd[-1]-1:iend_sense],Y_Pos[controlEnd[-1]-1:iend_sense],'b')
for i in range(0,n):
    controlTime = controllists[i]
    if i<1:
        plti.plot(X_Pos[controlBegin[i]:controlEnd[i]],Y_Pos[controlBegin[i]:controlEnd[i]],'r',label='Controlled Flight')
    else:
        plti.plot(X_Pos[controlBegin[i]:controlEnd[i]],Y_Pos[controlBegin[i]:controlEnd[i]],'r')
for ii in range(0,n-1):
    nonControl = nocontrollists[ii]
    plti.plot(X_Pos[controlEnd[ii]-1:controlBegin[ii+1]+1],Y_Pos[controlEnd[ii]-1:controlBegin[ii+1]+1],'b')
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
fig = plt.figure()
plti = fig.add_subplot(1, 1, 1)
plti.plot(Lat[0:controlBegin[0]+1],Lon[0:controlBegin[0]+1],'b',label='Manual Flight')
plti.plot(Lat[controlEnd[-1]-1:iend_sense],Lon[controlEnd[-1]-1:iend_sense],'b')
for i in range(0,n):
    controlTime = controllists[i]
    if i<1:
        plti.plot(Lat[controlBegin[i]:controlEnd[i]],Lon[controlBegin[i]:controlEnd[i]],'r',label='Controlled Flight')
    else:
        plti.plot(Lat[controlBegin[i]:controlEnd[i]],Lon[controlBegin[i]:controlEnd[i]],'r')
for ii in range(0,n-1):
    nonControl = nocontrollists[ii]
    plti.plot(Lat[controlEnd[ii]-1:controlBegin[ii+1]+1],Lon[controlEnd[ii]-1:controlBegin[ii+1]+1],'b')
plti.set_ylabel('Longitude (deg)')
plti.set_xlabel('Latitude (deg)')
plti.grid()
plti.legend()
plti.get_xaxis().get_major_formatter().set_useOffset(False)
plti.get_yaxis().get_major_formatter().set_useOffset(False)
plt.gcf().subplots_adjust(left=0.18)
pp.savefig()

#Close file
datafile.close()
#Close PDF
pp.close()
