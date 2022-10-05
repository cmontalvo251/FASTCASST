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
    import sixdof as dof
except:
    print('You need pdf and sixdof from Python.git This is on my Github just git clone that repo and put pdf.py and sixdof.py in this root or add to pythonpath')
    sys.exit()

##TRUNCATION START AND END TIME of flight (Set to negative to turn off)
tstart = 110
tend = 245
##TRUNCATION START AND END TIME of control (Set to negative to turn off)
cstart = 125
cend = 155

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
#Flight Time
if tstart > 0:
    istart_sense = np.where(sense_time>tstart)[0][0]
else:
    istart_sense = 0
if tend > 0:
    iend_sense = np.where(sense_time>tend)[0][0]
else:
    iend_sense = -1
#Control Time
if cstart > 0:
    istart_control = np.where(sense_time>cstart)[0][0]
else:
    istart_control = 0
if cend > 0:
    iend_control = np.where(sense_time>cend)[0][0]
else:
    iend_control = -1
for x in range(1,numVars):
    points = np.array([sense_time[istart_sense:iend_sense],sense_data[istart_sense:iend_sense,x]]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    fig, axs = plt.subplots(1, 1, sharex=True, sharey=True)
    cmap = ListedColormap(['b','r','b'])
    norm = BoundaryNorm([sense_time[istart_sense],sense_time[istart_control],sense_time[iend_control],sense_time[iend_sense]], cmap.N)
    lc = LineCollection(segments, cmap=cmap, norm=norm)
    lc.set_array(sense_time[istart_sense:iend_sense])
    lc.set_linewidth(2)
    line = axs.add_collection(lc)
    #This sets the color bar on side
    #fig.colorbar(line, ax=axs)
    axs.set_xlabel('Time (sec)')
    axs.set_ylabel(dataheaders[x])
    print(dataheaders[x],x)
    axs.grid()
    axs.get_yaxis().get_major_formatter().set_useOffset(False)
    axs.gcf().subplots_adjust(left=0.18)
    axs.set_xlim(c.min()-10, c.max()+10)
    axs.set_ylim(a.min()-10, a.max()+10)
    pp.savefig()

#for x in range(1,numVars):
#    fig = plt.figure()
#    plti = fig.add_subplot(1,1,1)
#    plti.plot(sense_time[istart_sense:iend_sense],sense_data[istart_sense:iend_sense,x],label=dataheaders[x])
#    plti.set_xlabel('Time (sec)')
#    plti.set_ylabel(dataheaders[x])
#    print(dataheaders[x],x)
#    plti.grid()
#    plti.get_yaxis().get_major_formatter().set_useOffset(False)
#    plt.gcf().subplots_adjust(left=0.18)
#    pp.savefig()

fig = plt.figure()    
plti = fig.add_subplot(1,1,1)
#plti.plot(sense_data[:,1],sense_data[:,2],label='Sense')
#plti.set_xlabel('X (m)')
#plti.set_ylabel('Y (m)')
plti.plot(sense_data[:,17],sense_data[:,16],label='Sense')
plti.set_xlabel('Longitude (deg)')
plti.set_ylabel('Latitude (deg)')
plti.grid()
plti.get_yaxis().get_major_formatter().set_useOffset(False)
plt.gcf().subplots_adjust(left=0.18)
pp.savefig()
################################################################################################################################
a = sense_data[:,1]
b = sense_data[:,2]
c = sense_time
# Create a set of line segments so that we can color them individually
# This creates the points as a N x 1 x 2 array so that we can stack points
# together easily to get the segments. The segments array for line collection
# needs to be (numlines) x (points per line) x 2 (for x and y)
points = np.array([c, a]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

fig, axs = plt.subplots(1, 1, sharex=True, sharey=True)

cmap = ListedColormap(['b','r','b'])
norm = BoundaryNorm([sense_time[istart_sense],sense_time[istart_control],sense_time[iend_control],sense_time[iend_sense]], cmap.N)
lc = LineCollection(segments, cmap=cmap, norm=norm)
lc.set_array(c)
lc.set_linewidth(2)
line = axs.add_collection(lc) 
#This sets the color bar on side
#fig.colorbar(line, ax=axs)

axs.set_xlim(c.min()-10, c.max()+10)
axs.set_ylim(a.min()-10, a.max()+10)
#plt.show()
################################################################################################################################
#Close file
datafile.close()
#Close PDF
pp.close()
