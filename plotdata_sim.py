#!/usr/bin/python3

import matplotlib.pyplot as plt
plt.rcParams.update({'figure.max_open_warning': 0})
import sys
sys.path.append('supplemental_files/')
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

#Run code - Comment out section later?
os.system('./clean_logs')
os.system('rm simonly.exe')
#os.system('make clean')
os.system('make simonly MODEL="cubesat"')
os.system('./simonly.exe cubesat/')
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
    #print('line = ',line)
    row = line.split(',')
    #print('row = ',row)
    if len(row) > 1:
        #print('len(row) = ',len(row))
        numarray = [float(x) for x in row]
        sense_data.append(numarray)
sense_data = np.array(sense_data)
for line in logfile:
    row = line.split(',')
    if len(row) > 1:
        numarray = [float(x) for x in row]
        model_data.append(numarray)
model_data = np.array(model_data)
#Plot everything
sense_time = sense_data[:,0]
model_time = model_data[:,0]
if tstart > 0:
    istart_sense = np.where(sense_time>tstart)[0][0]
    istart_model = np.where(model_time>tstart)[0][0]
else:
    istart_sense = 0
    istart_model = 0
if tend > 0:
    iend_sense = np.where(sense_time>tend)[0][0]
    iend_model = np.where(model_time>tend)[0][0]
else:
    iend_sense = -1
    iend_model = -1
for x in range(1,numVars):
    fig = plt.figure()
    plti = fig.add_subplot(1,1,1)
    plti.plot(sense_time[istart_sense:iend_sense],sense_data[istart_sense:iend_sense,x],'b',label=dataheaders[x])
    plti.plot(model_time[istart_model:iend_model],model_data[istart_model:iend_model,x],'y',label=logheaders[x])
    plti.set_xlabel('Time (sec)')
    plti.set_ylabel(dataheaders[x])
    print(dataheaders[x],x)
    plti.grid()
    plti.legend()
    plti.get_yaxis().get_major_formatter().set_useOffset(False)
    plti.get_xaxis().get_major_formatter().set_useOffset(False)
    plt.gcf().subplots_adjust(left=0.18)
    pp.savefig()


###PLOT A X,Y GRAPH
fig = plt.figure()    
plti = fig.add_subplot(1,1,1)
plti.plot(sense_data[istart_sense:iend_sense,1],sense_data[istart_sense:iend_sense,2],'b',label='Sense')
plti.plot(model_data[istart_sense:iend_sense,1],model_data[istart_sense:iend_sense,2],'y',label='Model')
plti.set_xlabel('X (m)')
plti.set_ylabel('Y (m)')
plti.grid()
plti.legend()
plti.get_yaxis().get_major_formatter().set_useOffset(False)
plti.get_xaxis().get_major_formatter().set_useOffset(False)
plt.gcf().subplots_adjust(left=0.18)
pp.savefig()


###AND LAT/LON
fig = plt.figure()    
plti = fig.add_subplot(1,1,1)
plti.plot(sense_data[istart_sense:iend_sense,16],sense_data[istart_sense:iend_sense,17],'b',label='Sense')
plti.plot(model_data[istart_sense:iend_sense,16],model_data[istart_sense:iend_sense,17],'y',label='Model')
plti.set_ylabel('Longitude (deg)')
plti.set_xlabel('Latitude (deg)')
plti.grid()
plti.legend()
plti.get_yaxis().get_major_formatter().set_useOffset(False)
plti.get_xaxis().get_major_formatter().set_useOffset(False)
plt.gcf().subplots_adjust(left=0.18)
pp.savefig()

###JUST FOR CUBESAT PLOT MOMENTS AND PQR
fig = plt.figure()
plti = fig.add_subplot(1,1,1)
plti.plot(model_time[istart_sense:iend_sense],model_data[istart_sense:iend_sense,10],label='P')
plti.plot(model_time[istart_sense:iend_sense],model_data[istart_sense:iend_sense,11],label='Q')
plti.plot(model_time[istart_sense:iend_sense],model_data[istart_sense:iend_sense,12],label='R')
plti.grid()
plti.legend()
plti.set_xlabel('Time (sec)')
plti.set_ylabel('Angular Velocity (rad/s)')
pp.savefig()

STICK_MAX = 2016.
STICK_MID = 1500.
STICK_MIN = 992.
dOmega_max = 10.
dPWM = (STICK_MAX-STICK_MIN)
IpwmC = (dOmega_max/dPWM)
try:
    control_signals = model_data[istart_model:iend_model,35:38]
except:
    control_signals = model_data[istart_model:iend_model,35:37]
moments = (control_signals - STICK_MID)*IpwmC
fig = plt.figure()    
plti = fig.add_subplot(1,1,1)
#plti.plot(sense_time[istart_sense:iend_sense],control_signals)
axis = ['L','M','N']
try:
    for i in range(0,3):
        plti.plot(model_time[istart_model:iend_model],moments[:,i],label=axis[i])
except:
    pass
plti.grid()
plti.legend()
plti.set_xlabel('Time (sec)')
#plti.set_ylabel('Control Signals (us)')
plti.set_ylabel('Moments (N-m)')
pp.savefig()

#Close file
datafile.close()
#Close PDF
pp.close()
