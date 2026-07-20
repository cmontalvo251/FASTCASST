#!/usr/bin/python3
"""Standalone plotting script for the Python SIMONLY driver (src/fast_simonly.py).

Adapted from plot_simulation_data.py's CSV-reading/plotting body (column
indices carry over unchanged -- data/0.csv and logs/0.csv have the same
column layout as the C++ side). Unlike plot_simulation_data.py this does
NOT depend on the external pdf.py/sixdof.py (Python.git, not vendored in
this repo) -- it uses matplotlib's built-in PdfPages directly, and does not
compile/run any C++ executable (that scaffolding doesn't apply here).

Usage: python3 plot_simonly.py [data_csv] [logs_csv]
Defaults to the most recently written CSV in data/ and logs/.
"""
import sys
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


def latest_csv(directory):
    files = glob.glob(os.path.join(directory, '*.csv'))
    if not files:
        sys.exit('No CSV files found in %s' % directory)
    return max(files, key=os.path.getmtime)


data_path = sys.argv[1] if len(sys.argv) > 1 else latest_csv('data')
logs_path = sys.argv[2] if len(sys.argv) > 2 else latest_csv('logs')
print('Sensed (data) file:', data_path)
print('Truth  (logs) file:', logs_path)

datafile = open(data_path, 'r')
logfile = open(logs_path, 'r')
dataheaders = datafile.readline().split(',')
logheaders = logfile.readline().split(',')
numVars = len(logheaders)
print('Number of Vars =', numVars)

sense_data = [ [float(x) for x in line.split(',')] for line in datafile if len(line.split(',')) > 1 ]
model_data = [ [float(x) for x in line.split(',')] for line in logfile if len(line.split(',')) > 1 ]
sense_data = np.array(sense_data)
model_data = np.array(model_data)

sense_time = sense_data[:, 0]
model_time = model_data[:, 0]

pdf = PdfPages('plot_simonly.pdf')

for x in range(1, numVars):
    fig = plt.figure()
    plti = fig.add_subplot(1, 1, 1)
    plti.plot(sense_time, sense_data[:, x], 'b', label=dataheaders[x])
    plti.plot(model_time, model_data[:, x], 'y', label=logheaders[x])
    plti.set_xlabel('Time (sec)')
    plti.set_ylabel(dataheaders[x])
    plti.grid()
    plti.legend()
    plti.get_yaxis().get_major_formatter().set_useOffset(False)
    plti.get_xaxis().get_major_formatter().set_useOffset(False)
    plt.gcf().subplots_adjust(left=0.18)
    pdf.savefig(fig)
    plt.close(fig)

##XY trajectory
fig = plt.figure()
plti = fig.add_subplot(1, 1, 1)
plti.plot(sense_data[:, 1], sense_data[:, 2], 'b', label='Sense')
plti.plot(model_data[:, 1], model_data[:, 2], 'y', label='Model')
plti.set_xlabel('X (m)')
plti.set_ylabel('Y (m)')
plti.grid()
plti.legend()
plt.gcf().subplots_adjust(left=0.18)
pdf.savefig(fig)
plt.close(fig)

##Lat/Lon trajectory (columns 16/17, matching the C++ column contract)
fig = plt.figure()
plti = fig.add_subplot(1, 1, 1)
plti.plot(sense_data[:, 16], sense_data[:, 17], 'b', label='Sense')
plti.plot(model_data[:, 16], model_data[:, 17], 'y', label='Model')
plti.set_xlabel('Latitude (deg)')
plti.set_ylabel('Longitude (deg)')
plti.grid()
plti.legend()
plt.gcf().subplots_adjust(left=0.18)
pdf.savefig(fig)
plt.close(fig)

pdf.close()
print('Wrote plot_simonly.pdf')
