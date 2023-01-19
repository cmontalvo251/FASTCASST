#!/usr/bin/python

import matplotlib.pyplot as plt
import numpy as np
import mio as M
from pdf import *
import mymath as MYM
import sys

plt.close("all")

#data = M.dlmread('Data_Files/March_11_2019_8_Motors_Pusher.txt',' ')
data = M.dlmread('Data_Files/output28.txt',' ')
#data = M.dlmread('Data_Files/March_11_2019_8_Motors_Pusher_Tuned_Gains.txt',' ')
columns = ['Time (sec)','Throttle Input (us)','Roll Input (us)','Pitch Input (us)','Yaw input (us)','Roll (deg)','Pitch (deg)','Yaw (deg)','Roll Rate (deg/s)','Pitch Rate (deg/s)','Yaw Rate (deg/s)','Motor1','Motor2','Motor3','Motor4','Motor5','Motor6','Motor7','Motor8','Motor9']

pp = PDF(0,plt)

ctr = 0
time = data[:,0]/1000000.0
throttle = data[:,1]
#loc = np.where(throttle > 1050)[0][0]
loc = 0
#print loc
#sys.exit()
for c in columns:
    plt.figure()
    plt.plot(time[loc:]-time[loc],data[loc:,ctr],label='Raw')
    [outy,outx] = MYM.LowPass(data[:,ctr],time,3.0)
    plt.plot(time[loc:]-time[loc],outy,'r-',label='Filtered')
    #plt.xlim([190,265])
    ctr+=1
    plt.xlabel('Time (sec)')
    plt.ylabel(c)
    plt.title(c)
    plt.legend()
    plt.grid()
    pp.savefig()

pp.close()

    
