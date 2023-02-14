#!/usr/bin/python3

import matplotlib.pyplot as plt
plt.rcParams.update({'figure.max_open_warning': 0})
plt.rcParams.update({'font.size': 14})
import sys
import os
import numpy as np

try:
    from pdf import *
    #import sixdof as dof
except:
    print('You need pdf and sixdof from Python.git This is on my Github just git clone that repo and put pdf.py and sixdof.py in this root or add to pythonpath')
    sys.exit()

pp = PDF(0,plt)

#Clean Logs
os.system('./clean_logs')

#Loop Through CubeSAT Configurations
configs = ['1U']#,'2U_Upright','2U_Sideways','6U']
controlname = ['No_Control','PID','FL','TS']
for config in configs:
    print('CONFIG = ',config)
    #Get Config File Name
    configfile = 'vehicles/cubesat/Input_Files/Config_'+config+'.txt'
    #Loop through ranks
    for rank in range(2,4):
        print('RANK = ',rank)
        #Change the Simulation File
        os.system('cp vehicles/cubesat/Input_Files/Simulation'+str(rank)+'.txt vehicles/cubesat/Input_Files/Simulation.txt')
        #Change the Params.h file
        os.system('cp vehicles/cubesat/src/params'+str(rank)+'.h vehicles/cubesat/src/params.h')
        ##Compile Software
        os.system('make clean')
        os.system('make simonly MODEL=cubesat')
        #Simulate the No Control Case
        if rank == 2:
            print('NO CONTROL')
            command = "sed 's/3 !C/0 !C/g' "+configfile+">vehicles/cubesat/Input_Files/Config.txt"
            #print(command)
            os.system(command)
            os.system('cat vehicles/cubesat/Input_Files/Config.txt')
            #Run Software
            os.system('./simonly.exe cubesat/')
            os.system('mv logs/0.csv logs/CubeSAT_No_Control')
        #Loop through Control Systems
        for controlnum in range(1,4):
            print("CONTROL SYSTEM=",controlname[controlnum])
            command = "sed 's/3 !C/"+str(controlnum)+" !C/g' "+configfile+">vehicles/cubesat/Input_Files/Config.txt"
            #print(command)
            os.system(command)
            os.system('cat vehicles/cubesat/Input_Files/Config.txt')
            #Run Software
            os.system('./simonly.exe cubesat/')
            #Copy Log file
            os.system('mv logs/0.csv logs/CubeSAT_'+controlname[controlnum]+str(rank))

    #Loop through logs
    ##TRUNCATION START AND END TIME (Set to negative to turn off)
    tstart = -99
    tend = 10.0

    ##Create plot handles
    figPQR = plt.figure()
    pltiPQR = figPQR.add_subplot(1,1,1)
    pltiPQR.grid()
    pltiPQR.set_xlabel('Time (sec)')
    pltiPQR.set_ylabel('Norm of Angular Velocity (rad/s)')
    pltiPQR.set_title("CubeSat = "+str(config))
    plt.gcf().subplots_adjust(left=0.12)

    figLMN = plt.figure()    
    pltiLMN = figLMN.add_subplot(1,1,1)
    pltiLMN.grid()
    pltiLMN.set_xlabel('Time (sec)')
    pltiLMN.set_ylabel('Norm of Moments (N-m)')
    pltiLMN.set_title("CubeSat = "+str(config))
    plt.gcf().subplots_adjust(left=0.18)

    STICK_MAX = 2016.
    STICK_MID = 1500.
    STICK_MIN = 992.
    dOmega_max = 10.
    dPWM = (STICK_MAX-STICK_MIN)
    IpwmC = (dOmega_max/dPWM)

    legends = ['No Control','PID r(M)=3','FL r(M)=3','Two Stage r(M)=3','PID r(M)=2','FL r(M)=2','Two Stage r(M)=2']
    files = ['CubeSAT_No_Control','CubeSAT_PID3','CubeSAT_FL3','CubeSAT_TS3','CubeSAT_PID2','CubeSAT_FL2','CubeSAT_TS2']
    counter = -1
    x = 0
    linestyle = '-'
    for filenamei in files:
        counter+=1
        if x == 1:
            x = 0 
        else:
            x = 1
        if linestyle == '--':
            linestyle = '-'
        else:
            linestyle = '--'
        #Open File
        print(filenamei)
        logfile = open('logs/'+filenamei,'r')
        logheaders = logfile.readline().split(',')
        numVars = len(logheaders)
        print('Number of Vars = ',numVars)
        print(logheaders)
        #Grab entire data file
        model_data = []
        for line in logfile:
            row = line.split(',')
            numarray = [np.float(x) for x in row]
            model_data.append(numarray)
        model_data = np.array(model_data)
        #Plot everything
        model_time = model_data[:,0]
        if tstart > 0:
            istart_model = np.where(model_time>tstart)[0][0]
        else:
            istart_model = 0
        if tend > 0:
            iend_model = np.where(model_time>tend)[0][0]
        else:
            iend_model = -1

        ###JUST FOR CUBESAT PLOT MOMENTS AND PQR
        p = model_data[istart_model:iend_model,10]
        q = model_data[istart_model:iend_model,11]
        r = model_data[istart_model:iend_model,12]
        #pltiPQR.plot(model_time[istart_model:iend_model],p,label='P')
        #pltiPQR.plot(model_time[istart_model:iend_model],q,label='Q')
        #pltiPQR.plot(model_time[istart_model:iend_model],r,label='R')
        pltiPQR.plot(model_time[istart_model:iend_model],np.sqrt(p**2+q**2+r**2),linestyle,linewidth=2+x,label=legends[counter])

        try:
            control_signals = model_data[istart_model:iend_model,35:38]
        except:
            control_signals = model_data[istart_model:iend_model,35:37]

        moments = (control_signals - STICK_MID)*IpwmC
        axis = ['L','M','N']
        lmn = moments[:,0]**2
        try:
            for i in range(1,3):
                lmn += moments[:,i]**2
                #pltiLMN.plot(model_time[istart_model:iend_model],moments[:,i],label=axis[i])
        except:
            pass
        lmn = np.sqrt(lmn)

        pltiLMN.plot(model_time[istart_model:iend_model],lmn,linestyle,linewidth=2+x,label=legends[counter])

        #Close file
        logfile.close()

    ##LEGENDS and Save figure
    #pltiPQR.legend(loc='best')
    plt.figure(figPQR.number)
    pp.savefig()
    #pltiLMN.legend(loc='best')
    plt.figure(figLMN.number)
    pp.savefig()

    #Show plots
    #plt.show()
pp.close()