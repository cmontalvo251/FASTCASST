#!/usr/bin/python

import numpy as np 
import matplotlib.pyplot as pyplot
import sys
from pdf import *
import mymath as M 
import os
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib import cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import mpl_toolkits.mplot3d as a3
import matplotlib.patches as patch
import copy as CPY

def plottool3(title,x,y,z,xlabel,ylabel,zlabel):
    fig = plt.figure('3-D')
    ax = fig.add_subplot(111,projection='3d')
    #ax.plot_wireframe(x,y,z, color = 'blue', linestyle = 'solid')
    ax.plot(x,y,z)
    plt.title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_zlabel(zlabel)
    return ax #just in case they want access to the ax variable


def get_state_data(inputfilename):
    print('Loading File...')
    statedata = open(inputfilename)
    time = []
    x = []
    y = []
    z = []
    phi = []
    theta = []
    psi = []
    u = []
    v = []
    w = []
    p = []
    q = []
    r = []
    de = []
    da = []
    dr = []
    mewt = []
    T = []
    lenfile = 0
    datastate = []

    for line in statedata:
	lenfile+=1
        if len(line) > 2:
	    row = line.split(' ')
        else:
            print('whoops')

	if len(row) > 1:
            #print row
            row_np = []
            for x in row:
                try:
                    val = np.float(x)
                except:
                    pass
                row_np.append(val)
            datastate.append(np.asarray(row_np))

    print('datastate made')
    return np.asarray(datastate)

def create_state_plots(data,pp):
    timeall = []
    xall = []
    yall = []
    zall = []
    phiall = []
    thetaall = []
    psiall = []
    uall = []
    vall = []
    wall = []
    pall = []
    qall = []
    rall = []
    deall = []
    daall = []
    drall = []
    mewtall = []
    Tall = []
    aileron_signal_sim_all = []
    elevator_signal_sim_all = []
    rudder_signal_sim_all = []

    for datastate in data:
        #timeall.append(datastate[:,0])
        time = datastate[:,0] ##assume time is the same for all
        xall.append(datastate[:,1])
        yall.append(datastate[:,2])
        zall.append(datastate[:,3])
        phiall.append(datastate[:,4] * (180/np.pi))
        thetaall.append(datastate[:,5] * (180/np.pi))
        psiall.append(datastate[:,6] * (180/np.pi))
        uall.append(datastate[:,7])
        vall.append(datastate[:,8])
        wall.append(datastate[:,9])
        pall.append(datastate[:,10])
        qall.append(datastate[:,11])
        rall.append(datastate[:,12])
        de = datastate[:,13]* (180/np.pi)
        deall.append(de)
        da = datastate[:,14]* (180/np.pi)
        daall.append(da)
        dr = datastate[:,15]* (180/np.pi)
        drall.append(dr)
        mewtall.append(datastate[:,16])
        Tall.append(datastate[:,17])
        
        #####CONVERT COMMANDED ANGLES TO SERVO SIGNALS#####
        ###Taken From TopGun.cpp 7/15/2019
        #SERVO_MAX = 1900.0 ##Defined in FASTPWM.h
        #SERVO_MID = 1500.0 ##Defined in FASTPWM.h
        #MAX_ANGLE = 30.0 ##Defined in Controller.h 
        #aileron_signal_sim = (da*((SERVO_MAX-SERVO_MID)/(MAX_ANGLE)))+SERVO_MID ##//Convert from radians to microsecond pulse
        #elevator_signal_sim = (de*((SERVO_MAX-SERVO_MID)/(MAX_ANGLE)))+SERVO_MID
        #rudder_signal_sim = (dr*((SERVO_MAX-SERVO_MID)/(MAX_ANGLE)))+SERVO_MID
        #^^^Notice that the code above uses (max-mid)/max_angle
        #vvvBut the code below uses (max-min)/(2*max_angle)
        #These two conversion are equivalent provided the min,mid,max values are all set
        #correctly. For now just use the one below with 2*max_angle since that is the
        #same as the C++ version.

        #Update 7/17/2019
        SERVO_MIN = 992.
        SERVO_MAX = 2016.
        THROTTLE_MIN = 992.
        THROTTLE_MAX = 2016.
        SERVO_MID  = 1504.
        RUDDER_SERVO_MIN = 985.
        RUDDER_SERVO_MAX = 2009.
        RUDDER_SERVO_MID = 1497.
        #Reading Angles I learned this --- all in degrees
        MAX_ANGLE_ELEVATOR = 20.
        MAX_ANGLE_AILERON = 23.
        MAX_ANGLE_RUDDER = 25.

        aileron_signal_sim_all.append(da*((SERVO_MAX-SERVO_MIN)/(2.*MAX_ANGLE_AILERON))+SERVO_MID)
        elevator_signal_sim_all.append(de*((SERVO_MAX-SERVO_MIN)/(2.*MAX_ANGLE_ELEVATOR))+SERVO_MID)
        rudder_signal_sim_all.append(dr*((RUDDER_SERVO_MAX-RUDDER_SERVO_MIN)/(2.*MAX_ANGLE_RUDDER))+RUDDER_SERVO_MID)

    #plot x
    plt.figure()
    for x in xall:
        plt.plot(time,x)
    plt.xlabel('Time (s)')
    plt.ylabel('x (m)')
    plt.grid()
    #plt.legend()
    plt.title('x v. t')
    pp.savefig()
    
    #plot y

    plt.figure()
    for y in yall:
        plt.plot(time,y)
    plt.xlabel('Time (s)')
    plt.ylabel('y (m)')
    plt.grid()
    #plt.legend()
    plt.title('y vs. t')
    pp.savefig()
    
    #plot z
    plt.figure()
    for z in zall:
        plt.plot(time,z)
    plt.xlabel('Time (s)')
    plt.ylabel('z (m)')
    plt.grid()
    #plt.legend()
    plt.title('z vs. t')
    pp.savefig()

    #plot phi
    plt.figure()
    for phi in phiall:
        plt.plot(time,phi)
    plt.xlabel('Time (s)')
    plt.ylabel('$\\phi$ (Degrees)') #this way of adding greek letters works!!
    plt.grid()
    #plt.legend()
    plt.title('$\\phi$ v. t')
    pp.savefig()

    #plot theta
    plt.figure()
    for theta in thetaall:
        plt.plot(time,theta)
    plt.xlabel('Time (s)')
    plt.ylabel('$\\theta$ (Degrees)')
    plt.grid()
    #plt.legend()
    plt.title('$\\theta$ v. t')
    pp.savefig()

    #plot psi
    plt.figure()
    for psi in psiall:
        plt.plot(time,psi)
    plt.xlabel('Time (s)')
    plt.ylabel('$\\psi$ (Degrees)')
    plt.grid()
    #plt.legend()
    plt.title('$\\psi$ v. t')
    pp.savefig()

    #plot u
    plt.figure()
    for u in uall:
        plt.plot(time,u)
    plt.xlabel('Time (s)')
    plt.ylabel('u (m/s)')
    plt.grid()
    #plt.legend()
    plt.title('u v. t')
    pp.savefig()

    plt.figure()
    for v in vall:
        plt.plot(time,v)
    plt.xlabel('Time (s)')
    plt.ylabel('v (m/s)')
    plt.grid()
    #plt.legend()
    plt.title('v v. t')
    pp.savefig()

    plt.figure()
    for w in wall:
        plt.plot(time,w)
    plt.xlabel('Time (s)')
    plt.ylabel('w (m/s)')
    plt.grid()
    #plt.legend()
    plt.title('w v. t')
    pp.savefig()

    #plot p
    plt.figure()
    for p in pall:
        plt.plot(time,p)
    plt.xlabel('Time (s)')
    plt.ylabel('p (rad/s)')
    plt.grid()
    #plt.legend()
    plt.title('p v. t')
    pp.savefig()

    #plot q
    plt.figure()
    for q in qall:
        plt.plot(time,q)
    plt.xlabel('Time (s)')
    plt.ylabel('q (rad/s)')
    plt.grid()
    #plt.legend()
    plt.title('q v. t')
    pp.savefig()

    #plot r
    plt.figure()
    for r in rall:
        plt.plot(time,r)
    plt.xlabel('Time (s)')
    plt.ylabel('r (rad/s)')
    plt.grid()
    #plt.legend()
    plt.title('r v. t')
    pp.savefig()
    
    plt.figure()
    for de in deall:
        plt.plot(time,de)
    plt.xlabel('Time (s)')
    plt.ylabel('Elevator (deg)')
    plt.grid()
    plt.title('Elevator v. t')
    pp.savefig()


    plt.figure()
    for elevator_signal_sim in elevator_signal_sim_all:
        plt.plot(time,elevator_signal_sim)
    plt.grid()
    plt.xlabel('Time (sec)')
    plt.ylabel('Elevator Signal (us)')
    pp.savefig()
    
    plt.figure()
    for da in daall:
        plt.plot(time,da)
    plt.xlabel('Time (s)')
    plt.ylabel('Aileron (deg)')
    plt.grid()
    plt.title('Aileron v. t')
    pp.savefig()
    
    plt.figure()
    for aileron_signal_sim in aileron_signal_sim_all:
        plt.plot(time,aileron_signal_sim)
    plt.grid()
    plt.xlabel('Time (sec)')
    plt.ylabel('Aileron Signal (us)')
    pp.savefig()
    
    plt.figure()
    for dr in drall:
        plt.plot(time,dr)
    plt.xlabel('Time (s)')
    plt.ylabel('Rudder (deg)')
    plt.grid()
    plt.title('Rudder v. t')
    pp.savefig()
    
    plt.figure()
    for rudder_signal_sim in rudder_signal_sim_all:
        plt.plot(time,rudder_signal_sim)
    plt.grid()
    plt.xlabel('Time (sec)')
    plt.ylabel('Rudder Signal (us)')
    pp.savefig()
    
    plt.figure()
    for mewt in mewtall:
        plt.plot(time,mewt)
    plt.xlabel('Time (s)')
    plt.ylabel('Motor ESC Signal (us)')
    plt.grid()
    plt.title('Motor ESC v. t')
    pp.savefig()
    
    plt.figure()
    for T in Tall:
        plt.plot(time,T)
    plt.xlabel('Time (s)')
    plt.ylabel('Thrust (N)')
    plt.grid()
    plt.title('Thrust v. t')
    pp.savefig()

if __name__ == '__main__':

    #################################INJECT INPUTS INTO SIMULATION#################

    t0 = 0
    tf = 25
    timestep = 0.0001
    controltype = 1 #0 = off, 1 = PID, 2 = interpolate

    text = ['!Initial X Position (m)',
            '!Initial Y Position (m)',
            '!Initial Z Position (m)',
            '!Initial Roll Angle (deg)',
            '!Initial Pitch Angle (deg)',
            '!Initial Yaw Angle (deg)',
            '!Initial Xbody Velocity (m/s)',
            '!Initial Ybody Velocity (m/s)',
            '!Initial Zbody Velocity (m/s)',
            '!Initial Roll Rate (rad/s)',
            '!Initial Pitch Rate (rad/s)',
            '!Initial Yaw Rate (rad/s)']
    
    ##Grab Initial Conditions of First Aircraft
    x0 = 0.0
    y0 = 0.0
    z0 = -100.0
    roll0 = 0.0
    pitch0 = 0*np.pi/180.
    yaw0 = 0
    u0 = 10.0
    v0 = 0.0
    w0 = -1.0
    p0 = 0.0
    q0 = 0
    r0 = 0
    state0 = [x0,y0,z0,roll0,pitch0,yaw0,u0,v0,w0,p0,q0,r0]

    ###Initial Conditions of Second Aircraft
    x0 = 0.0
    y0 = 0.743*2.0
    z0 = -100.0
    roll0 = 0.0
    pitch0 = 0*np.pi/180.
    yaw0 = 0
    u0 = 10.0
    v0 = 0.0
    w0 = 1.0
    p0 = 0
    q0 = 0
    r0 = 0
    state1 = [x0,y0,z0,roll0,pitch0,yaw0,u0,v0,w0,p0,q0,r0]

    ##Output Initial Conditions File to 1st Aircraft
    fid = open('Input_Files/Aircraft1/A++.IC','w')
    for i in range(0,len(state0)):
        string = str(state0[i]) + '\t' + text[i] + '\n'
        fid.write(string)
    fid.close()

    ##Output Initial Conditions File to 2nd Aircraft
    fid = open('Input_Files/Aircraft2/A++.IC','w')
    for i in range(0,len(state1)):
        string = str(state1[i]) + '\t' + text[i] + '\n'
        fid.write(string)
    fid.close()
    
    ###Output Simulation File (Default is first aircraft
    ttext = ['!Initial Time (sec) ','!Final Time (sec)','!Timestep (sec)','!Use Experimental Input Files']
    fid = open('Input_Files/Aircraft1/A++.SIM','w')
    tstate = [t0,tf,timestep,controltype] ##The zero means the control system is off but if it's a three then it's constant control inputs
    for i in range(0,len(tstate)):
        string = str(tstate[i]) + '\t' + ttext[i] + '\n'
        fid.write(string)
    fid.close()
    #But control system variable is in both files
    fid = open('Input_Files/Aircraft2/A++.SIM','w')
    tstate = [t0,tf,timestep,controltype] ##The zero means the control system is off but if it's a three then it's constant control inputs
    for i in range(0,len(tstate)):
        string = str(tstate[i]) + '\t' + ttext[i] + '\n'
        fid.write(string)
    fid.close()


    os.system('make')
    os.system('./A++.exe Input_Files/Aircraft1/A++.files Input_Files/Aircraft2/A++.files')

    #sys.exit()

    print('Processing State output File')


    SHOWPLOTS = 0 #Set to 1 at your own peril. It will output a ton of windows. Better to just leave at 0
    pp = PDF(SHOWPLOTS,plt)

    data_all = []
    #inputfilename = inputfilenames[0]
    outputfilenames = ['Output_Files/C1++.OUT','Output_Files/C2++.OUT']
    for out in outputfilenames:
        data = get_state_data(out)
        data_all.append(data)
    
    create_state_plots(data_all,pp)

    pp.close()
