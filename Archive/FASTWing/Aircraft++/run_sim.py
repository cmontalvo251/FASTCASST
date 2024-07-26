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

def create_state_plots(datastate,pp):
	#create sense hat plots
	time = datastate[:,0]
	x = datastate[:,1]
	y = datastate[:,2]
	z = datastate[:,3]
	phi = datastate[:,4] * (180/np.pi)
	theta = datastate[:,5] * (180/np.pi)
	psi = datastate[:,6] * (180/np.pi)
	u =  datastate[:,7]
	v = datastate[:,8]
	w = datastate[:,9]
	p = datastate[:,10]
	q = datastate[:,11]
	r = datastate[:,12]
	de = datastate[:,13]* (180/np.pi)
	da = datastate[:,14]* (180/np.pi)
	dr = datastate[:,15]* (180/np.pi)
	mewt = datastate[:,16]
	T = datastate[:,17]
	
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

	aileron_signal_sim = da*((SERVO_MAX-SERVO_MIN)/(2.*MAX_ANGLE_AILERON))+SERVO_MID
	elevator_signal_sim = de*((SERVO_MAX-SERVO_MIN)/(2.*MAX_ANGLE_ELEVATOR))+SERVO_MID
	rudder_signal_sim = dr*((RUDDER_SERVO_MAX-RUDDER_SERVO_MIN)/(2.*MAX_ANGLE_RUDDER))+RUDDER_SERVO_MID

	#plot x
	plt.figure()
	plt.plot(time,x,label = 'x')
	plt.xlabel('Time (s)')
	plt.ylabel('x (m)')
	plt.grid()
	plt.legend()
	plt.title('x v. t')
	pp.savefig()
	
	#plot y

	plt.figure()
	plt.plot(time,y,label = 'y')
	plt.xlabel('Time (s)')
	plt.ylabel('y (m)')
	plt.grid()
	plt.legend()
	plt.title('y vs. t')
	pp.savefig()
	#plot z
	plt.figure()
	plt.plot(time,z,label = 'z')
	plt.xlabel('Time (s)')
	plt.ylabel('z (m)')
	plt.grid()
	plt.legend()
	plt.title('z vs. t')
	pp.savefig()

	#plot phi
	plt.figure()
	plt.plot(time,phi,label = '$\\phi$')
	plt.xlabel('Time (s)')
	plt.ylabel('$\\phi$ (Degrees)') #this way of adding greek letters works!!
	plt.grid()
	plt.legend()
	plt.title('$\\phi$ v. t')
	pp.savefig()

	#plot theta
	plt.figure()
	plt.plot(time,theta,label = '$\\theta$')
	plt.xlabel('Time (s)')
	plt.ylabel('$\\theta$ (Degrees)')
	plt.grid()
	plt.legend()
	plt.title('$\\theta$ v. t')
	pp.savefig()

	#plot psi
	plt.figure()
	plt.plot(time,psi,label = '$\\psi$')
	plt.xlabel('Time (s)')
	plt.ylabel('$\\psi$ (Degrees)')
	plt.grid()
	plt.legend()
	plt.title('$\\psi$ v. t')
	pp.savefig()

	#plot u
	plt.figure()
	plt.plot(time,u,label = 'u')
	plt.xlabel('Time (s)')
	plt.ylabel('u (m/s)')
	plt.grid()
	plt.legend()
	plt.title('u v. t')
	pp.savefig()

	plt.figure()
	plt.plot(time,v,label = 'v')
	plt.xlabel('Time (s)')
	plt.ylabel('v (m/s)')
	plt.grid()
	plt.legend()
	plt.title('v v. t')
	pp.savefig()

	plt.figure()
	plt.plot(time,w,label = 'w')
	plt.xlabel('Time (s)')
	plt.ylabel('w (m/s)')
	plt.grid()
	plt.legend()
	plt.title('w v. t')
	pp.savefig()

		#plot phi
	plt.figure()
	plt.plot(time,p,label = 'p')
	plt.xlabel('Time (s)')
	plt.ylabel('p (rad/s)')
	plt.grid()
	plt.legend()
	plt.title('p v. t')
	pp.savefig()

	#plot phi
	plt.figure()
	plt.plot(time,q,label = 'q')
	plt.xlabel('Time (s)')
	plt.ylabel('q (rad/s)')
	plt.grid()
	plt.legend()
	plt.title('q v. t')
	pp.savefig()

	#plot phi
	plt.figure()
	plt.plot(time,r,label = 'r')
	plt.xlabel('Time (s)')
	plt.ylabel('r (rad/s)')
	plt.grid()
	plt.legend()
	plt.title('r v. t')
	pp.savefig()
	
	plt.figure()
	plt.plot(time,de)
	plt.xlabel('Time (s)')
	plt.ylabel('Elevator (deg)')
	plt.grid()
	plt.title('Elevator v. t')
	pp.savefig()


	plt.figure()
	plt.plot(time,elevator_signal_sim,label='Sim')
	plt.grid()
	plt.xlabel('Time (sec)')
	plt.ylabel('Elevator Signal (us)')
	pp.savefig()
	
	plt.figure()
	plt.plot(time,da)
	plt.xlabel('Time (s)')
	plt.ylabel('Aileron (deg)')
	plt.grid()
	plt.title('Aileron v. t')
	pp.savefig()
	
	plt.figure()
	plt.plot(time,aileron_signal_sim,label='Sim')
	plt.grid()
	plt.xlabel('Time (sec)')
	plt.ylabel('Aileron Signal (us)')
	pp.savefig()
	
	plt.figure()
	plt.plot(time,dr)
	plt.xlabel('Time (s)')
	plt.ylabel('Rudder (deg)')
	plt.grid()
	plt.title('Rudder v. t')
	pp.savefig()
	
	plt.figure()
	plt.plot(time,rudder_signal_sim,label='Sim')
	plt.grid()
	plt.xlabel('Time (sec)')
	plt.ylabel('Rudder Signal (us)')
	pp.savefig()
	
	plt.figure()
	plt.plot(time,mewt)
	plt.xlabel('Time (s)')
	plt.ylabel('Motor ESC Signal (us)')
	plt.grid()
	plt.title('Motor ESC v. t')
	pp.savefig()
	
	plt.figure()
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

    ##Grab Initial Conditions
    x0 = 0.0
    y0 = 0.0
    z0 = -100.0
    roll0 = 0.0
    pitch0 = 0*np.pi/180.
    yaw0 = 0
    u0 = 10.0
    v0 = 0.0
    w0 = 0.0
    ## v = w x r
    ## in 2D -> v = w*r
    ## w = v/r
    p0 = 1.0/0.743
    q0 = 0
    r0 = 0
    state0 = [x0,y0,z0,roll0,pitch0,yaw0,u0,v0,w0,p0,q0,r0]
    ##Output Initial Conditions File
    fid = open('Input_Files/A++.IC','w')
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
    for i in range(0,len(state0)):
        string = str(state0[i]) + '\t' + text[i] + '\n'
        fid.write(string)
    fid.close()
    ###Output Simulation File
    fid = open('Input_Files/A++.SIM','w')
    tstate = [t0,tf,0.01,1] ##The zero means the control system is off but if it's a three then it's constant control inputs
    ttext = ['!Initial Time (sec) ','!Final Time (sec)','!Timestep (sec)','!Use Experimental Input Files']
    for i in range(0,len(tstate)):
        string = str(tstate[i]) + '\t' + ttext[i] + '\n'
        fid.write(string)
    fid.close()

    os.system('make')
    os.system('./A++.exe')

    print('Processing State output File')

    inputfilename = 'Output_Files/C++.OUT'

    SHOWPLOTS = 0 #Set to 1 at your own peril. It will output a ton of windows. Better to just leave at 0
    pp = PDF(SHOWPLOTS,plt)

    data_all = []
    #inputfilename = inputfilenames[0]
    data = get_state_data(inputfilename)
    #data_all.append(data)
    
    #datastate = data_all
    #print len(datastate[0])
    
    create_state_plots(data,pp)
    
    pp.close()
