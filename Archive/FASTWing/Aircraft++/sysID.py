#!/usr/bin/python
import sys
import numpy as np
import plotting as P
from pdf import *
import matplotlib.pyplot as plt
import time
import mymath as MYM
import os
from copy import copy

def writeToFile(filename,data,txt):
	fid = open(filename,'w')
	for i in range(0,len(data)):
		string = str(data[i]) + '\t' + txt[i] + '\n'
		fid.write(string)
	fid.close()

def printCurrentVars(coeffsIN,vardx):
        varsOUT = [coeffsIN[i] for i in vardx]
        print(varsOUT)
        return varsOUT

def computeCost(coeffsIN,textIN):
        writeToFile('Input_Files/A++.AERO',coeffsIN,textIN)
        os.system('./A++.exe')
        cost = np.loadtxt('Output_Files/A++.COST')
        return cost

##Setup plotting
pp = PDF(0,plt)

###########################EXTRACT DATA FROM FILE#####################################

########################The data in this flight is using the following column header#####
#0 - 'Time'
#1 - ,'Roll (deg)'
#2 - ,'Pitch (deg)'
#3 - ,'Yaw (deg)'
#4 - ,'Roll Rate P (deg/s)'
#5 - ,'Pitch Rate, Q (deg/s)'
#6 - ,'Yaw Rate, R (deg/s)'
#7 - ,'Rollrc (usec)'
#8 - ,'Pitchrc (usec)'
#9 - ,'Yawrc (usec)'
#10 - ,'Throttle (usec)'
#11 - 'Motor (usec)'
#12 - ,'Aileron (usec)'
#13 - ,'Elevator (usec)'
#14 - ,'Rudder (usec)'
#15 - ,'Latitude (deg)'
#16 - ,'Longitude (deg)'
#17 - ,'Altitude (m)'
#18 - ,'Pitot Tube (Volts)'
#19 - ,'Pressure (mbar)']
data = np.loadtxt('../Data_Files/Irvington/6_14_2019/SysID_Flight_Jun_14_2019.txt')
[r,c] = np.shape(data)
print('Data is ',r,' by ',c)
#Fixes time to seconds
time = data[:,0]
time -= time[0]
time /= 1000000.0
roll = data[:,1]
pitch = data[:,2]
yaw = data[:,3]
yaw[yaw<0]+=360.0
roll_rate = data[:,4]
pitch_rate = data[:,5]
yaw_rate = data[:,6]
aileron = data[:,7] ##microsecond
elevator = data[:,8] ##microsecond
rudder = data[:,9] ###microsecond
motor_signal = data[:,10]

#################################INJECT INPUTS INTO SIMULATION#################

##Full Data Set
t0 = time[0]
tf = time[-1]

##Pitch Oscillations
#t0 = 40
#tf = 55

#Roll Oscillations
#t0 = 75.0
#tf = 80.0

#Yaw Oscillations
t0 = 25.0
tf = 40.0

##Grab the location of the initial condition
loc0 = np.where(time > t0)[0][0]
locf = np.where(time >= tf)[0][0]

##Grab Initial Conditions
x0 = 0.0
y0 = 0.0
z0 = -100.0
roll0 = roll[loc0]
pitch0 = pitch[loc0]
yaw0 = yaw[loc0]
u0 = 10.0
v0 = 0.0
w0 = 0.0
p0 = roll_rate[loc0]*np.pi/180.0
q0 = pitch_rate[loc0]*np.pi/180.0
r0 = yaw_rate[loc0]*np.pi/180.0

state0 = [x0,y0,z0,roll0,pitch0,yaw0,u0,v0,w0,p0,q0,r0]

##Output Initial Conditions File
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
writeToFile('Input_Files/A++.IC',state0,text)

###Output Simulation File
#If you don't want the iteration to run simply send this
#tstate = [t0,tf,0.01,0]
#PID
#tstate = [t0,tf,0.01,1]
#Newton Raphson
tstate = [t0,tf,0.01,2] ##The two means that the C++ routine will interpolate the file we create
ttext = ['!Initial Time (sec) ','!Final Time (sec)','!Timestep (sec)','!Use Experimental Input Files']
writeToFile('Input_Files/A++.SIM',tstate,ttext)

###Create a file with time,de,da,dr,mewt from experimental tests
locf+=1
##The last column of this file is the data set we're trying to fit. 
#So if we want to fit the pitch_rate send the pitch rate like this
#control_exp = np.transpose(np.vstack((time[loc0:locf],elevator[loc0:locf],aileron[loc0:locf],rudder[loc0:locf],motor_signal[loc0:locf],np.pi/180.0*pitch_rate[loc0:locf])))
#Otherwise for roll oscillations send roll rate
#By the way you also need to head over to Controller.cpp::Compute and change the sim state
#control_exp = np.transpose(np.vstack((time[loc0:locf],elevator[loc0:locf],aileron[loc0:locf],rudder[loc0:locf],motor_signal[loc0:locf],np.pi/180.0*roll_rate[loc0:locf])))
#Or Yaw rate
control_exp = np.transpose(np.vstack((time[loc0:locf],elevator[loc0:locf],aileron[loc0:locf],rudder[loc0:locf],motor_signal[loc0:locf],np.pi/180.0*yaw_rate[loc0:locf])))
np.savetxt('Input_Files/A++.CTL', control_exp, fmt='%.18e', delimiter=' ', newline='\n', header='', footer='', comments='# ')

###########################RUN ITERATIONS OF SOFTWARE########################
########################Multidimensional Newton Raphson######################

##Nominal Parameters
#coeffs_PRESYSID
#coeffsi = [0.3441029736813229,5.185317204810547,5.9369349224934975,0.41181051485504044,0.01727851900786801,1.0080305157535787,-0.24095762862664397,
#0.0688450367504697,-0.027733042384004744,0.17663419386614151,-0.01633611041536569,-0.6583142767075709,0.09800094300447591,0.19085572159332118,
#0.004667460118675911,-0.03632475219322319,-2.5363576941792045,-17.11816235985625,-1.0238066966535029,-0.03284800659907888,0.08831709693307076,
#-0.06906055250375191,-0.216,-0.17115641081019545,0.2286,1.4859,1.225,12.1]

##These are the coefficients after I ran the pitch oscillations routine
#coeffsi = [0.3441029736813229,5.185317204810547,5.9369349224934975,0.41181051485504044,0.01727851900786801,1.0080305157535787,-0.24095762862664397,
#0.0688450367504697,-0.027733042384004744,0.17663419386614151,-0.01633611041536569,-0.6583142767075709,0.09800094300447591,0.19085572159332118,
#0.004667460118675911,-0.07636788929236879,-2.1859941783855077,-24.450947269394277,-1.1530526838451003,-0.03284800659907888,0.08831709693307076,
#-0.06906055250375191,-0.216,-0.17115641081019545,0.2286,1.4859,1.225,12.1]

#THese are the coeffs after I ran the pitch and roll oscillations
#coeffsi = [0.3441029736813229,5.185317204810547,5.9369349224934975,0.41181051485504044,0.01727851900786801,1.0080305157535787,-0.24095762862664397,
#0.0688450367504697,-0.027733042384004744,0.17663419386614151,-0.01633611041536569,-0.50160400189302223,0.09800094300447591,0.28052396489578457,
#0.004667460118675911,-0.07636788929236879,-2.1859941783855077,-24.450947269394277,-1.1530526838451003,-0.03284800659907888,0.08831709693307076,
#-0.06906055250375191,-0.216,-0.17115641081019545,0.2286,1.4859,1.225,12.1]

#These are the coeffs after I ran the yaw oscillations
#Initial = [-0.03284800659907888, -0.06906055250375191, -0.17115641081019545, 0.08831709693307076, -0.216]
#Final = [-0.78826431744966985, -0.33411253656820367, -0.014305373822254552, 0.27846397495417952, 0.061169396698422564]
#coeffsi = [0.3441029736813229,5.185317204810547,5.9369349224934975,0.41181051485504044,0.01727851900786801,1.0080305157535787,-0.24095762862664397,
0.0688450367504697,-0.027733042384004744,0.17663419386614151,-0.01633611041536569,-0.50160400189302223,0.09800094300447591,0.28052396489578457,
0.004667460118675911,-0.07636788929236879,-2.1859941783855077,-24.450947269394277,-1.1530526838451003,-0.78826431744966985,0.27846397495417952,
-0.33411253656820367,0.061169396698422564,-0.014305373822254552,0.2286,1.4859,1.225,12.1]

aero_text = ['CLzero','CLalpha','CLq','CLdele','CDzero','CDalpha','Cybeta','Cydelr','Cyp','Cyr','CLbeta','CLp','CLr','CLdela','CLdelr','Cmzero','Cmalpha','Cmq','Cmdele','Cnp','Cnbeta','Cnr','Cndela','Cndelr','cbar','bws','rho','W']

####Alright so I ran the sysID routine for the following variables and I get good fits when I do this
##This is for the pitch oscillations
#vars_to_change = ['Cmzero','Cmalpha','Cmq','Cmdele']
#dvar = 1e-4
#alfa = 1e-2
#niters = 20
#Initial Vars = -0.03632475219322319, -2.5363576941792045, -17.11816235985625, -1.0238066966535029
#Initial Cost = 6619.307147
#Final Vars   = -0.07636788929236879, -2.1859941783855077, -24.450947269394277, -1.1530526838451003
#Final Cost = 5353.91612
#Which is pretty good.

###For the roll oscillations I ran this
#vars_to_change = ['CLp','CLdela']
#Initial Vars = -0.6583142767075709, 0.19085572159332118
#initial cost = 2572
#Final Vars = -0.50160400189302223, 0.28052396489578457
#final cost = 
#dvar = 1e-4
#alfa = 1e-2
#niters = 1 #30

#For the yaw oscillations I ran this
vars_to_change = ['Cnp','Cnr','Cndelr','Cnbeta','Cndela']
#('Initial Cost = ', array(7410.472597))
#[-0.03284800659907888, -0.06906055250375191, -0.17115641081019545, 0.08831709693307076, -0.216]
#('New Minimum = ', array(1610.373164), 'iteration = ', 33)
#[0.28768037442631966, -0.12315025379055297, -0.70647703359682701, 0.14194313386868443, -0.14969820384256141]
#('New Minimum = ', array(1156.508719), 'iteration = ', 95)
#[-0.33884482668824556, -0.24456614110846342, -0.22309592812613516, 0.22569859518847624, 0.0019327694346564575]
#('New Minimum = ', array(992.2159), 'iteration = ', 113)
#[-0.78826431744966985, -0.33411253656820367, -0.014305373822254552, 0.27846397495417952, 0.061169396698422564]
#('New Minimum = ', array(1385.83127), 'iteration = ', 175)
#[-2.932720420183891, -0.6826406679809407, -0.098116777736093799, 0.39351501865022154, 0.38269794207778396]
#('New Minimum = ', array(1140.382379), 'iteration = ', 196)
#[-2.5197850776568105, -0.87417325423501302, -0.81470680582454413, 0.51095313342148363, 0.70615579394989914]
#('New Minimum = ', array(987.872165), 'iteration = ', 208)
#[-4.6245800849406491, -1.1748106011502704, -0.64093250884752628, 5.1887248890803566, 0.21822800429450942]
#('New Minimum = ', array(978.754457), 'iteration = ', 215)
#[-2.9728680812869985, -7.6023303864334659, 2.875462638641944, 6.3320624112454391, -0.463511152236317]
dvar = 1e-4
alfa = 1e-2
niters = 1#300
##It seems that Cnp and Cndela are insensitive so perhaps run it again without those?
#vars_to_change = ['Cnr','Cndelr','Cnbeta']
#dvar = 1e-4
#alfa = 1e-2
#niters = 99
#('Minimum Cost = ', array(1069.65257))
#[-0.06906055250375191, -0.17115641081019545, 0.08831709693307076]
#[-0.68914564324462935, -0.19970501718170269, 2.6937082603159972]
#^^^This answer seems to agree somewhat with this one below so I'm going to go with this one
#('New Minimum = ', array(992.2159), 'iteration = ', 113)
#vars_to_change = ['Cnp','Cnr','Cndelr','Cnbeta','Cndela']
#Initial = [-0.03284800659907888, -0.06906055250375191, -0.17115641081019545, 0.08831709693307076, -0.216]
#Final = [-0.78826431744966985, -0.33411253656820367, -0.014305373822254552, 0.27846397495417952, 0.061169396698422564]

#dvar = 1e-4
#alfa = 1e-2
#niters = 300

###NR Parameters
#dvar = 1e-4
#alfa = 1e-2
#niters = 99
#niters = 0 ##Set to zero if you don't want the sysID routine to run

#############################################################################
#############################################################################

################################COMPILE CODE#################################
os.system('make')

##Automatically find which vars to change
var_idx = []
for var in vars_to_change:
        #indices = [i for i,x in enumerate(aero_text) if x == var][0]
        indices = aero_text.index(var)
        var_idx.append(indices)

##Ok so what is the basic NR algorithm?
#var_new = var_nominal - f(var_nominal)/fprime(var_nominal)
fprime = copy(var_idx)

##Notify User of initial status
initial_cost = computeCost(coeffsi,aero_text)
absolute_min = initial_cost
min_cost = initial_cost
print('Initial Cost = ',initial_cost)
print(vars_to_change)
var_current = printCurrentVars(coeffsi,var_idx)
minvars = var_current

i = 0
while (i < niters):
        #Alright so for the algorithm first we need to compute the nominal cost
        nominal_cost = computeCost(coeffsi,aero_text)
        #Then we need to perturb the variable forwards and backwards using central differencing
        for idx in range(0,len(var_idx)):
                #Move variable forward
                coeffsi[var_idx[idx]] += dvar
                forward_cost = computeCost(coeffsi,aero_text)
                #Move variable backwards
                coeffsi[var_idx[idx]] -= 2.0*dvar
                backwards_cost = computeCost(coeffsi,aero_text)
                #Compute fprime
                fprime[idx] = (forward_cost - backwards_cost)/(2.0*dvar)
                #Reset Variable
                coeffsi[var_idx[idx]] += dvar
        ##Compute next iteration
        del_var = nominal_cost/fprime
        var_current -= alfa*del_var
        #Then save into coeffsi
        ctr = 0
        for vdx in var_idx:
                coeffsi[vdx] = var_current[ctr]
                ctr+=1
        #Print New State
        var_current = printCurrentVars(coeffsi,var_idx)
        i+=1
        ##Check that we lowered cost
        updated_cost = computeCost(coeffsi,aero_text)
        #if updated_cost > initial_cost: ##This worked well for the pitch oscillations
        if updated_cost > min_cost: #This worked for the roll oscillations
                #This means we screwed up
                #Rest var_current
                var_current += alfa*del_var
                #Reduce alfa by a percentage
                alfa /= 2.0 #this worked well for pitch oscillations
                #alfa *= 0.8 this worked for the roll oscillations
                print('Lowering Alfa = ',alfa)
                #Update coeffsi
                ctr = 0
                for vdx in var_idx:
                        coeffsi[vdx] = var_current[ctr]
                        ctr+=1
        else:
                print('New Minimum = ',updated_cost,'iteration = ',i)
                min_cost = updated_cost
                if updated_cost < absolute_min:
                        absolute_min = updated_cost
                        minvars = var_current                

#Run one final time
#Write coeffs, run, cost
final_cost = computeCost(coeffsi,aero_text)
print('Final Cost = ',final_cost)

print('Minimum Cost = ',absolute_min)
print(minvars)

################################PULL IN DATA FROM SIMULATION###################

datastate = np.loadtxt('Output_Files/C++.OUT')
[rs,cs] = np.shape(datastate)
print('Sim Data is ',rs,' by ',cs)

time_sim = datastate[:,0]
x_sim = datastate[:,1]
y_sim = datastate[:,2]
z_sim = datastate[:,3]
phi_sim = datastate[:,4] * (180/np.pi)
theta_sim = datastate[:,5] * (180/np.pi)
psi_sim = datastate[:,6] * (180/np.pi)
u_sim =  datastate[:,7]
v_sim = datastate[:,8]
w_sim = datastate[:,9]
p_sim = datastate[:,10] * (180/np.pi)
q_sim = datastate[:,11] * (180/np.pi)
r_sim = datastate[:,12] * (180/np.pi)
de_sim = datastate[:,13] * (180.0/np.pi)
da_sim = datastate[:,14] * (180.0/np.pi)
dr_sim = datastate[:,15] * (180.0/np.pi)
mewt_sim = datastate[:,16]
T_sim = datastate[:,17]

#####CONVERT COMMANDED ANGLES TO SERVO SIGNALS#####
###Taken From TopGun.cpp 7/15/2019
# SERVO_MAX = 1900.0 ##Defined in FASTPWM.h
# SERVO_MID = 1500.0 ##Defined in FASTPWM.h
# MAX_ANGLE = 30.0*np.pi/180.0 ##Defined in Controller.h
# RAD2PWM
# aileron_signal_sim = (da_sim*((SERVO_MAX-SERVO_MID)/(MAX_ANGLE)))+SERVO_MID; ##//Convert from radians to microsecond pulse
# elevator_signal_sim = (de_sim*((SERVO_MAX-SERVO_MID)/(MAX_ANGLE)))+SERVO_MID;
# rudder_signal_sim = (dr_sim*((SERVO_MAX-SERVO_MID)/(MAX_ANGLE)))+SERVO_MID;
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

###RAD2PWM
aileron_signal_sim = da_sim*((SERVO_MAX-SERVO_MIN)/(2.*MAX_ANGLE_AILERON))+SERVO_MID
elevator_signal_sim = de_sim*((SERVO_MAX-SERVO_MIN)/(2.*MAX_ANGLE_ELEVATOR))+SERVO_MID
rudder_signal_sim = dr_sim*((RUDDER_SERVO_MAX-RUDDER_SERVO_MIN)/(2.*MAX_ANGLE_RUDDER))+RUDDER_SERVO_MID

#################################PLOT RESULTS####################################

plt.figure()
plt.plot(time,roll,label='Exp')
plt.plot(time_sim,phi_sim,label='Sim')
plt.grid()
plt.legend()
plt.xlabel('Time (sec)')
plt.ylabel('Roll (deg)')
plt.xlim([t0,tf])
pp.savefig()

plt.figure()
plt.plot(time,pitch,label='Exp')
plt.plot(time_sim,theta_sim,label='Sim')
plt.grid()
plt.legend()
plt.xlabel('Time (sec)')
plt.ylabel('Pitch (deg)')
plt.xlim([t0,tf])
pp.savefig()

plt.figure()
plt.plot(time,yaw,label='Exp')
plt.plot(time_sim,psi_sim,label='Sim')
plt.grid()
plt.legend()
plt.xlabel('Time (sec)')
plt.ylabel('Yaw (deg)')
plt.xlim([t0,tf])
pp.savefig()

plt.figure()
#plt.plot(time,yaw,label='Exp')
plt.plot(time_sim,u_sim,label='Sim')
plt.grid()
plt.legend()
plt.xlabel('Time (sec)')
plt.ylabel('u (m/s)')
plt.xlim([t0,tf])
pp.savefig()

plt.figure()
plt.plot(time,roll_rate,label='Exp')
plt.plot(time_sim,p_sim,label='Sim')
plt.grid()
plt.legend()
plt.xlabel('Time (sec)')
plt.ylabel('Roll Rate (deg/s)')
plt.xlim([t0,tf])
pp.savefig()

plt.figure()
plt.plot(time,pitch_rate,label='Exp')
plt.plot(time_sim,q_sim,label='Sim')
plt.grid()
plt.legend()
plt.xlabel('Time (sec)')
plt.ylabel('Pitch Rate (deg/s)')
plt.xlim([t0,tf])
pp.savefig()

plt.figure()
plt.plot(time,yaw_rate,label='Exp')
plt.plot(time_sim,r_sim,label='Sim')
plt.grid()
plt.legend()
plt.xlabel('Time (sec)')
plt.ylabel('Yaw Rate (deg/s)')
plt.xlim([t0,tf])
pp.savefig()

plt.figure()
plt.plot(time,motor_signal,label='Exp')
plt.plot(time_sim,mewt_sim,label='Sim')
plt.grid()
plt.legend()
plt.xlabel('Time (sec)')
plt.ylabel('Motor Signal (us)')
plt.xlim([t0,tf])
pp.savefig()

plt.figure()
plt.plot(time,aileron,label='Exp')
plt.plot(time_sim,aileron_signal_sim,label='Sim')
plt.legend()
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Aileron Signal (us)')
plt.xlim([t0,tf])
pp.savefig()

plt.figure()
plt.plot(time_sim,da_sim)
plt.xlabel('Time (s)')
plt.ylabel('Aileron (deg)')
plt.grid()
plt.title('Aileron v. t')
pp.savefig()

plt.figure()
plt.plot(time,elevator,label='Exp')
plt.plot(time_sim,elevator_signal_sim,label='Sim')
plt.legend()
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Elevator Signal (us)')
plt.xlim([t0,tf])
pp.savefig()

plt.figure()
plt.plot(time_sim,de_sim)
plt.xlabel('Time (s)')
plt.ylabel('Elevator (deg)')
plt.grid()
plt.title('Elevator v. t')
pp.savefig()

plt.figure()
plt.plot(time,rudder,label='Exp')
plt.plot(time_sim,rudder_signal_sim,label='Sim')
plt.legend()
plt.grid()
plt.xlabel('Time (sec)')
plt.ylabel('Rudder Signal (us)')
plt.xlim([t0,tf])
pp.savefig()

plt.figure()
plt.plot(time_sim,dr_sim)
plt.xlabel('Time (s)')
plt.ylabel('Rudder (deg)')
plt.grid()
plt.title('Rudder v. t')
pp.savefig()

plt.figure()
plt.plot(time_sim,T_sim)
plt.xlabel('Time (s)')
plt.ylabel('Thrust (N) - Simulation Only')
plt.grid()
plt.title('Thrust v. t')
pp.savefig()

pp.close()
