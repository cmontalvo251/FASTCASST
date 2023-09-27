#!/usr/bin/python

###Simulate a 2-wheeled inline vehicle

import numpy as np 
import matplotlib.pyplot as plt
from matplotlib import animation
import sys
from random import random
if sys.platform == 'linux2':
    sys.path.append('/home/carlos/Dropbox/BlackBox/mypy')
else:
    sys.path.append('c:\Users\cmontalvo\Dropbox\BlackBox\mypy')
from mypy import *

#Define global control variable
dnwc = 0.0
throttlec = 0.0
#Define other globals
NOSTATES = 8
#Define global prameters
l = 2.0
ORIGIN = np.asarray([30.415117,-88.104879])
xyprev = np.asarray([0.,0.])
uprev = 0
psiprev = 0.
GPStime = 0.
GPSUpdate = 0.5
TMAX = 0.

def GPS(xk):
    global ORIGIN
    lat_lon = convertXY2LATLON(xk[0:2],ORIGIN)
    return lat_lon

def Control(xk,tk):
    global xyprev,psiprev,GPStime,dnwc,throttlec,uprev
    #Assume your GPS measures LAT and LON
    #If it's time to sample GPS then sample otherwise just
    #use the old value of GPS
    if tk > GPStime:
        GPStime+=GPSUpdate #Update at 1Hz
        lat_lon = GPS(xk)
        #Now we need to convert the LAT and LON coordinates to 
        #x and y. This assumes an origin
        xynew = convertLATLON(lat_lon,ORIGIN)
        #Add some senor noise to simulate the GPS inaccuracy
        xynew[0] = xynew[0] + (0.5-random())*2.04
        xynew[1] = xynew[1] + (0.5-random())*2.04
        #Use the new value and the previous of xy to compute psi
        psinew = np.arctan2(xynew[1]-xyprev[1],xynew[0]-xyprev[0])
        #Use new and old value to get u
        unew = (np.sqrt(xynew[1]**2 + xynew[0]**2)-np.sqrt(xyprev[1]**2 + xyprev[0]**2))/GPSUpdate
        #Then use a moving average filter
        sp = 0.2
        psi = psinew*(1-sp) + psiprev*sp
        psiprev = psi
        #Use a moving average filter to filter out GPS noise
        s = 0.0
        xy = xynew*(1-s) + xyprev*s
        xyprev = xy
        #Use a moving average filter for velocity
        su = 0.0
        u = unew*(1-su) + uprev*su
        uprev = u
    else:
        xy = xyprev
        psi = psiprev
        u = uprev
    #Now we need to get an angle command to direct us to our
    #waypoint
    x = xy[0]
    y = xy[1]
    xc = 200
    yc = 200
    #This routine takes care of the wrapping nature of atan2
    delpsi = psiwrap(xc,yc,x,y,psi)
    #Now we can feed psic into the nose wheel command
    dnwc = 0.1*delpsi
    if abs(dnwc) > 30.0*np.pi/180.0:
        dnwc = 30.0*np.pi/180.0*np.sign(dnwc)
    #Feed u to throttle
    distance_to_target = np.sqrt((x-xc)**2 + (y-yc)**2)
    uc = 10.0
    #throttlec = 0.01*(uc-u)
    throttlec = 0.01*distance_to_target
    if throttlec > 1.0:
        throttlec = 1.0
    elif throttlec < 0.0:
        throttlec = 0.0
    if distance_to_target < 50:
        throttlec = 0.0

def Derivatives(xk,tk):
    #Set globals
    global l,dnwc,throttlec,TMAX
    #Unwrap State Vector
    x = xk[0]
    y = xk[1]
    psi = xk[2]
    u = xk[3]
    v = xk[4]
    r = xk[5]
    dnw = xk[6]
    throttle = xk[7]
    
    dxdt = np.zeros(NOSTATES)

    #Kinematics    
    uv = xk[3:5]
    T = np.array([[np.cos(psi),-np.sin(psi)],[np.sin(psi),np.cos(psi)]])    
    xydot = np.dot(T,uv)  #page 1
    dxdt[0:2] = xydot 

    #Rotational Kinematics
    dxdt[2] = r #page 1
    
    #Some helpful rotations
    tan = np.tan(dnw)
    sin = np.sin(dnw)
    cos = np.cos(dnw)
    
    #Mass and Inertia Parameters
    m = 2.6
    cb = 0.003
    g = 9.81

    #Forces
    Froll = cb*m*g*np.sign(u) #again page 1
    TNW = np.asarray([[np.cos(dnw),-np.sin(dnw)],[np.sin(dnw),np.cos(dnw)]]) #page 1

    #The lateral force from the front wheel
    #is fairly complex. The derivation was done by hand
    #Inorm = l**2.*m/Izz
    #II = (1-Inorm)
    #numerator = m*r*u + tan*(T+m*r*v-Froll)-Inorm*sin*Froll
    #denominator = II*cos+tan*sin
    #FWLAT = -numerator/denominator 
    #vnw_nw = cos*(v+l*r) - sin*u
    #FWLAT = -c*vnw_nw
    
    #Enforce vdot = 0 Enforcing no tokyo drift
    FWLAT = (m*r*u + sin*Froll)/cos #page 4
    
    #Need T to enforce udot = 0
    #T = sin*FWLAT + (1-cos)*Froll - m*r*v #page 4
    
    #Use throttle to get thrust (TMAX = 1.0,PMAX = 10)
    #Power = T*v - let PMAX = TMAX*10
    #This is a new derivation that I did in my head because i didn't want to enforce
    #udot = 0 anymore
    PMAX = 50.0
    TMAX = PMAX/10.0
    #T = Power/v
    P = PMAX*throttle
    Thrust = P/u
    if Thrust > TMAX:
        Thrust = TMAX

    #Total forces on the body - page 1
    XYNW_NWFrame = np.asarray([-Froll,FWLAT])
    XYNW = np.dot(TNW,XYNW_NWFrame)
    XY = np.asarray([-Froll + Thrust,0.0]) + XYNW

    #Moments - page 1
    YNW = XYNW[1]
    N = l*YNW
    
    #Translational Dynamics - page 1 and 4
    uvdot = XY/m - np.asarray([-r*v,r*u])
    dxdt[3:5] = uvdot
    
    #Rotational Dynamics
    #Compute r based on dnw
    #r = tan(dnw)*u/l - page 3
        
    #Nose wheel dynamics
    tau = 0.6
    dxdt[6] = tau*(dnwc-dnw) #these are first order filters

    #Throttle Dynamics
    taut = 0.6
    dxdt[7] = taut*(throttlec-throttle) #these are first order filters

    #Rotational Dynamics
    #r = tan(dnw)*u/l - page 3
    #use derivation from page 3 and take a derivative
    #rdot = sec^2(dnw)*dnwdot*u/l + tan(dnw)*udot/l
    #remember that dnwdot = tau*(dnwc-dnw)
    dxdt[5] = (tau*(dnwc-dnw)*(sec(dnw)**2)*u + tan*uvdot[0])/l
    
    return dxdt

#################MAIN CODE#############
#######################################

dt = 0.01
tend = 200.0

time = np.arange(0,tend,dt)
xstate = np.zeros([NOSTATES,len(time)])

#Initial Conditions
x0 = 0.0
y0 = 0.
psi0 = 0.
u0 = 0.1 #Don't set this to zero
v0 = 0.
r0 = 0.
dnw0 = 0.
throttle0 = 0.
xstate0 = np.asarray([x0,y0,psi0,u0,v0,r0,dnw0,throttle0])
xstate[:,0] = xstate0

for idx in range(0,len(time)-1):
    xk = xstate[:,idx]
    tk = time[idx]
    #Call Control
    Control(xk,tk)
    #Integrate
    k1 = Derivatives(xk,tk)
    k2 = Derivatives(xk+k1*dt/2.,tk+dt/2.)
    k3 = Derivatives(xk+k2*dt/2.,tk+dt/2.)
    k4 = Derivatives(xk+k3*dt,tk+dt)
    phi = (1./6.)*(k1 + 2.*k2 + 2.*k3 + k4)
    xstate[:,idx+1] = xstate[:,idx] + phi*dt
    #print 'Iteration = ',idx
    
#print xstate

##Print all variables
f,axarr = plt.subplots(2,3)
iter = 0
State_Names = ['X (m)','Y (m)','Psi (rad)','U (m/s)','V (m/s)','R (rad/s)']
for row in range(0,2):
    for col in range(0,3):
        axarr[row,col].plot(time,xstate[iter,:])
        axarr[row,col].set_title(State_Names[iter])
        iter+=1

##Print Control Variabls
f,axarr = plt.subplots(1,2)
iter = 0
State_Names = ['Nose Wheel (rad)','Throttle (0-1)']
for col in range(0,2):
    axarr[col].plot(time,xstate[iter+6,:])
    axarr[col].set_title(State_Names[iter])
    iter+=1

#Plot a Topdown view of the bicycle
plottool(12,'X (m)','Y (m)','TopDown')
x = xstate[0,:]
y = xstate[1,:]
plt.plot(x,y)

#Plot Nosewheel angle
#f2 = plt.figure()
#plt.plot(time,nwstate)
#plt.xlabel('Time (sec)')
#plt.ylabel('Delta Nose Wheel (rad)')

#Plot Nose Wheel Velocity in Body and Nose Wheel Frame
#ustate = xstate[3,:]
#vstate = xstate[4,:]
#rstate = np.tan(nwstate)*ustate/l
#unw_body = ustate
#vnw_body = vstate+l*rstate
#unw_nw = 0*unw_body
#vnw_nw = 0*vnw_body
#for idx in range(0,len(time)):
#    dnw = nwstate[idx]
#    unw_nw[idx] = np.cos(dnw)*unw_body[idx] + np.sin(dnw)*vnw_body[idx]
#    vnw_nw[idx] = -np.sin(dnw)*unw_body[idx] + np.cos(dnw)*vnw_body[idx]

#f3,axarr3 = plt.subplots(2,2)
#axarr3[0,0].plot(time,unw_body)
#axarr3[0,0].set_title('U NW Body (m/s)')
#axarr3[0,1].plot(time,vnw_body)
#axarr3[0,1].set_title('V NW Body (m/s)')

#axarr3[1,0].plot(time,unw_nw)
#axarr3[1,0].set_title('U NW NW (m/s)')
#axarr3[1,1].plot(time,vnw_nw)
#axarr3[1,1].set_title('V NW NW (m/s)')

###Try to run the animation
def init():
    line.set_data([], [])
    line2.set_data([],[])
    return line,line2
    
# animation function.  This is called sequentially
def animate(n):
    global xstate
    x = xstate[0,n]
    y = xstate[1,n]
    psi = xstate[2,n]
    xnw = x + np.cos(psi)*l
    ynw = y + np.sin(psi)*l
    line.set_data([x,xnw], [y,ynw])
    line2.set_data([x,xnw],[y,ynw])
    return line,line2

# First set up the figure, the axis, and the plot element we want to animate
fig = plt.figure()
ax = plt.axes(xlim=(np.min(x), np.max(x)), ylim=(np.min(y), np.max(y)))
line, = ax.plot([], [], lw=2)
line2, = ax.plot([], [], lw=2)

#Smaller numbers go faster BUT MUST BE INTEGERS!!!
speed = 1

#call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init,frames=len(time), interval=speed, blit=True)

plt.show()
    

