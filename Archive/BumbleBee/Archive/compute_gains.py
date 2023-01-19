#!/usr/bin/python

import numpy as np

##Gains from Kyrell
kd = 0.9
ki = 0.5
kyaw = 1.0

##Some Unit Conversion
GEARTH = 9.81
GEARTHENG = 32.2
KG2LBF = 2.2
METERS2FT = 3.28

##Distances from cg to motors
ry = (0.10115*METERS2FT)
rx = (0.0762*METERS2FT)
rtorque = 1.0 #it doesn't matter where it is

##Computation of kt
Tmax = (5.0/GEARTH)*KG2LBF
dpwm = 1766-1019
#Tmax = kt*dpwm**2
kt = Tmax/dpwm**2
kq = np.sqrt(kt**3)/np.sqrt(2)

##Mass
mass = ((735./1000.)*KG2LBF)/GEARTHENG

#Dimensions
a = (6.5/100.)*METERS2FT

##Inertias
Ix = (0.0973*KG2LBF)*METERS2FT*METERS2FT;
Iy = (0.07413*KG2LBF)*METERS2FT*METERS2FT;
Iz = Ix + Iy - mass*(a**2)/6.0

##Computation of New Gains
kd_new = (kd**2)*rx*kt/Iy
ki_new = (ki**2)*rx*kt/Iy
kyaw_new = (kyaw**2)*kq

##Print to home
print 'kd = ',kd_new
print 'ki = ',ki_new
print 'kyaw = ',kyaw_new


