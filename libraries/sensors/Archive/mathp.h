#ifndef MATHP_H
#define MATHP_H

#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

//MACROS
#define SQUARE(x) (((x)*(x)))
#define CUBE(x) (((x)*(x)*(x)))
#define PI 3.14159265358979323846264338327950288419716939937510582
#define MEARTH (5.972*pow(10,24)) //%%%Mass of the earth in kg
#define MSUN (1.989*pow(10,30)) //%%%Mass of the Sun in kg
#define GSPACE (6.6742*pow(10,-11)) //%%Gravitational constant - units?
#define MUEARTH (GSPACE*MEARTH)
#define REARTH          6371393. //Radius of earth - This is mean radius
#define REARTH_POLAR    6357000. //##this is polar radius
#define REARTH_EQ       6378100. //##this is equatorial radius
#define RAD2DEG ((double)180/PI)
#define DEG2RAD (PI/(double)180)
#define ANGULAREARTH ((PI)/(12.0*3600)) //Angular velocity of earth rad/s
#define METERS2FT 3.28
#define KG2LBF 2.2
#define GEARTHENG 32.2
#define GEARTH 9.80665 //graviational acceleration
#define NM2FT 6076.115485560000
#define FT2M 0.3048
#define GPSVAL (60.0*NM2FT*FT2M)
#define GRAVITYENG 32.2
#define GRAVITYSI 9.81
#define RHOSLSI 1.225
#define GNDSTIFF 112.0
#define GNDDAMP 50.0
#define GNDCOEFF 0.1
#define CONSTRAIN(in,min,max) (in > max ? max : (in < min ? min : in))
#define wrap_Pi(x) (x < -Pi ? x+Pi2 : (x > Pi ? x - Pi2: x))
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

using namespace std;

void ClearHome();
double randnum(double,double);
void printdouble(double,char*);
double delpsi(double,double);
double sat(double,double,double);

#endif

// Copyright - Carlos Montalvo 2015
// You may freely distribute this file but please keep my name in here
// as the original owner
