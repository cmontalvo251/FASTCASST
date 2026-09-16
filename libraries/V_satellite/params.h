#ifndef SATELLITE_PARAMS_H
#define SATELLITE_PARAMS_H
#define KMAG 67200
#define NUMTURNS 84
#define AREA 0.02
#define MAXCURRENT 0.12
#define NUMTORQUERS 3 //X,Y no Z (unless this is 3 in which case Z is active)
#define NUMRWS 3 //These are the number of reaction wheels in the system. 3 is the minimum for full 3-axis control.
#define RADRW 0.042 //Radius of reaction wheel (m)
#define MAXMOMENT 0.015 //Maximum momentum Nms
#define MASSRW 0.13 //Mass of reaction wheel (kg) //These are based on BCT RWp015
#define MAXTORQUE 0.004 //Maximum Torque N-m
#endif 
