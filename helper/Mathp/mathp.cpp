#include "mathp.h"

void ClearHome() {
 #ifdef WIN32
  system("CLS");
 #else
  system("clear");
 #endif
}

void ConvertXYZ2LLH(double XYZ[],double LLH[],double X_origin,double Y_origin) {
  double X = XYZ[0];
  double Y = XYZ[1];
  double Z = XYZ[2];
  double dlat = X/GPSVAL;
  double latitude = (dlat + X_origin);
  //printf("dlat = %lf latitude = %lf X = %lf origin = %lf \n",dlat,latitude,X,X_origin);
  double longitude = Y/(GPSVAL*cos(X_origin*PI/180.0)) + Y_origin;
  double altitude = -Z;
  LLH[0] = latitude;
  LLH[1] = longitude;
  LLH[2] = altitude;
  //printf("LLH = %lf %lf %lf \n",latitude,longitude,altitude);
}

double sat(double input,double epsilon,double scalefactor)
{
  if (input > epsilon) {
    //Right side of graph
    return scalefactor;
  } else if (input < -epsilon) {
    //left side of graph
    return -scalefactor;
  } else {
    //Inside the boundary so interpolate
    return scalefactor*input/epsilon;
  }
}

double randnum(double start,double end) //returns a random number from start to end
{
  //srand(time(NULL)+rand());
  //rand() returns a random integer from 0 to RAND_MAX
  //So first we need to scale rand by the start and end values
  double scalefactor = RAND_MAX/(end-start);
  double num = rand()/scalefactor;
  return (num + start);
}

void printdouble(double in,char name[]) {
	printf("\n");
	printf("%s %s %.8e \n",name," = ",in);
	printf("\n");
}

double delpsi(double psi,double psic) {
  double spsi = sin(psi);
  double cpsi = cos(psi);
  double spsic = sin(psic);
  double cpsic = cos(psic);
  double out = atan2(spsi*cpsic-cpsi*spsic,cpsi*cpsic+spsi*spsic);
  return (out);
}

// Copyright - Carlos Montalvo 2015
// You may freely distribute this file but please keep my name in here
// as the original owner
