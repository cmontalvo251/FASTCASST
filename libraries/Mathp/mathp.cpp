#include "mathp.h"

void ClearHome() {
 #ifdef WIN32
  system("CLS");
 #else
  system("clear");
 #endif
}

double ConvertZ2Pressure(double Z) {
  //printf("Z = %lf \n",Z);
  double altitude = -Z;
  //printf("Altitude = %lf \n",altitude);
  double pascals = 101325.0*pow((1.0-2.25577*pow(10,(-5.0))*altitude),5.25588);
  //printf("Pascals = %lf \n",pascals);
  double pressure = pascals*0.01;
  return pressure;
}

void ConvertLLH2XYZ(double XYZ[],double LLH[],double X_origin,double Y_origin){
  //Extract LLH
  double latitude = LLH[0];
  double longitude = LLH[1];
  double altitude = LLH[2];
  ///CONVERT LAT LON TO METERS
  double X,Y,Z;
  Y = (longitude - Y_origin)*GPSVAL*cos(X_origin*DEG2RAD);  
  X = (latitude - X_origin)*GPSVAL;
  Z = -altitude;
  //Populate XYZ matrix
  XYZ[0] = X;
  XYZ[1] = Y;
  XYZ[2] = Z;
  //printf("ConvertLLH2XY: %lf %lf %lf %lf \n",X,Y,latitude,longitude);
}

void ConvertLLH2XYZSPHERICAL(double XYZ[],double LLH[]) {
  //Extract LLH
  double latitude = LLH[0];
  double longitude = LLH[1];
  double altitude = LLH[2];
  //Compute spherical coordinates
  double thetaE = (90.0 - latitude)*PI/180.0;
  double psiE = longitude*PI/180.0;
  double rho = altitude + REARTH;
  ///CONVERT LAT LON TO METERS
  double X,Y,Z; 
  X = rho*sin(thetaE)*cos(psiE);
  Y = rho*sin(thetaE)*sin(psiE);
  Z = rho*cos(thetaE);
  //Populate XYZ matrix
  XYZ[0] = X;
  XYZ[1] = Y;
  XYZ[2] = Z;
}

void ConvertXYZ2LLHSPHERICAL(double XYZ[],double LLH[]) {
  double X = XYZ[0];
  double Y = XYZ[1];
  double Z = XYZ[2];
  //printf("XYZ = %lf %lf %lf \n",X,Y,Z);
  double rho = sqrt(X*X + Y*Y + Z*Z);
  double thetaE = acos(Z/rho);
  double psiE = atan2(Y,X);
  double latitude = 90.0 - thetaE*180.0/PI;
  double longitude = psiE*180.0/PI;
  double altitude = rho - REARTH;
  LLH[0] = latitude;
  LLH[1] = longitude;
  LLH[2] = altitude;
  //printf("LLH = %lf %lf %lf \n",latitude,longitude,altitude);
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

double delpsi(double psi,double psic) {
//%%%returns delta psi from a heading and a heading command without
//%worrying about wrapping issues
//%%%This computes delpsi = psi-psic btw

double spsi = sin(psi);
double cpsi = cos(psi);
double spsic = sin(psic);
double cpsic = cos(psic);

double out = atan2(spsi*cpsic-cpsi*spsic,cpsi*cpsic+spsi*spsic);

return out;
}

double saturation(double input,double max_value) {
  if (abs(input) > max_value) {
    return copysign(1.0,input)*max_value;
  } else {
    return input;
  }
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

// Copyright - Carlos Montalvo 2015
// You may freely distribute this file but please keep my name in here
// as the original owner
