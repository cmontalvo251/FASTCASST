#include "mathp.h"

void ClearHome() {
 #ifdef WIN32
  system("CLS");
 #else
  system("clear");
 #endif
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
