#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>
#include "uvwfrontend.h" //startup routine for windmodel

using namespace std;

int main()
{
  double uvw[3],x,y,z,t;

  UVWSTARTUP();

  x = 0;
  z = 50;
  y = 0;
  t = 1;
  uvwwind(uvw,x,y,z,t);
  printf("uvw = %lf %lf %lf \n",uvw[0],uvw[1],uvw[2]);
  t = 1.1;
  uvwwind(uvw,x,y,z,t);
  printf("uvw = %lf %lf %lf \n",uvw[0],uvw[1],uvw[2]);


  return 0;
}
