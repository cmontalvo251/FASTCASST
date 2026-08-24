#ifndef WRF_H
#define WRF_H
//%%This is the front end of uvwout(), run this script  */
/* %before you run uvwout You only need to run it once */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define dim 40
#define tlength 601

class WRF {
  private:
    char* PATH="libraries/wrf/WRF_Data_Static/";
    char* temp=(char*)malloc(8);
    int markX,markY,markZ,markT,parameters[5],mark1[2],mark2[2];
    int bounds,boundflag,pathlength,dx,dy,ztop;
    double xcoord[dim],ycoord[dim],zcoord[dim],terrain[dim][dim],tcoord[tlength];
    double U0[dim][dim][dim],Udt[dim][dim][dim],V0[dim][dim][dim],Vdt[dim][dim][dim];
    double W0[dim][dim][dim],Wdt[dim][dim][dim];
    char* U0name=(char*)malloc(256);
    char* Udtname =(char*)malloc(256);
    char* V0name=(char*)malloc(256);
    char* Vdtname =(char*)malloc(256);
    char* W0name=(char*)malloc(256);
    char* Wdtname =(char*)malloc(256);
    void importwind(double outmat[dim][dim][dim],char* file);
    int find(double invec[],int row,double value);
  public:
    WRF();
    void getWRF(double wind[3],double xstar,double ystar,double zstar,double tstar);
};

#endif