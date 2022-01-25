//////Facility for Aerial Systems and Technology (FAST) Real Time Simulator/////////
/////Initial edited by Carlos Montalvo

//See the following Issue on Github - https://github.com/cmontalvo251/C/issues/3
using namespace std;
//using namespace boost;

#include "timer.h"
#include <iostream>
#include <stdlib.h>

//define Global time parameters 
#include "IMU.h"

//////Main/////////////
int main(int argc,char** argv) {
  double PRINT = 0;
  double startTime,current_time,prev_time;
  TIMER timer;
  IMU orientation;
  startTime = timer.getTimeSinceStart();
  current_time = timer.getTimeSinceStart() - startTime;
  double PRINTRATE = 0.1;
  orientation.init(0); //0 for MPU and 1 for LSM

  //Kick off main while loop
  while (1) {
    current_time = timer.getTimeSinceStart()-startTime;
    double dt = current_time - prev_time;
    prev_time = current_time;

    orientation.FilterConstant = 0.0; 
    orientation.loop(dt);
    
    /////////////////Print to STDOUT////////////////
    if (PRINT<current_time) {
      printf("%f ",current_time);
      printf("%lf %lf %lf ",orientation.roll,orientation.pitch,orientation.yaw);
      printf("\n");
      PRINT+=(1*PRINTRATE);
      //PAUSE();
    }
    /////////////////////////////////////////////////

  } //End while loop on main loop
  printf("Simulation Complete\n");
}
