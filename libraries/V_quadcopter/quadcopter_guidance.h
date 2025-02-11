#ifndef GUIDANCE_H
#define GUIDANCE_H

#ifdef ARDUINO
#include "MATLAB.h"
#include "RCIO.h"
#include "Datalogger.h"
#else
#include <MATLAB/MATLAB.h>
#include <RCIO/RCIO.h>
#include <Datalogger/Datalogger.h>
#endif

class guidance {
private:
  double elapsedTime = 0,lastTime=0; //These are used to keep track of time elapsed.
  double dthrottle = 0;
  double altitude_prev = -999;
  double altitude_command = 0;
  double altitude_int = 0;
  double altitude_dot = 0;
  void AltitudeLoop(double,MATLAB);
 public:
  guidance();
  void loop(int[],double,MATLAB);
  void init();
  void anti_windup();
  //MATLAB guidance_matrix;
};

#endif GUIDANCE
