#ifndef GUIDANCE_H
#define GUIDANCE_H

#ifdef ARDUINO
#include "MATLAB.h"
#include "RCIO.h" //this is for STICK values
#else
#include <MATLAB/MATLAB.h>
#include <RCIO/RCIO.h> //this is for STICK values
#endif

class guidance {
public:
  int GUIDANCE_FLAG=0;
  int NUMSIGNALS=6; //roll,pitch,yaw,vel,alt,armswitch
  MATLAB guidance_matrix; //This is a vector of TAERA1A2 in PWM signals
  void loop(double currentTime,int rx_array[],MATLAB sense_matrix);
  void init(MATLAB in_configuration_matrix);
  void print();
  guidance(); //constructor
private:
  double elapsedTime = 0,lastTime=0; //These are used to keep track of time elapsed.
  double throttle_rx,aileron_rx,elevator_rx,rudder_rx,arm_switch_rx;
  double roll_command,pitch_command,yaw_rate_command,altitude_command,velocity_command;
  void set_defaults();
};

#endif
