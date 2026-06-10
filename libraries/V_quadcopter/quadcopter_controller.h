#ifndef CONTROLLER_H
#define CONTROLLER_H

//This is another class that is craft dependent as as such
//must adhere to specific standards

#ifdef ARDUINO
#include "MATLAB.h"
#include "RCIO.h"
#include "Datalogger.h"
#else
#include <MATLAB/MATLAB.h>
#include <RCIO/RCIO.h> //this is for STICK values
#include <Datalogger/Datalogger.h>
#endif

#include "quadcopter_guidance.h"

class controller {
private:
  double elapsedTime = 0,lastTime=0; //These are used to keep track of time elapsed.
  double motor_upper_left,motor_upper_right,motor_lower_left,motor_lower_right;
  double roll_command,pitch_command,yaw_rate_command,altitude_command;
  double droll=0,dyaw=0,dthrottle=0,dpitch=0;
  double altitude_int = 0;
  void AltitudeLoop(MATLAB);
  void YawRateLoop(MATLAB);
  void InnerLoop(MATLAB);
  void autoloop(double currentTime,int rx_array[],MATLAB sense_matrix,int icontrol);
  int CONTROLLER_FLAG = -99;
  void set_defaults();
  guidance guid;
public:
  int NUMSIGNALS=5; //TAER+Arm Switch
  MATLAB control_matrix; //This is a vector of TAERA1A2 in PWM signals
  void loop(double currentTime,int rx_array[],MATLAB sense_matrix);
  void init(MATLAB in_configuration_matrix);
  void init(int);
  void print();
  controller(); //constructor
};

#endif
