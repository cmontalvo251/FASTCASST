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

class controller {
private:
  double elapsedTime = 0,lastTime=0; //These are used to keep track of time elapsed.
  double roll_command,pitch_command;
  double droll,dpitch,dyaw;
  void YawRateLoop_QUAD(MATLAB);
  void InnerLoop_QUAD(MATLAB);
  void InnerLoop_AIRPLANE(MATLAB);
  void autoloop(double currentTime,int rx_array[],MATLAB sense_matrix,int icontrol);
  int CONTROLLER_FLAG = -99;
  void set_defaults();
public:
  int NUMSIGNALS=8; //TAER+Arm Switch for quad and then AER for airplane so 8 total
  MATLAB control_matrix; //This is a vector of TAERA1A2 in PWM signals
  void loop(double currentTime,int rx_array[],MATLAB sense_matrix);
  void init(MATLAB in_configuration_matrix);
  void init(int);
  void print();
  controller(); //constructor
};

#endif
