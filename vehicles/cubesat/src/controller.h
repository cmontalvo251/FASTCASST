#ifndef CONTROLLER_H
#define CONTROLLER_H

//This is another class that is craft dependent as as such
//must adhere to specific standards

#include "params.h"
#include <MATLAB/MATLAB.h>
#include <RCIO/RCIO.h> //this is for STICK values
#include <Timer/timer.h>

class controller {
private:
  MATLAB pqr,mxyz,desired_moments,fully_controlled,gamma,I,I_pqr,I_gamma,pqrskew_I_pqr;
  double p_command,r_command,q_command;
  double elapsedTime = 0,lastTime=0; //These are used to keep track of time elapsed.
  double delOmega_max = 10,delPWM = (STICK_MAX-STICK_MIN),pwmC = (delPWM/delOmega_max);
  int CONTROLLER_FLAG = -99;
  void set_defaults();
  void ProportionalLoop(MATLAB);
  void FeedbackLinearizedLoop(MATLAB);
  void TwoStageLoop(MATLAB);
  void CleanControl();
public:
  int NUMSIGNALS=NUMTORQUERS; //Number set in params.h
  MATLAB control_matrix; //This is a vector of TAERA1A2 in PWM signals
  void loop(double currentTime,int rx_array[],MATLAB sense_matrix);
  void init(MATLAB in_configuration_matrix);
  void print();
  controller(); //constructor
};

#endif
