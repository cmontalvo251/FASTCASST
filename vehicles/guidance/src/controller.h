#ifndef CONTROLLER_H
#define CONTROLLER_H

//This is another class that is craft dependent as as such
//must adhere to specific standards

#include <MATLAB/MATLAB.h>
#include <RCIO/RCIO.h> //this is for STICK values

class controller {
private:
  double elapsedTime = 0,lastTime=0; //These are used to keep track of time elapsed.
  double throttle,aileron,elevator,rudder,arm_switch;
  double roll_command,pitch_command,yaw_rate_command,altitude_command;
  double altitude_prev = -999,droll=0,dyaw=0,dthrottle=0,dpitch=0;
  double altitude_int = 0;
  void AltitudeLoop(MATLAB); // Altitude
  void InnerLoop(MATLAB); // Roll,pitch,yaw
  int CONTROLLER_FLAG = -99;
  void set_defaults();
public:
  int NUMSIGNALS=5; //TAER(AS)
  MATLAB control_matrix; //This is a vector of TAERA1A2 in PWM signals
  void loop(double currentTime,int rx_array[],MATLAB sense_matrix);
  void init(MATLAB in_configuration_matrix);
  void print();
  controller(); //constructor
};

#endif
