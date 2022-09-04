#ifndef CONTROLLER_H
#define CONTROLLER_H

//This is another class that is craft dependent as as such
//must adhere to specific standards

#include <MATLAB/MATLAB.h>
#include <RCIO/RCIO.h> //this is for STICK values

class controller {
private:
  double elapsedTime = 0,lastTime=0; //These are used to keep track of time elapsed.
  int CONTROLLER_FLAG = -99;
  void set_defaults();
public:
  int NUMSIGNALS=2; //1 motor and 1 servo
  MATLAB control_matrix; //This is a vector of motor and servo pwm signals
  void loop(double currentTime,int rx_array[],MATLAB sense_matrix);
  void init(MATLAB in_configuration_matrix);
  void print();
  controller(); //constructor
};

#endif
