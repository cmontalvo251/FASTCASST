#ifndef RCIO_H
#define RCIO_H

///////////INPUTS TO RCIO CLASS///////////
// 1 - in_configuration_matrix (MATLAB)
// 2 - control_matrix (MATLAB)

//////////OUTPUTS FROM RCIO CLASS////////////
// 1 - rx_array (double array)
// 2 - pwm_array (double array)

#ifdef ARDUINO
#include <RCInput.h>
#include <RCOutput.h>
#else
#include <RCInput/RCInput.h>
#include <RCOutput/RCOutput.h>
#endif

class RCIO {
 private:
 public:
  //Rcinput class
  RCInput in;
  //RCoutput class
  RCOutput out;
  //constructor
  RCIO();
  //Read receiver signals
  void read();
  //Print receiver channels
  void printIn(int);
  //write pwm channels
  void write();
  //Initialize PWM channels
  void outInit(int num);
  //print pwm channels
  void printOut();
};

#endif
