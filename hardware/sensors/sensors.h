#ifndef SENSORS_H
#define SENSORS_H

///////////INPUTS TO SENSORS CLASS///////////
// 1 - in_error_matrix (MATLAB)
// 2 - model_matrix (MATLAB) - SIL
// 3 - UART_sense_matrix (MATLB) - HIL

//////////OUTPUTS FROM SENSORS CLASS////////////
// 1 - sense_matrix and semse_matrix_dot x3 (one to datalogger, one to uart, one to controller)

//Helper Modules
#include <MATLAB/MATLAB.h>

//Hardware
//Analog Signals
#include <ADC/ADC.h>
//Barometer and Thermometer
#include <BaroTemp/BaroTemp.h>
//IMU
#include <IMU/IMU.h>
//GPS
#include <GPS/GPS.h>

class sensors {
 private:
  ADC analog;
  BaroTemp atm;
  IMU orientation;
  GPS satellites;
  int NUMVARS;
 public:
  //Public variables
  MATLAB sense_matrix,sense_matrix_dot;
  char** headernames,headernames_dot;
  //constructor
  sensors();
  //Polling routine
  void poll(double currentTime,double elapsedTime);
  //Printing routine
  void print();
  //Get number of variabls routine
  int getNumVars();
};

#endif
