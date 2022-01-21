#ifndef SENSORS_H
#define SENSORS_H

//Helper Modules
#include <MATLAB/MATLAB.h>

//Hardware
//Analog Signals
#include <ADC/ADC.h>
//Barometer and Thermometer
#include <BaroTemp/BaroTemp.h>

class sensors {
 private:
  ADC analog;
  BaroTemp atm;
 public:
  //constructor
  sensors();
  //Polling routine
  void poll(double currentTime);
  //Printing routine
  void print();  
};

#endif
