#ifndef BAROTEMP_H
#define BAROTEMP_H

#include <BaroTemp/MS5611.h>
#include <Util/Util.h>
#include <unistd.h>
#include <stdio.h>

#define SLEEP_TIME 0.01 //seconds
#define LOOP_TIME 1.0 //seconds

class BaroTemp {
 private:
  MS5611 barometer;
  double pressure0=-99;
  double updatetime=-99;
  double Z=0;
  int PHASE=0;
  void ConvertZ2Pressure();
  void ConvertPressure2Altitude();
 public:
  BaroTemp(); //constructor
  void poll(double currentTime);
  double temperature=-99;
  double pressure=-99;
  double altitude=-99;
  void SendZ(double);
};

#endif
