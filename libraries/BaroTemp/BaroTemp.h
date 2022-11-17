#ifndef BAROTEMP_H
#define BAROTEMP_H

#include <Mathp/mathp.h>
#include <BaroTemp/MS5611.h>
#include <Util/Util.h>
#include <unistd.h>
#include <stdio.h>

#define SLEEP_TIME 0.01 //seconds
#define LOOP_TIME 1.0 //seconds
#define NOMINALTEMP 25.5

class BaroTemp {
 private:
  MS5611 barometer;
  int CALIBRATE = 0;
  int CALIBRATE_FLAG = 1;
  double pressure0=0;
  double updatetime=-99;
  double Z=0;
  int PHASE=0;
  void ConvertPressure2Altitude();
 public:
  BaroTemp(); //constructor
  void poll(double currentTime);
  double temperature=NOMINALTEMP;
  double pressure=-99;
  double altitude=0.0; //Initialize altitude to zero
  void SendZ(double);
};

#endif
