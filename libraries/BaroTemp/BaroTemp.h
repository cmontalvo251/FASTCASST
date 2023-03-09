#ifndef BAROTEMP_H
#define BAROTEMP_H

#ifdef ARDUINO
#include "mathp.h"
#include "Adafruit_MPL115A2.h"
#include "Datalogger.h"
#else
#include <Mathp/mathp.h>
#include <Datalogger/Datalogger.h>
#include <MS5611/MS5611.h>
#include <Util/Util.h>
#include <unistd.h>
#include <stdio.h>
#endif

//Different sleep times for different hardware
#ifdef ARDUINO
#define SLEEP_TIME 0.01
#else
#define SLEEP_TIME 0.01 //seconds
#endif

#define LOOP_TIME 1.0 //seconds
#define NOMINALTEMP 25.5

class BaroTemp {
 private:
  #ifdef ARDUINO
  //Use the MPL155A2
  Adafruit_MPL115A2 barometer;
  #else
  //Raspberry Pi uses the MS5611
  MS5611 barometer;
  #endif
  int CALIBRATE = 0;
  int CALIBRATE_FLAG = 1;
  double pressure0=0;
  double updatetime=-99;
  double Z=0;
  int PHASE=0;
  void ConvertPressure2Altitude();
 public:
  BaroTemp(); //constructor
  void init();
  void print();
  void poll(double currentTime);
  double temperature=NOMINALTEMP;
  double pressure=-99;
  double altitude=0.0; //Initialize altitude to zero
  void SendZ(double);
};

#endif
