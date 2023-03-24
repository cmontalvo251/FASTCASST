#ifndef PTH_H
#define PTH_H

#ifdef ARDUINO
#include "PTHSensor.h"
#include "mathp.h"
#include "Datalogger.h"
#include "timer.h"
#else
#include <Timer/timer.h>
#include <Mathp/mathp.h>
#include <Datalogger/Datalogger.h>
#include <PTH/PTHSensor.h>
#endif

class PTH {
 public:
  //Variables
  double pressure,altitude,temperature,humidity,Z=0;
  //Constructor
  PTH();
  //Functions
  void init(int);
  void poll(double);
  void SendZ(double);
  void print();
 private:
  bool IPTH;
  int CALIBRATE=0;
  bool CALIBRATE_FLAG = 1;
  double pressure0=0;
  int sensor_type = 0;
  PTHSensor *pth_sensor;  
};

#endif
