#ifndef PTH_H
#define PTH_H

#ifdef ARDUINO
#include "PTHSensor.h"
#include "mathp.h"
#include "Datalogger.h"
#include "timer.h"
#include "MS5611.h"
#else
#include <MS5611/MS5611.h>
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
    double pressure0=0;
    bool IPTH;
    int CALIBRATE=0;
    bool CALIBRATE_FLAG = 1;
    int sensor_type = 0;
    PTHSensor *pth_sensor;  
};

#endif
