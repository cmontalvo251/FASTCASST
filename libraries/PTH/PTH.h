#ifndef PTH_H
#define PTH_H

#ifdef ARDUINO
#include "PTHSensor.h"
#include "mathp.h"
#include "Datalogger.h"
#include "timer.h"
#include "MS5611.h"
#include "Adafruit_MPL115A2.h"
#include "BME280.h"
#include "LPS22.h"
#else
#include <Timer/timer.h>
#include <Mathp/mathp.h>
#include <Datalogger/Datalogger.h>
#include <PTH/PTHSensor.h>
//Right now these sensors only work on Arduino
//#include <Adafruit_MPL115A2-master/Adafruit_MPL115A2.h>
//#include <Adafruit_BME280_Library/BME280.h>
//This is the only one that works in AUTO mode on RPI
#include <MS5611/MS5611.h>
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
