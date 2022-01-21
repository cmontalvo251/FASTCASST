#ifndef IMU_H
#define IMU_H

#ifdef ARDUINO
#include "MPU9250.h"
#else
#include <IMU/MPU9250.h>
#include <IMU/LSM9DS1.h>
#include <IMU/AHRS.h>
#include <Mathp/mathp.h>
#endif

class IMU {
 public:
  InertialSensor *mpulsm;
  AHRS ahrs;
  IMU();
  double TEMP_SCALE = 1.0;
  float ax=0, ay=0, az=0;
  float gx=0, gy=0, gz=0;
  float mx=0, my=0, mz=0;
  double FilterConstant=0.0; //Filter constant for the complementary filter. 
  double temperature=25.0; //Hardcoded again
  double roll=0.0,pitch=0.0,yaw=0.0;
  double gx_filtered=0, gy_filtered=0, gz_filtered=0;
  double offset[3];
  double roll_rate=0.0,pitch_rate=0.0,yaw_rate=0.0;
  void init(int);
  void setOrientation(double,double,double,double,double,double);
  void setTemperature(double);
  void loop(double);
  void printALL();
  void printEuler();
  void printRates();
  void filter();
};

#endif
