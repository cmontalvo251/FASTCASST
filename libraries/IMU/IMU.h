#ifndef IMU_H
#define IMU_H

#ifdef ARDUINO
#include "AHRS.h"
#include "InertialSensor.h"
#include "mathp.h"
#include "Datalogger.h"
#include "LSM9DS1.h"
#include "MPU9250.h"
#include "timer.h"
#include "BNO055.h"
#else
#include <Timer/timer.h>
#include <MPU9250/MPU9250.h>
#include <LSM9DS1/LSM9DS1.h>
#include <AHRS/AHRS.h>
#include <Mathp/mathp.h>
#include <Datalogger/Datalogger.h>
#endif

class IMU {
 public:
  int sensor_type = 0;
  InertialSensor *imu_sensor;
  AHRS ahrs;
  IMU();
  double TEMP_SCALE = 1.0;
  float ax=0, ay=0, az=0;
  float gx=0, gy=0, gz=0;
  float mx=0, my=0, mz=0;
  float q0=0,q1=0,q2=0,q3=0;
  double FilterConstant=0.0; //Filter constant for the complementary filter. 
  double temperature=25.0; //Hardcoded again
  double roll=0.0,pitch=0.0,yaw=0.0,magyaw=0;
  double gx_filtered=0, gy_filtered=0, gz_filtered=0;
  double offset[3];
  double roll_rate=0.0,pitch_rate=0.0,yaw_rate=0.0;
  void init(int);
  void setOrientation(double,double,double,double,double,double);
  void setTemperature(double);
  void loop(double);
  void printALL();
  void getMagHeading();
  void printEuler();
  void printRates();
  void filterGyro();
};

#endif
