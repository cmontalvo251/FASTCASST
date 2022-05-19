#ifndef IMU_H
#define IMU_H

#include <Common/MPU9250.h>
#include <Navio2/AHRS.h>

class IMU {
 public:
  InertialSensor *mpu;
  AHRS ahrs;
  IMU();
  float ax=0, ay=0, az=0;
  float gx=0, gy=0, gz=0;
  float mx=0, my=0, mz=0;
  double temperature=25.0; //Hardcoded again
  double roll=0.0,pitch=0.0,yaw=0.0;
  double gx_filtered=0, gy_filtered=0, gz_filtered=0;
  double offset[3];
  double roll_rate=0.0,pitch_rate=0.0,yaw_rate=0.0;
  void imuSetup();
  void setOrientation(double,double,double,double,double,double);
  void setTemperature(double);
  void loop(double,double);
};

#endif
