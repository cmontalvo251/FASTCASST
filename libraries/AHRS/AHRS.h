/*
Mahony AHRS algorithm implemented by Madgwick
See: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

Adapted by Igor Vereninov (igor.vereninov@emlid.com)
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com
*/

#ifndef AHRS_H
#define AHRS_H

#ifdef ARDUINO
#include "mathp.h"
#else
#include <cmath>
#include <stdio.h>
#include <memory>
#include <IMU/InertialSensor.h>
#include <Mathp/mathp.h>
#endif

class AHRS {
 private:
  float gyroOffset[3];
  float twoKi;
  float twoKp;
  float integralFBx, integralFBy, integralFBz;
 public:
  AHRS();
  double q0, q1, q2, q3;
  void update(float ax,float ay,float az,float gx, float gy, float gz,float mx,float my,float mz,float elapsedTime);
  void updateNOMAG(float ax,float ay,float az,float gx,float gy,float gz,float elapsedTime);
  void setGyroOffset(float offx,float offy,float offz);
  void setQuaternions(float q0,float q1,float q2,float q3);
  void getEuler(double*,double*,double*);
  float invSqrt(float x);
  float getW();
  float getX();
  float getY();
  float getZ();
  float gx=0, gy=0, gz=0;
};

#endif // AHRS

