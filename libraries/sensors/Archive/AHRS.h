/*
Mahony AHRS algorithm implemented by Madgwick
See: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

Adapted by Igor Vereninov (igor.vereninov@emlid.com)
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com
*/

#ifndef AHRS_H
#define AHRS_H

#include <cmath>
#include <stdio.h>
#include <memory>
#include "InertialSensor.h"
#include "mathp.h"

class AHRS {
 private:
  float q0, q1, q2, q3;
  float gyroOffset[3];
  float twoKi;
  float twoKp;
  float integralFBx, integralFBy, integralFBz;
 public:
  AHRS();
  void update(float ax,float ay,float az,float gx, float gy, float gz,float mx,float my,float mz,float elapsedTime);
  void updateNOMAG(float ax,float ay,float az,float gx,float gy,float gz,float elapsedTime);
  void setGyroOffset(float offx,float offy,float offz);
  void getEuler(double*,double*,double*);
  float invSqrt(float x);
  float getW();
  float getX();
  float getY();
  float getZ();
  float gx=0, gy=0, gz=0;
};

#endif // AHRS

