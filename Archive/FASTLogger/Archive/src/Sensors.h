#ifndef SENSORS_H
#define SENSORS_H

#include <Common/IMU.h>
#include <Common/GPS.h>
#include <MATLAB/MATLAB.h>
#include <mathp.h>

class Sensors {
 private:
 public:
  IMU orientation;
  double gpstime = 1.0;
  GPS satellites;
  void update(double,double,int);
};

#endif
