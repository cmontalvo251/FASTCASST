/*
  Written by Alexey Bulatov (alexey.bulatov@emlid.com) for Raspberry Pi
*/

#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>
#include "InertialSensor.h"
#include "Datalogger.h"
#include "Adafruit_BNO055.h"
#include "timer.h"

class BNO055 : public InertialSensor
{
public:
    BNO055();

    Adafruit_BNO055 bno;

    bool initialize();
    bool probe();
    void update();
    void getEuler(double*,double*,double*);

    //imu::Vector<3> euler;
    //imu::Vector<3> rate;
    //imu::Vector<3> accel;
    //imu::Vector<3> mag;
    //imu::Quaternion quat;

private:
   
};

#endif //BNO055