#include "BNO055.h"

BNO055::BNO055()
{
    bno = Adafruit_BNO055(55);
}

//This runs first
bool BNO055::probe() {
    return initialize();
}

bool BNO055::initialize() {
    bool status = bno.begin();
    if (status) {
        bno.setExtCrystalUse(true);
    }
    return status;
}

void BNO055::update() {
    rate = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    _gx = rate.x();
    _gy = rate.y();
    _gz = rate.z();
    accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    _ax = accel.x();
    _ay = accel.y();
    _az = accel.z();
    mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    _mx = mag.x();
    _my = mag.y();
    _mz = mag.z();
    quat = bno.getQuat();
    _q0 = quat.w();
    _q1 = quat.x();
    _q2 = quat.y();
    _q3 = quat.z();
}