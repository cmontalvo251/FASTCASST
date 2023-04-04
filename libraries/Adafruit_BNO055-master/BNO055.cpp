#include "BNO055.h"

BNO055::BNO055()
{
    bno = Adafruit_BNO055(55);
}

//This runs first
bool BNO055::probe() {
    printstdout("Probing BNO55 \n");
    bool status = bno.begin();
    if (status) {
        printstdout("BNO Begin success! Waiting 1 second to calibrate DO NOT MOVE! \n");
        cross_sleep(1.0);
    } else {
        printstdout("BNO NOT INITIALIZED PROPERLY \n");
        exit(1);
    }    
    return status;
}

bool BNO055::initialize() {
    bno.setExtCrystalUse(true);
    return true;
}

void BNO055::update() {
    imu::Vector<3> rate = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    /*Serial.print(rate.x(),8);
    Serial.print(" ");
    Serial.print(rate.y(),8);
    Serial.print(" ");
    Serial.print(rate.z(),8);
    Serial.print("\n");*/
    _gx = rate.x();
    _gy = rate.y();
    _gz = rate.z();
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    _ax = accel.x();
    _ay = accel.y();
    _az = accel.z();
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    _mx = mag.x();
    _my = mag.y();
    _mz = mag.z();
    imu::Quaternion quat = bno.getQuat();
    _q0 = quat.w();
    _q1 = quat.x();
    _q2 = quat.y();
    _q3 = quat.z();
}