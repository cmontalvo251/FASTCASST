#ifndef _INERTIAL_SENSOR_H
#define _INERTIAL_SENSOR_H

class InertialSensor {
public:
    virtual bool initialize() = 0;
    virtual bool probe() = 0;
    virtual void update() = 0;

    double read_temperature() {return temperature;};
    void read_accelerometer(float *ax, float *ay, float *az) {*ax = _ax; *ay = _ay; *az = _az;};
    void read_gyroscope(float *gx, float *gy, float *gz) {*gx = _gx; *gy = _gy; *gz = _gz;};
    void read_magnetometer(float *mx, float *my, float *mz) {*mx = _mx; *my = _my; *mz = _mz;};

protected:
    double temperature;
    float _ax=0.0, _ay=0.0, _az=0.0;
    float _gx=0.0, _gy=0.0, _gz=0.0;
    float _mx=0.0, _my=0.0, _mz=0.0;
};

#endif //_INERTIAL_SENSOR_H
