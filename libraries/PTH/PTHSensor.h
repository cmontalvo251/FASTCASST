#ifndef _PTH_SENSOR_H
#define _PTH_SENSOR_H

class PTHSensor {
 public:
  ///Virtual Functions
  virtual bool initialize() = 0;
  virtual bool update(double) = 0;
  
  //Read Functions - Each sensor must populate these underscore variables.
  double read_temperature() {return _temperature;};
  double read_pressure() {return _pressure;};
  double read_humidity() {return _humidity;};

 protected:
  //Protected variables
  double _pressure=1013.25;
  double _temperature=25;
  double _humidity=0.0;
};

#endif
