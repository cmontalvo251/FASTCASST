#ifndef BME280_H
#define BME280_H

#include <stdint.h>

#include "PTHSensor.h"
#include "Adafruit_BME280.h"

class BME280 : public PTHSensor {
public:
	//Functions required for PTHSensor
	bool initialize();
	bool update(double);

	//Functions specific to BME
	Adafruit_BME280 bme;
	BME280();
};

#endif