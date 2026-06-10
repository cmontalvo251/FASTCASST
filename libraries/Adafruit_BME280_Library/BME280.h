#ifndef BME280_H
#define BME280_H

#include <stdint.h>

#ifdef ARDUINO
#include "PTHSensor.h"
#include "Adafruit_BME280.h"
#else
#include <PTH/PTHSensor.h>
#endif

class BME280 : public PTHSensor {
public:
	//Functions required for PTHSensor
	bool initialize();
	bool update(double);

	//Functions specific to BME
	#ifdef ARDUINO
	Adafruit_BME280 bme;
	#else
	bool bme;
	#endif
	BME280();
};

#endif