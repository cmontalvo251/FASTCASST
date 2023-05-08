#ifndef LPS22_H
#define LPS22_H

#include <stdint.h>

#ifdef ARDUINO
#include "PTHSensor.h"
#include "Adafruit_LPS2X.h"
#else
#include <PTH/PTHSensor.h>
#endif

class LPS22 : public PTHSensor {
public:
	//Functions required for PTHSensor
	bool initialize();
	bool update(double);

	//Functions specific to BME
	#ifdef ARDUINO
	sensors_event_t temp;
	sensors_event_t pressure;
	Adafruit_LPS22 lps;
	#else
	double temp,pressure;
	bool lps;
	#endif
	LPS22();
};

#endif