#include "LPS22.h"

LPS22::LPS22() {
	//Nothing needed for constructor
}

bool LPS22::initialize() {
	bool status = lps.begin_I2C();
	if (!status) {
		Serial.println("Could not find a valid LPS22 sensor \n");
		exit(1);
	} else {
		lps.setDataRate(LPS22_RATE_10_HZ);
	}
	return status;
}

bool LPS22::update(double currentTime) {
	lps.getEvent(&pressure,&temp);
	_pressure = pressure.pressure;
	_temperature = temp.temperature;
	_humidity = -99.0;
	return true;
}