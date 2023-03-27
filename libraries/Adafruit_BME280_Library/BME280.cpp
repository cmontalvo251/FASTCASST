#include "BME280.h"

BME280::BME280() {
	//Nothing needed for constructor
}

bool BME280::initialize() {
	bool status = bme.begin();
	return status;
}

bool BME280::update(double currentTime) {
	_pressure = bme.readPressure();
	_temperature = bme.readTemperature();
	_humidity = bme.readHumidity();
}