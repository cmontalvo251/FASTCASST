#include "BME280.h"

BME280::BME280() {
	//Nothing needed for constructor
}

bool BME280::initialize() {
	bool status = bme.begin();
	if (!status) {
		Serial.println("Could not find a valid BME280 sensor \n");
	}
	return status;
}

bool BME280::update(double currentTime) {
	_pressure = bme.readPressure()/100.0;
	_temperature = bme.readTemperature();
	_humidity = bme.readHumidity();
	return true;
}