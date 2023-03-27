#include "PTH.h"

//Constructor
PTH::PTH() {
}


void PTH::init(int sensor_type_in) {
  sensor_type = sensor_type_in;
  #ifndef DESKTOP
  if (sensor_type == 0) {
    printstdout("Selected: No PTH Sensor \n");
    IPTH = 0;
  } else {
    IPTH = 1;
  }
  if (sensor_type == 1) {
    printstdout("Selected: MS5611 \n");
    pth_sensor = new MS5611();
  }
  if (sensor_type == 2) {
    printstdout("Selected: Adafruit_MPL115A2 \n");
    pth_sensor = new Adafruit_MPL115A2();
  }
  if (sensor_type == 3) {
    printstdout("Selected: BME280 \n");
    pth_sensor = new BME280();
  }
  if (IPTH) {
    pth_sensor->initialize();
  }
  #else
  printstdout("Running Fictitious PTH Sensor on Desktop \n");
  #endif
}

void PTH::print() {
  printstdoutdbl(pressure);
  printstdout(" ");
  printstdoutdbl(temperature);
  printstdout(" ");
  printstdoutdbl(altitude);
  printstdout(" ");
  printstdoutdbl(humidity);
}

void PTH::poll(double currentTime) {
  #ifndef DESKTOP
  //Update sensor
  bool status = pth_sensor->update(currentTime);
  //Get results
  if (status) {
    pressure = pth_sensor->read_pressure();
    temperature = pth_sensor->read_temperature();
    humidity = pth_sensor->read_humidity();
    if (CALIBRATE < 5) {
      printstdout("CALIBRATING BAROMETER !!! Pressure = ");
      printstdoutdbl(pressure/0.01);
      CALIBRATE+=1;
      pressure0 += pressure/0.01;
    } else {
      if (CALIBRATE_FLAG) {
        pressure0 /= 5;
        printstdout("BAROMETER CALIBRATED !!! Presure0 = ");
        printstdoutdbl(pressure0);
        CALIBRATE_FLAG = 0;
      }
    }
  }
  #else
  //This below runs on desktop
  //Using fictitious pressure and temperature
  pressure = ConvertZ2Pressure(Z); //This function is in mathp btw.
  if (CALIBRATE_FLAG == 1) {
    pressure0 = ConvertZ2Pressure(0)/0.01; //get sea-level pressure 
    CALIBRATE_FLAG = 0;
  }
  temperature = 25; //nominal for now
  humidity = 0.0; //0 for now
  #endif

  //Convert Pressure to Altitude but only if we have a valid measurement
  if (pressure != -99) {
    altitude = ConvertPressure2Z(pressure,pressure0);
    Z = -altitude;
  }
}

void PTH::SendZ(double Zin) {
  Z = Zin;
}
