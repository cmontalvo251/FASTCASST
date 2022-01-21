///BaroTemp Class
#include "BaroTemp.h"

BaroTemp::BaroTemp() {
  #ifndef DESKTOP
  barometer.initialize();
  #else
  printf("Running Fictitious Barometer on Desktop \n");
  #endif
}

void BaroTemp::poll(double currentTime) {
  #ifndef DESKTOP
  if (PHASE == 0) {
    if ((currentTime - updatetime) > LOOP_TIME) {
      barometer.refreshPressure();
      PHASE = 1;
      updatetime = currentTime;
    }
  }
  if (PHASE == 1) {
    if ((currentTime - updatetime) > SLEEP_TIME) {
      barometer.readPressure();
      barometer.refreshTemperature();
      PHASE = 2;
      updatetime = currentTime;
    }
  }
  if (PHASE == 2) {
    if ((currentTime - updatetime) > SLEEP_TIME) {
      barometer.readTemperature();
      barometer.calculatePressureAndTemperature();
      temperature = barometer.getTemperature();
      pressure = barometer.getPressure();
      updatetime = currentTime;
      PHASE = 0;
    }
  }
  #else
  //Using fictitious pressure and temperature
  ConvertZ2Pressure();
  temperature = 25.5; // just gonna have to hard code this
  #endif
  //Convert Pressure to Altitude but only if we have a valid measurement
  if (pressure != -99) {
    ConvertPressure2Altitude();
  }
}

void BaroTemp::SendZ(double Zin) {
  Z = Zin;  
}

void BaroTemp::ConvertPressure2Altitude() {
  pressure0 = 1010; //Going to have to Hardcode this.
  double pascals = pressure/0.01;
  altitude = (1.0-pow((pascals/101325.0),1.0/5.25588))/(2.2557*pow(10,-5.0));
  //printf("\n Pressure = %lf ",pressure);
  //printf("Pressure0 = %lf ",pressure0);
  //printf("Altitude = %lf ",altitude);
  //printf("Pascals = %lf \n",pascals);
  Z = -altitude;
}

void BaroTemp::ConvertZ2Pressure() {
  //printf("Z = %lf \n",Z);
  altitude = -Z;
  //printf("Altitude = %lf \n",altitude);
  if (pressure0 == -99) {
    pressure0 = pressure;
  }
  double pascals = 101325.0*pow((1.0-2.25577*pow(10,(-5.0))*altitude),5.25588);
  //printf("Pascals = %lf \n",pascals);
  pressure = pascals*0.01;
}
