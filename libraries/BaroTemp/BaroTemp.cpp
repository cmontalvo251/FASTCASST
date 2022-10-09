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
      if (CALIBRATE < 5) {
	       printf("CALIBRATING BAROMETER !!! Pressure = %lf \n",pressure/0.01);
       	 CALIBRATE+=1;
	       pressure0 += pressure/0.01; //convert mbars to pascals
      } else {
        if (CALIBRATE_FLAG) {
          pressure0 /= 5;
          printf("BAROMETER CALIBRATED !!! Pressure0 = %lf \n",pressure0);
          CALIBRATE_FLAG = 0;
        }
      }
    }
  }
  #else
  //Using fictitious pressure and temperature
  pressure = ConvertZ2Pressure(Z); //This function is in mathp btw.
  if (CALIBRATE_FLAG == 1) {
    pressure0 = ConvertZ2Pressure(0)/0.01; //get sea-level pressure using hardcoded equation
    CALIBRATE_FLAG = 0;
  }
  temperature = NOMINALTEMP; // just gonna have to hard code this
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
  //pressure0 = 1010; //Going to have to Hardcode this.MOVED TO CALIBRATION
  double pascals = pressure/0.01;
  altitude = (1.0-pow((pascals/pressure0),1.0/5.25588))/(2.2557*pow(10,-5.0));
  //printf("\n Pressure = %lf ",pressure);
  //printf("Pressure0 = %lf ",pressure0);
  //printf("Altitude = %lf ",altitude);
  //printf("Pascals = %lf \n",pascals);
  Z = -altitude;
}
