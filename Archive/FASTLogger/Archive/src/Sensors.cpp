#include "Sensors.h"

void Sensors::update(double currentTime,double elapsedTime,int FILEOPEN) {
  //////////////////POLLING IMU AS QUICKLY AS POSSIBLE////////////
  //Filter value is 0.45 for meta
  orientation.loop(elapsedTime,0.45); //second number is the filter number
  #ifdef PRINTSEVERYWHERE
  printf("IMU... \n");
  #endif

  ///////////////////POLL GPS/////////////////////////////////
  if ((currentTime - satellites.lastTime)/1000000.0 > gpstime) {
    satellites.poll(currentTime,FILEOPEN);
    #ifdef PRINTSEVERYWHERE
    printf("GPS \n");
    #endif 
  }
  //////////////////////////////////////////////////////////////
}
