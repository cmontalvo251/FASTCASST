#include "Autopilot.h"
#include "links/mathp.h"
#include "Navio2/PWM.h"
#include <Common/FASTPWM.h>

void Autopilot::loop(float rollrc, float pitchrc, float throttle, float yawrc,float elapsedTime,float mode){

  float roll_command,pitch_command,u_command;

  //CONVERT PILOT INPUTS TO COMMANDS
  double yaw_command = 0;
  float yaw_error = orientation.yaw - yaw_command;
  signals[0] = yaw_error*20 + throttle;
  signals[1] = -yaw_error*20 + throttle;
  signals[2] = mode;
}

void Autopilot::setup(int THREADQ) {

  MS5611_Thread = THREADQ;
  barotemp.THREAD = THREADQ;

  setupMS5611(&barotemp);

  printf("Pressure Temperature (after setup) = %lf %lf \n",barotemp.Pressure,barotemp.Temperature);
}

void Autopilot::update(float currentTime,float elapsedTime,int FILEOPEN) {

  //////////////////POLLING IMU AS QUICKLY AS POSSIBLE////////////
  //Filter value is 0.45 for meta
  orientation.loop(elapsedTime,0.45); //second number is the filter number
  #ifdef PRINTSEVERYWHERE
  printf("IMU... \n");
  #endif
  ///////////////////////////////////////////////////////////////////

  //////////////////POLL BAROMETER////////////////////
  if (((currentTime - barotemp.lastTime)/1000000.0 > 1.0) && (!barotemp.THREAD)) { //1.0 is hardcoded because the barometer is kind of fickle
    #ifdef PRINTSEVERYWHERE
    printf("Polling Barometer \n");
    #endif
    barotemp.lastTime = currentTime;
    barotemp.refreshP();
    barotemp.wait();
    barotemp.readP();
    barotemp.process();
  } //otherwise this is handled in a separate thread
  
  ///////////////////POLL GPS/////////////////////////////////
  if ((currentTime - satellites.lastTime)/1000000.0 > GPSPERIOD) {
    satellites.poll(currentTime,FILEOPEN);
    #ifdef PRINTSEVERYWHERE
    printf("GPS \n");
    #endif 
  }
  //////////////////////////////////////////////////////////////
  
}
