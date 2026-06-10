#include "Autopilot.h"
#include "links/mathp.h"
#include "Navio2/PWM.h"

void Autopilot::loop(float rollrc, float pitchrc, float throttle, float elapsedTime,float oh_shit){

  float roll_command,pitch_command,u_command;

  //CONVERT PILOT INPUTS TO COMMANDS
  if (oh_shit > 1800) {
    ////Holy crap
    roll_command = DEG2RAD*15;
    pitch_command = DEG2RAD*45;
    u_command = -99;
  } else {
    roll_command = DEG2RAD*(45.0/515.0)*(rollrc-SERVO_MID); //Convert from microsecond pulse to radian
    pitch_command = -DEG2RAD*(45.0/515.0)*(pitchrc-SERVO_MID); //Convert from microsecond pulse to radian
    u_command = (VELOCITY_MAX/1024.0)*(throttle-THROTTLE_MIN); //Convert from microsecond pusle to m/s
  }
  
  //printf("u_command: %f, ",u_command);
  //printf("pitch_command: %f, ", pitch_command);
  //printf("roll_command: %f, ", roll_command);
  //printf("pitchrc: %f, ", pitchrc);
  //printf("rollrc: %f, ", rollrc);
  //float u = windspeed_filtered;
  //printf("u: %f ",pitots.windspeed_average);

  //Run the control loop from Aircraft++
  ctl.InnerLoop(roll_command,pitch_command,u_command,orientation.roll*DEG2RAD,orientation.pitch*DEG2RAD,satellites.speed,orientation.roll_rate*DEG2RAD,orientation.pitch_rate*DEG2RAD,orientation.yaw_rate*DEG2RAD,elapsedTime);
  
  //Controls is elevator,aileron,rudder,thrust(us)
  float dele = ctl.controls.get(1,1); //radians
  float dela = ctl.controls.get(2,1); //radians
  float delr = ctl.controls.get(3,1); //radians
  float mewt = ctl.controls.get(4,1); //us

  //printf("Motor Signal: %lf \n",mewt);

  float aileron_signal = ctl.RAD2PWM(dela,SERVO_MIN,SERVO_MAX,SERVO_MID,MAX_ANGLE_AILERON);
  float elevator_signal = ctl.RAD2PWM(dele,SERVO_MIN,SERVO_MAX,SERVO_MID,MAX_ANGLE_ELEVATOR);
  float rudder_signal = ctl.RAD2PWM(delr,RUDDER_SERVO_MIN,RUDDER_SERVO_MAX,RUDDER_SERVO_MID,MAX_ANGLE_RUDDER);

  signals[0] = mewt; //Send to motors
  signals[1] = aileron_signal;
  signals[2] = elevator_signal;
  signals[3] = rudder_signal;
}

void Autopilot::reset() {
  //Reset the integral gains to remove antiwindup
  ctl.AntiWindup();

  //Compute ADC Averages
  pitots.computeADCAverages();
}

void Autopilot::setup(int THREADQ) {

  MS5611_Thread = THREADQ;
  barotemp.THREAD = THREADQ;

  setupMS5611(&barotemp);

  printf("Pressure Temperature (after setup) = %lf %lf \n",barotemp.Pressure,barotemp.Temperature);
  
  int channels[2];
  channels[0] = 4;
  channels[1] = 5;
  pitots.setup(channels,2,barotemp.T_avg);
}

void Autopilot::update(float currentTime,float elapsedTime,int FILEOPEN) {

  //////////////////POLLING IMU AS QUICKLY AS POSSIBLE////////////
  orientation.loop(elapsedTime,0.45); //second number is the filter number
  #ifdef PRINTSEVERYWHERE
  printf("IMU... \n");
  #endif
  ///////////////////////////////////////////////////////////////////

  ////////////////////POLL PITOT AND BAROMETER/////////////////
  if ((currentTime - pitots.lastTime)/1000000.0 > PITOTPERIOD) {
    pitots.lastTime = currentTime;
    pitots.poll(FILEOPEN);
  }
  ////////////////////////////////////////////////////

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
