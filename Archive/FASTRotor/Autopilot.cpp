#include "Autopilot.h"

void Autopilot::loop(double rollrc, double pitchrc, double throttle, double yawrc,double extra_servo,double elapsedTime,double oh_shit){

    ///PID LOOP

    //================make commands===============
    //i need to take the pwm signals and make them angle commands
    //These commands only need to be generated when you poll the receiver
    double roll_command = (50.0/500.0)*(double(rollrc)-1500.0); //in degrees 
    double pitch_command = -(50.0/500.0)*(double(pitchrc)-1500.0); //in degrees
    double yaw_rx = -(135.0/500.0)*(double(yawrc)-1500.0); //this is really a yawRate command in deg/s

    //printf("%lf %lf %lf \n",roll_command,pitch_command,yaw_rx);

    //PID GAINS
    double kp = 3.0;
    double kd = 0.9;
    double ki = 0.2;
    double kyaw = 10.0;
    //kyaw at 1.0 did not work so trying 3.0 2/18/2020
    //3.0 did not work either but 10.0 did.

    //Using Control Architecture from KyrellGod
    //Compute outer loop commands
    //2/26/2019 - twichiness so lowered from 4.5
    double phidot_command = kp*(roll_command - orientation.roll);
    double thetadot_command = kp*(pitch_command - orientation.pitch);

    //Constrain by 250 deg/s
    phidot_command = CONSTRAIN(phidot_command,-250.0,250.0);
    thetadot_command = CONSTRAIN(thetadot_command,-250.0,250.0);

    //This is direct from the user
    double psidot_command = yaw_rx;

    //These ptpdot commands are Euler angle derivatives and need to be converted to body frame angular rates
    double cos_theta = cos(orientation.pitch*DEG2RAD);
    double sin_theta = sin(orientation.pitch*DEG2RAD);
    double cos_phi = cos(orientation.roll*DEG2RAD);
    double sin_phi = sin(orientation.roll*DEG2RAD);
    double p_command = phidot_command - sin_theta*psidot_command;
    double q_command = cos_phi*thetadot_command + sin_phi*cos_theta*psidot_command;
    double r_command = -sin_phi*thetadot_command + cos_phi*cos_theta*psidot_command;

    //We then compute the inner loop commands which is a PI controller except for yaw
    double perror = (p_command - orientation.roll_rate);
    double qerror = (q_command - orientation.pitch_rate);
    double rerror = (r_command - orientation.yaw_rate);

    //Need to compute integral error + anti-Windup
    perrorIntegral += perror*elapsedTime;
    perrorIntegral = CONSTRAIN(perrorIntegral,-50.0,50.0);
    qerrorIntegral += qerror*elapsedTime;
    qerrorIntegral = CONSTRAIN(qerrorIntegral,-50.0,50.0);

    //Then compute your commands note that yaw_out is just proportional
    double d_roll = kd*perror + ki*perrorIntegral;
    double d_pitch = kd*qerror + ki*qerrorIntegral;
    double d_yaw = kyaw*rerror;

    //Constrain individual commands
    d_roll = CONSTRAIN(d_roll,-500,500);
    d_pitch = CONSTRAIN(d_pitch,-500,500);
    d_yaw = CONSTRAIN(d_yaw,-500,500);

    signals[0] = throttle + d_roll - d_pitch + d_yaw + IDLE - THROTTLE_MIN; //back left
    signals[1] = throttle - d_roll - d_pitch - d_yaw + IDLE - THROTTLE_MIN; //back right
    signals[2] = throttle - d_roll + d_pitch + d_yaw + IDLE - THROTTLE_MIN; //front right
    signals[3] = throttle + d_roll + d_pitch - d_yaw + IDLE - THROTTLE_MIN; //front left
}

void Autopilot::setup(int THREADQ) {

  MS5611_Thread = THREADQ;
  barotemp.THREAD = THREADQ;

  setupMS5611(&barotemp);

  printf("Pressure Temperature (after setup) = %lf %lf \n",barotemp.Pressure,barotemp.Temperature);
}

void Autopilot::reset() {
  perrorIntegral = 0;
  qerrorIntegral = 0;
}

void Autopilot::update(double currentTime,double elapsedTime,int FILEOPEN) {

  //////////////////POLLING IMU AS QUICKLY AS POSSIBLE////////////
  //Filter value is 0.45 for meta
  //Changed 0.55 on quadcopter
  orientation.loop(elapsedTime,0.45); //second number is the filter number
  #ifdef PRINTSEVERYWHERE
  printf("IMU... \n");
  #endif
  ///////////////////////////////////////////////////////////////////

  /*  //////////////////POLL BAROMETER////////////////////
  if (((currentTime - barotemp.lastTime)/1000000.0 > 1.0) && (!barotemp.THREAD)) { //1.0 is hardcoded because the barometer is kind of fickle
    #ifdef PRINTSEVERYWHERE
    printf("Polling Barometer \n");
    #endif
    barotemp.lastTime = currentTime;
    barotemp.refreshP();
    barotemp.wait();
    barotemp.readP();
    barotemp.process();
    } //otherwise this is handled in a separate thread*/
  
  ///////////////////POLL GPS/////////////////////////////////
  /*if ((currentTime - satellites.lastTime)/1000000.0 > GPSPERIOD) {
    satellites.poll(currentTime,FILEOPEN);
    #ifdef PRINTSEVERYWHERE
    printf("GPS \n");
    #endif 
    }*/
  //////////////////////////////////////////////////////////////
  
}
