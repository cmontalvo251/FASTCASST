#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <Common/IMU.h>

//Which feedback control law are we using
//#define USEKYRELLPID
//#define USEMONTALVOPID
#define USERECONFIG

//If you're flying BDE uncomment this
#define BDE

#include "links/MATLAB.h"
#include "links/Motor.h"
#include "links/Propulsion.h"
#include "links/mathp.h"
#include <Common/FASTPWM.h>

class Autopilot {
 public:
  float kproll,kdroll,d_roll=0,kppitch,kdpitch,d_pitch=0,d_yaw=0,kipitch,kiroll;
  float yawLock = 0;
  double roll_command,pitch_command,yaw_rx,thrust_desired,mass,pusherout,Ixx,Iyy,Izz;
  double pitcherrorIntegral,rollerrorIntegral,perrorIntegral,qerrorIntegral,rx,ry,rz,rxpusher;
  double Tdatapt,pwm_datapt,omegaRPMdatapt,R;
  double MOTORSOUT[8];
  double MOTORSOUT_[8];
  double inSIG_[8];
  double inSIG[8];
  MATLAB I,datapts,rxyz;
  IMU orientation;
  Propulsion propel;
  void loop(double,double,double,double,double,double,float);
  void update(float,float,int);
  void reset();
  //Contructor
  Autopilot();
};

#endif
