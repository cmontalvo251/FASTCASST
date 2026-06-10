#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "MATLAB.h"
#include "Rotation3.h"
#include <fstream>
#include "mathp.h"
#include "timer.h"

#define SERVO_MIN 992. /*ms*/
#define SERVO_MAX 2016. /*mS*/
#define SERVO_MID 1504. /*ms*/
#define RUDDER_SERVO_MAX 2009.
#define RUDDER_SERVO_MIN 985.
#define RUDDER_SERVO_MID 1497.
#define THROTTLE_MIN 992.
#define THROTTLE_MAX 2016.
#define IDLE 1200. /*mS*/

#define MAX_ANGLE_ELEVATOR 20.0*PI/180.0
#define MAX_ANGLE_AILERON 23.0*PI/180.0
#define MAX_ANGLE_RUDDER 25.0*PI/180.0
#define MAX_ANGLE_PITCH 60.0*PI/180.0
#define MAX_ANGLE_ROLL 60.0*PI/180.0

class Controller {
 private:
  double cost = 0;
  double uint = 0;
  double dele,dela,delr,mewt=SERVO_MIN,param;
  FILE* infile;
  double t0=0,tf=0,dele0=0,delef=0,dela0=0,delaf=0,delr0=0,delrf=0,mewt0=0,mewtf=0;
  double measured_state0=0,measured_statef=0;
 public:
  int CONTROLTYPE;
  MATLAB controls; //de,da,dr,mewt,T
  void print_to_file(FILE*);
  void printcost();
  void compute(double,MATLAB,MATLAB,double);
  void InnerLoopOuterLoop(MATLAB,MATLAB,double);
  void InnerLoop(double,double,double,double,double,double,double,double,double,double);
  void SaturationBlock();
  double RAD2PWM(double,double,double,double,double);
  double PWM2RAD(double,double,double,double,double);
  void AntiWindup();
  void setup();
  void read2Rows();
  void read1Row();

  //Constructor
  Controller();
};

#endif
