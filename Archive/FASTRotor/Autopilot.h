#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <Common/IMU.h>
#include <Common/GPS.h>
#include <Common/BarometerTemperature.h>
#include <MATLAB/MATLAB.h>
#include <mathp.h>
#include <Navio2/PWM.h>
#include <Common/FASTPWM.h>

#define NUM_MOTORS 5
#define VELOCITY_MAX 20.0 //This is in m/s, and it is a guess and will need to be verified during flight test.
#define VELOCITYCONTROL 0 //Set this to zero if you want velocity to be manual

class Autopilot {
 private:
  double perrorIntegral=0,qerrorIntegral=0;
 public:
  IMU orientation;
  double signals[NUM_MOTORS];
  int MS5611_Thread = 0;
  BarometerTemperature barotemp;
  void loop(double rollrc, double pitchrc, double throttle, double yawrc,double extra_servo,double elapsedTime, double oh_shit);
  void reset();
  GPS satellites;
  void setup(int);
  void update(double,double,int);
};

#endif
