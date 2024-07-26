#ifndef AUTOPILOT_H
#define AUTOPILOT_H

//Include the controller routine from Aircraft++
#include <Common/IMU.h>
#include <Common/GPS.h>
#include <Common/BarometerTemperature.h>

#define VELOCITY_MAX 20.0 //This is in m/s, and it is a guess and will need to be verified during flight test.
#define VELOCITYCONTROL 0 //Set this to zero if you want velocity to be manual

class Autopilot {
 public:
  IMU orientation;
  double signals[3];
  int MS5611_Thread = 0;
  BarometerTemperature barotemp;
  void loop(float rollrc, float pitchrc, float throttle, float yawrc,float elapsedTime, float);
  void reset();
  GPS satellites;
  void setup(int);
  void update(float,float,int);
};

#endif
