#ifndef FORCES_H
#define FORCES_H

#include <Environment/environment.h>
#include <MATLAB/MATLAB.h> //This is needed for variable length arrays as inputs
#include <Mathp/mathp.h> //this is for density at sea-level
#include <Timer/timer.h> //for pause function
#include <RCIO/RCIO.h> //for stick min values

class forces {
 private:
  //You can put any private functions or vars in here that you like but it 
  //must adhere to the standards below
  double Rrotor,cq,ct,kt,rx,ry,rz,AREA,spin_slope;
  MATLAB thrust_motors,torque_motors;
  void compute_thrust_and_torque(MATLAB pwm_out);
  int NUMMOTORS = 4;
 public:
  double length = 1; //This is the length of the quadcopter. It is used in the Dryden model
  //These are 3x1 MATLAB vectors that must be in units of Newtons
  //and in the body frame
  MATLAB FB,MB; 
  //There the inputs are the current time, the current MATLAB state vector,
  //the current derivative state vector and the control vector.
  //The control vector is in units of PWM signal b/c this is an RC A/C sim
  //TAERA1A2 (Throttle, Aileron, Elevator, Rudder, Aux1, Aux2)
  //time is in seconds
  //state is a 13x1 using quaternions and using standard aerospace convention
  //statedot is the derivatives of the state vector
  void ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB pwm_out,const environment& env);
  //Constructor
  forces();
};

#endif
