#ifndef VEHICLE_H
#define VEHICLE_H

//Because C++ does not have any native matrix multiplication
//I create my own. They are included here.
#include "MATLAB.h"
#include "Rotation3.h"
#include <fstream>
#include "mathp.h"
#include "timer.h"
#include "Aerodynamics.h"

class Vehicle {
  friend class Connection;
 private:
  double mass;
  double zint=0,theta_int=0,roll_int=0,yaw_int=0;
  char input_file_[256];
  MATLAB simdata,massdata,icdata,stateinitial,k1,k2,k3,k4;
  MATLAB aerodata,state,rkstate,phi,F,M,Fgrav,Fbody,ForceConnection,MomentConnection;
  MATLAB Iinv,I,pqr,uvw,uvwdot,xyzdot,ptp,pqrdot,I_pqr,LMN,ptpdot,Kuvw_pqr,pqrskew_I_pqr;
  Aerodynamics aero;
 public:
  //Constructor
  Vehicle();
  void init(char*);
  int ok;
  int ImportFiles();
  int ImportFile(char*,MATLAB*,char*,int);
  void Integrate();
  void Derivatives(MATLAB,double,MATLAB);
  void ForceandMoment(MATLAB,MATLAB,double);
  void setup(double);
  void stepstate(double);
  Rotation3 ine2bod123;
};

#endif
