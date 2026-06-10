#ifndef RK4_H
#define RK4_H
///This routine will give you everything you need to integrate a runge
//kutta 4 routine

//This routine requires a dynamic model. You can see an example in FASTSim/Dynamics.*

#include <MATLAB/MATLAB.h>

class RK4 {
 private:
  //private vars and functions
  MATLAB k1,k2,k3,k4,phi;
  double TIMESTEP;
 public:
  int NUMVARS;
  MATLAB State,StateDel,k;
  //public vars and functions
  void init(int NUMVARS,double dt);
  void set_ICs(MATLAB x0);
  void integrate(int i);
  //Contructor
  RK4();

};

#endif
