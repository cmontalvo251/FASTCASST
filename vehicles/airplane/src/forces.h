#ifndef FORCES_H
#define FORCES_H

/* Forces Template 2021

This forces file is a template for a fictitious portalcube
with thrusters and a simple aero model. The Dynamics.cpp module
will call a few candidate functions. If you make your own aero
file with header and cpp file you must conform to the following
functions otherwise the software will completely break.

*/

#include <Environment/environment.h>
#include <MATLAB/MATLAB.h> //This is needed for variable length arrays as inputs
#include <Mathp/mathp.h> //this is for density at sea-level
#include <Timer/timer.h> //for pause function
#include <RCIO/RCIO.h> //for stick min values

struct Aero_Pack {
  double CLzero = 0.34995;
  double CLalpha = 4.8872;
  double CLq = 7.993598;
  double CLdele = 0.41181051485504044;
  double CDzero = 0.015342;
  double CDalpha = 0.713431;
  double Cybeta = -0.299134;
  double Cydelr = 0.0688450367504697;
  double Cyp = -0.0423188;
  double Cyr = 2.04408;
  double CLbeta = -0.0249279;
  double CLp =  -0.646828;
  double CLr = 0.257828;
  double CLdela = 0.28052396489578457;
  double CLdelr = 0.004667460118675911;
  double Cmzero = -0.074721;
  double Cmalpha= -4.104604;
  double Cmq = -44.691481;
  double Cmdele = -1.1530526838451003;
  double Cnp = -0.01965;
  double Cnbeta = 1.022042;
  double Cnr = -1.167828;
  double Cndela = 0.061169396698422564;
  double Cndelr = -0.014305373822254552; //May need to run a sysID flight on rudder to actually get this one
  double cbar=0.2286;// #Aerodynamic chord length: m
  double bws=1.372;  //#Wing span: m
  double S = cbar*bws;
  double mass = 27.8; //This has got to be Newtons - Maxwell says yes since the a/c is like 2.72 lbf = 2.72*4.44 ~= 12.0
  double Tmax = mass*GEARTH;//these are set in the setup() routine
  double max_speed = 20; //this is just from anectodal evidence and looking at plots
  double Rrotor = (13./12.0)/(2*3.28); //9.5 inch props to meters
  double AREA = PI*pow(Rrotor,2.0);  //m^2
  double Voltage = 3.7*6; //4 Cell
  double KV = 800.0;
  double omegaRPM = KV*Voltage;
  double omegarads = omegaRPM*2.0*PI/180.0;
  double spin_slope = omegarads/(STICK_MAX-STICK_MIN);
  double ct = Tmax/(0.5*RHOSLSI*AREA*pow(Rrotor*omegarads,2.0)); //.0335
  double cq = pow(ct,3.0/2.0)/sqrt(2.0); 
};

class forces {
 private:
  //You can put any private functions or vars in here that you like but it 
  //must adhere to the standards below
  Aero_Pack aeropack;
 public:
  //These are 3x1 MATLAB vectors that must be in units of Newtons
  //and in the body frame
  MATLAB FB,MB; 
  //There the inputs are the current time, the current MATLAB state vector,
  //the current derivative state vector and the actuator values.
  //The actuator is in units of PWM signal b/c this is an Radio Controlled sim
  //TAERA1A2 (Throttle, Aileron, Elevator, Rudder, Aux1, Aux2)
  //time is in seconds
  //state is a 13x1 using quaternions and using standard aerospace convention
  //statedot is the derivatives of the state vector
  void ForceMoment(double time,MATLAB state,MATLAB statedot,int pwm_array[],environment env); 
  //Constructor
  forces();
};

#endif
