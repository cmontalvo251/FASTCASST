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
  double CLzero = 0.3435714503122909;
  double CLalpha = 5.370313924026151;
  double CLq = 9.928984189123296;
  double CLdele = 0.37233690709212364;
  double CDzero = 0.017698385325262064;
  double CDalpha = 1.0738514087162259;
  double Cybeta = -0.31871229819026886;
  double Cydelr = 0.0910606566257911;
  double Cyp = -0.04508856682769727;
  double Cyr = 0.3541247757669655;
  double CLbeta = -0.026559358182522405;
  double CLp = -0.6468277159429427;
  double CLr = 0.11540326055865319;
  double CLdela = -0.29259555030166484;
  double CLdelr = 0.007588388052149259;
  double Cmzero = -0.38931523333313595;
  double Cmalpha= -10.033814972698428;
  double Cmq = -37.23369070921236;
  double Cmdele = -0.889654556768791;
  double Cnp = -0.017897227495871214;
  double Cnbeta = 0.17706238788348275;
  double Cnr = -0.20116058286851857;
  double Cndela = 0.216;
  double Cndelr = -0.21757856007932388; //This is coming from get coefficient. May need to run a sysID flight on rudder to actually get this one
  double cbar=0.2286;// #Aerodynamic chord length: m
  double bws=1.3716;  //#Wing span: m
  double S = cbar*bws;
  double mass = 27.801385095; //This has got to be Newtons - Maxwell says yes since the a/c is like 2.72 lbf = 2.72*4.44 ~= 12.0
  double Tmax = mass*GEARTH;//these are set in the setup() routine
  double max_speed = 30; //leaving this alone for now
  double Rrotor = (13.0/12.0)/(2*3.28); //13.0 inch props to meters
  double AREA = PI*pow(Rrotor,2.0);  //m^2
  double Voltage = 3.7*6; //6 Cell
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
