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
  double CLzero = 0.3441029736813229;
  double CLalpha = 5.185317204810547;
  double CLq = 5.9369349224934975;
  double CLdele = 0.41181051485504044;
  double CDzero = 0.01727851900786801;
  double CDalpha = 1.0080305157535787;
  double Cybeta = -0.24095762862664397;
  double Cydelr = 0.0688450367504697;
  double Cyp = -0.027733042384004744;
  double Cyr = 0.17663419386614151;
  double CLbeta = -0.01633611041536569;
  double CLp =  -0.50160400189302223;
  double CLr = 0.09800094300447591;
  double CLdela = 0.28052396489578457;
  double CLdelr = 0.004667460118675911;
  double Cmzero = -0.07636788929236879;
  double Cmalpha= -2.1859941783855077;
  double Cmq = -24.450947269394277;
  double Cmdele = -1.1530526838451003;
  double Cnp = -0.78826431744966985;
  double Cnbeta = 0.27846397495417952;
  double Cnr = -0.33411253656820367;
  double Cndela = 0.061169396698422564;
  double Cndelr = -0.014305373822254552; //May need to run a sysID flight on rudder to actually get this one
  double cbar=0.2286;// #Aerodynamic chord length: m
  double bws=1.4859;  //#Wing span: m
  double S = cbar*bws;
  double mass = 1.2364; //This has got to be Newtons - Maxwell says yes since the a/c is like 2.72 lbf = 2.72*4.44 ~= 12.0
  double Tmax = mass*GEARTH;//these are set in the setup() routine
  double max_speed = 30; //this is just from anectodal evidence and looking at plots
  double Rrotor = (9.5/12.0)/(2*3.28); //9.5 inch props to meters
  double AREA = PI*pow(Rrotor,2.0);  //m^2
  double Voltage = 3.7*4; //4 Cell
  double KV = 950.0;
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
  void ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB pwm_out,environment env); 
  //Constructor
  forces();
};

#endif
