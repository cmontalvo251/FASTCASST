#include "Aerodynamics.h"
#include "MATLAB.h"
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "mathp.h"

using namespace std;

//Constructor
Aerodynamics::Aerodynamics()
{
  Faero.zeros(3,1,"Force Aerodynamics");
  Maero.zeros(3,1,"Moment Aerodynamics");
}

void Aerodynamics::setup(MATLAB data) {
  if (AEROTYPE == 2) {
    aeropack.CLzero = data.get(1,1);
    aeropack.CLalpha = data.get(2,1);
    aeropack.CLq = data.get(3,1);
    aeropack.CLdele = data.get(4,1);
    aeropack.CDzero = data.get(5,1);
    aeropack.CDalpha = data.get(6,1);
    aeropack.Cybeta = data.get(7,1);
    aeropack.Cydelr = data.get(8,1);
    aeropack.Cyp = data.get(9,1);
    aeropack.Cyr = data.get(10,1);
    aeropack.CLbeta = data.get(11,1);
    aeropack.CLp = data.get(12,1);
    aeropack.CLr = data.get(13,1);
    aeropack.CLdela = data.get(14,1);
    aeropack.CLdelr = data.get(15,1);
    aeropack.Cmzero = data.get(16,1);
    aeropack.Cmalpha = data.get(17,1);
    aeropack.Cmq = data.get(18,1);
    aeropack.Cmdele = data.get(19,1);
    aeropack.Cnp = data.get(20,1);
    aeropack.Cnbeta = data.get(21,1);
    aeropack.Cnr = data.get(22,1);
    aeropack.Cndela = data.get(23,1);
    aeropack.Cndelr = data.get(24,1);
    aeropack.cbar = data.get(25,1);
    aeropack.bws = data.get(26,1);
    aeropack.rho = data.get(27,1);
    aeropack.W = data.get(28,1);
  }
  aeropack.Tmax = aeropack.W*1.2; 
  aeropack.Kt = aeropack.Tmax/(800*800);
  aeropack.S = aeropack.cbar*aeropack.bws;
  ctl.setup();
}

void Aerodynamics::print_to_file(FILE* outfile) {
  ctl.print_to_file(outfile);
  fprintf(outfile,"%lf ",T);
}

double Aerodynamics::computeThrust(double motor_signal,double V) {

  //Now the idea is that if we go into a dive, the motor is probably just making more drag. 
  //Because we can't model that we will just have an if statement here that says thrust can't be negative

  double T_static = aeropack.Kt*(motor_signal-THROTTLE_MIN)*(motor_signal-THROTTLE_MIN); //REVISIT - There is no advance ratio stuff in here.
  //This is really static thrust when Vinf = 0. How would thrust decline with Vinf? According to my notes it's linear 
  //So I'm just going to move this to it's own routine. 

  //Ok so this routine computeThrust will compute Static Thrust and then from here compute Thrust in flight
  //In order to compute Thrust in flight I am assuming that at maximum throttle thrust_max = drag
  //So I need to compute total drag at the fastest flying speed so I'm going to add into the aero pack the fastest flight
  //speed
  double T_max_speed = 0.5*aeropack.rho*(aeropack.max_speed*aeropack.max_speed)*aeropack.S*aeropack.CDzero;
  
  //Ok so we basically have two data points
  // V = 0 , T = Tstatic
  // V = Vmax, T = T_max_speed
  // So we just do a simple linear interpolation method
  // ystar - y1 = m*(xstar - x1)
  // T - Tstatic = m*(V - 0)
  // T = m*V + Tstatic
  // m = (y2 - y1) / (x2 - x1)
  // m = (T_max_speed - T_static) / (aeropack.max_speed - 0)

  double T_static_full_throttle = aeropack.Kt*(THROTTLE_MAX-THROTTLE_MIN)*(THROTTLE_MAX-THROTTLE_MIN);

  ///This is the slope from thrust at full throttle when v = 0 and when v = vinput (from above)
  double m = (T_max_speed - T_static_full_throttle) / (aeropack.max_speed - 0);
  double T_full_throttle_V = m*V + T_static_full_throttle;

  //Then compute a ratio?
  double ratio = T_full_throttle_V/T_static_full_throttle;

  //Then multiply the ratio by T_static
  T = ratio*T_static;

  if (T < 0) {
    T = 0; //This just makes sure we don't create excess drag
  }

  //cout << " T_full_throttle_V = " << T_full_throttle_V << " ratio = " << ratio << endl;
  //cout << "m = " << m << " Tfull throttle(V0) = " << T_static_full_throttle << " Tmax Speed = " << T_max_speed << " Tstatic = " << T_static << " T = " << T << endl;

  //printf("Static Thrust = %lf Thrust @ Max V = %lf Thrust = %lf V = %lf \n",T_static,T_max_speed,T,V);

  return T;
}

void Aerodynamics::ForceMomentAero(double time,MATLAB state,MATLAB dxdt) {
  Faero.mult_eq(0);
  Maero.mult_eq(0);

  double z = state.get(3,1);

  double phi = state.get(4,1);
  double theta = state.get(5,1);

  double u = state.get(7,1);
  double v = state.get(8,1);
  double w = state.get(9,1);

  double zdot = dxdt.get(3,1);

  double p = state.get(10,1);
  double q = state.get(11,1);
  double r = state.get(12,1);
  
  double vinf=sqrt(u*u+v*v+w*w); //#Stream line velocity: Total velocity
  double beta=asin(v/vinf); //#Equation 21: Sideslip angle beta
  double alpha=atan(w/u); //#Equation 20: Angle of attack alpha
  double phat=(p*aeropack.bws)/(2*vinf);
  double qhat=(q*aeropack.cbar)/(2*vinf);
  double rhat=(r*aeropack.bws)/(2*vinf);

  //Call the Control Routine
  ctl.compute(time,state,dxdt,dt);

  //Extract Controls
  double dele = ctl.controls.get(1,1);
  double dela = ctl.controls.get(2,1);
  double delr = ctl.controls.get(3,1);
  double mewt = ctl.controls.get(4,1);
  T = computeThrust(mewt,vinf);

  //Aerodynamics Coefficients
  double CL = aeropack.CLzero + (aeropack.CLalpha*alpha) + (aeropack.CLq*qhat) + (aeropack.CLdele*dele); //#Equation 19: Coefficient of Lift
  double CD = aeropack.CDzero + (aeropack.CDalpha*(alpha*alpha)); //#Equation 19: Coefficient of Drag
  double CY = (aeropack.Cybeta*beta)+(aeropack.Cydelr*delr)+(aeropack.Cyp*phat)+(aeropack.Cyr*rhat);
  Faero.set(1,1,CL*sin(alpha)-(CD*cos(alpha)));
  Faero.set(2,1,CY);
  Faero.set(3,1,-CL*cos(alpha)-CD*sin(alpha)); //#Equation 18: Aerodynamic forces
  Faero.mult_eq(0.5*aeropack.rho*(vinf*vinf)*aeropack.S);

  //Append thrust
  Faero.set(1,1,Faero.get(1,1)+T);
  
  //#Total Aerodynamic Moments:
  double Cm = (aeropack.Cmzero)+(aeropack.Cmalpha*alpha)+(aeropack.Cmq*qhat)+(aeropack.Cmdele*dele);
  double Cl = (aeropack.CLbeta*beta)+(aeropack.CLp*phat)+(aeropack.CLr*rhat)+(aeropack.CLdela*dela)+(aeropack.CLdelr*delr);
  double Cn = (aeropack.Cnp*phat)+(aeropack.Cnbeta*beta)+(aeropack.Cnr*rhat)+(aeropack.Cndela*dela)+(aeropack.Cndelr*delr);
  Maero.set(1,1,aeropack.bws*Cl);
  Maero.set(2,1,aeropack.cbar*Cm);
  Maero.set(3,1,aeropack.bws*Cn);
  Maero.mult_eq(0.5*aeropack.rho*(vinf*vinf)*aeropack.S);
}






