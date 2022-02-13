/* Forces Template 2021

This forces file is a template for a fictitious portalcube
with thrusters and a simple  model. The Dynamics.cpp module
will call a few candidate functions. If you make your own 
file with header and cpp file you must conform to the following
functions otherwise the software will completely break.

*/

#include "forces.h"

//Constructor
forces::forces() {
  //The constructor must create these 3x1 vectors
  FB.zeros(3,1,"Force in Body Frame");
  MB.zeros(3,1,"Moment in Body Frame");
}

void forces::ForceMoment(double time,MATLAB state,MATLAB statedot,int pwm_array[],environment env) {
  //The only thing this function needs to do is populate FB and MB. 
  //You can do whatever you want in here but you must create those two vectors.
  FB.mult_eq(0); //Zero these out just to make sure something is in here
  MB.mult_eq(0);

  //Extract Actuator Values
  //Remember that control is in PWM (us)
  double throttleUS = pwm_array[0];
  double aileronUS = pwm_array[1];
  double elevatorUS = pwm_array[2];
  double rudderUS = pwm_array[3];
	
  //Convert throttle signals to thruster value
  double TMAX = 1000;
  double TORQUEMAX = 10.0;
  double max_slope = (STICK_MAX-STICK_MIN);
  double mid_slope = (STICK_MAX-STICK_MID);
  double Zthrust = -(throttleUS - STICK_MIN)/max_slope*TMAX;
  double Lthrust = (aileronUS - STICK_MID)/mid_slope*TORQUEMAX;
  double Mthrust = (elevatorUS - STICK_MID)/mid_slope*TORQUEMAX;
  double Nthrust = (rudderUS - STICK_MID)/mid_slope*TORQUEMAX;
	
  // Parameters
  double S = 0.1; //m^2
  double c = 1.0; //mean chord
  double CD = 1.0; //Linear Drag Coefficient
  double CM = 3.0; //Rotational Drag Coefficient
	
  //Extract States
  double u = state.get(8,1);
  double v = state.get(9,1);
  double w = state.get(10,1);
  double p = state.get(11,1);
  double q = state.get(12,1);
  double r = state.get(13,1);
	
  //Total Velocity
  double V = sqrt(u*u + v*v + w*w);

  //Non-Dimensional Angular Velocity
  double pbar=0,qbar=0,rbar=0;
  if (abs(V)>0.001) {
    pbar = p*c/(2*V);
    qbar = q*c/(2*V);
    rbar = r*c/(2*V);
  } else {
    V = 1.0;
    pbar = p*c;
    qbar = q*c;
    rbar = r*c;
  }
	
  //Dynamic Pressure
  double qinf = 0.5*RHOSLSI*V*S;
  //printf("qinf = %lf pbar = %lf \n",qinf,pbar);
  //printf("V = %lf \n",V);
	
  //Forces
  FB.plus_eq(qinf);
  FB.mult_eq1(1,1,-u*CD);
  FB.mult_eq1(2,1,-v*CD);
  FB.mult_eq1(3,1,-w*CD); 
  //Add Thrust
  FB.plus_eq1(3,1,Zthrust);
	
  //Moments
  MB.plus_eq(qinf);
  MB.mult_eq1(1,1,-V*pbar*c*CM);
  MB.mult_eq1(2,1,-V*qbar*c*CM);
  MB.mult_eq1(3,1,-V*rbar*c*CM);
  //Add Thruster
  MB.plus_eq1(1,1,Lthrust);
  MB.plus_eq1(2,1,Mthrust);
  MB.plus_eq1(3,1,Nthrust);
	
  //FB.disp();
  //MB.disp();
  //PAUSE();
}


