/* Forces Template 2021

This forces file is a template for a **CAR** \\ Modified to reflect 'Car' requirements
with thrusters and a simple aero model. The Dynamics.cpp module
will call a few candidate functions. If you make your own aero
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
  //The only thing this function needs to do is populate FAEROB and MAEROB. 
  //You can do whatever you want in here but you must create those two vectors.
  FB.mult_eq(0); //Zero these out just to make sure something is in here
  MB.mult_eq(0);
  
  //Friction Parameters
  double d = 0.13335; //(m) - From wheel to center
  double force;
  double Vmax = 6.0; //(m/s) Need to find the max speed of the tank
  
  //Extract Actuator Values
  //Remember that control is in PWM (us)
  double motor = pwm_array[0];
  //printf("MOTOR = %lf \n",motor);
  double steering = pwm_array[1]; 

  //Extract States
  double x = state.get(1,1);
  double u = state.get(8,1);
  double v = state.get(9,1);
  double w = state.get(10,1);
  double p = state.get(11,1);
  double q = state.get(12,1);
  double r = state.get(13,1);
  
  //Calculate Forces
  double force_max = 50;
  double s = 0.007681;
  double dpwm = (motor-STICK_MID);
  //printf("dpwm = %lf \n",dpwm);
  double dsteer = (steering - STICK_MID);
  double steer_angle = 45*PI/180.0 * dsteer / (STICK_MAX - STICK_MID);
  force = copysign(1.0,dpwm)*force_max*(1-exp(-s*fabs(dpwm)));
  //printf("FORCE = %lf \n",force);
  double xforce = force - 7.65*u;    // Modified to reflect 'Car' requirements
  //printf("xforce = %lf \n",xforce);
  double yforce = -10.0*v + 0.0*steer_angle;    // Modified to reflect 'Car' requirements

  //Calculate Moments
  double Nmoment = 75*(steer_angle-0.4*r);  
  
  //Forces
  FB.plus_eq1(1,1,xforce);
  FB.plus_eq1(2,1,yforce);
  FB.plus_eq1(3,1,0.0); 

  //Moments
  MB.plus_eq1(1,1,0.0);
  MB.plus_eq1(2,1,0.0);
  MB.plus_eq1(3,1,Nmoment);

  //FB.disp();
  //MB.disp();
  //PAUSE();
  
}


