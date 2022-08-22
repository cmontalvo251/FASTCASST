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
  double XDAMPCOEFF = 7.26;
  // double DAMPCOEFF = 50.0; //Guess and Check (y-direction)    // Modified to reflect 'Car' requirements
  // double DAMPROTCOEFF = 1.0; //Guess and Check (yaw)    // (ditto)
  double d = 0.13335; //(m) - From wheel to center
  double force1;
  // double force2;    // Modified to reflect 'Car' requirements
  double Vmax = 1.0; //(m/s) Need to find the max speed of the tank
  
  //Extract Actuator Values
  //Remember that control is in PWM (us)
  double motor1_US = pwm_array[0];
  // double motor2_US = pwm_array[1];    // Similary, 'Car' does not need 'motor2_US', as it has only one (1) motor.

  //Extract States
  double x = state.get(1,1);
  //printf("x = %lf \n",x);
  double u = state.get(8,1);
  //printf("u = %lf \n",u);
  double v = state.get(9,1);
  double w = state.get(10,1);
  double p = state.get(11,1);
  double q = state.get(12,1);
  double r = state.get(13,1);
  
  //Calculate Forces
  double force_max = (((1.7)/2.2)*9.81 + 4.454)/2.0;
  double s = 0.007681;
  //motor1_US = STICK_MAX;
  //motor2_US = STICK_MAX;
  double dpwm1 = (motor1_US-STICK_MID);
  // double dpwm2 = (motor2_US-STICK_MID); //Need to find equation by plotting microsec vs force    // Similarly, 'Car' does not need 'dpwm2',...
  force1 = -copysign(1.0,dpwm1)*force_max*(1-exp(-s*fabs(dpwm1)));
  // force2 = copysign(1.0,dpwm2)*force_max*(1-exp(-s*fabs(dpwm2)));    // Similarly, 'Car' does not need 'force2' as it acts in the y-direction.
  //printf("forces before = %lf %lf \n",force1,force2);
  
  /*double vf=1.0;
  //First check and see if the user is trying to accelerate and moving forward
  if ((u > 0) && (force1+force2 > 0)) {
  vf = 1-u/Vmax;
  }
  if ((u < 0) && (force1 + force2 < 0)) {
  vf = 1+u/Vmax;
  }
  
  if (vf < 0) {
  force1 = 0.0;
  force2 = 0.0;
  } else {
  force1 *= vf;
  force2 *= vf;
  }*/
  
  //printf("forces after = %lf %lf \n",force1,force2);
  // double xforce = force1 + force2 - XDAMPCOEFF*u;    // Modified to reflect 'Car' requirements

  double xforce = force1 - XDAMPCOEFF*u;    // Modified to reflect 'Car' requirements
  
  // double yforce = -DAMPCOEFF*v;    // Modified to reflect 'Car' requirements
  double yforce = -0.0000001*v;    // Modified to reflect 'Car' requirements

  //Calculate Moments
  // double Nmoment = (force1 - force2)*d - DAMPROTCOEFF*r;    // // Modified to reflect 'Car' requirements
  double Nmoment = (force1)*d - 0.0000001*r;    // // Modified to reflect 'Car' requirements
  
  //Forces
  FB.plus_eq1(1,1,xforce);
  FB.plus_eq1(2,1,yforce);
  FB.plus_eq1(3,1,0.0); 

  //Moments
  MB.plus_eq1(1,1,0.0);
  MB.plus_eq1(2,1,0.0);
  MB.plus_eq1(3,1,Nmoment);
  
  /*FB.disp();
  MB.disp();
  for (int i = 0;i<2;i++) {
    printf(" %d ",pwm_array[i]);
  }
  printf("\n");
  PAUSE();*/
}


