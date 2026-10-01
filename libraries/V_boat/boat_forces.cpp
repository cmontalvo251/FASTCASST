/* Forces Template 2026

This forces file is a template for a **BOAT** 
but literally using the car routine since I don't have the RAOs yet

*/

#include "boat_forces.h"

//Constructor
forces::forces() {
  //The constructor must create these 3x1 vectors
  FB.zeros(3,1,"Force in Body Frame");
  MB.zeros(3,1,"Moment in Body Frame");
}

void forces::ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB pwm_out,const environment& env) {
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
  //double motor = pwm_array[0];
  double motor1 = pwm_out.get(1,1);
  //printf("MOTOR = %lf \n",motor);
  //double steering = pwm_array[1]; 
  double motor2 = pwm_out.get(2,1);
  //pwm_out.disp();

  // Cast const away from env to call non-const get() methods
  environment& env_nonconst = const_cast<environment&>(env);

  //Extract States
  double u = state.get(8,1);
  double v = state.get(9,1);
  double w = state.get(10,1);
  double p = state.get(11,1);
  double q = state.get(12,1);
  double r = state.get(13,1);

  double uEXT = env_nonconst.AEROVECB.get(1,1);
  double vEXT = env_nonconst.AEROVECB.get(2,1);
  double wEXT = env_nonconst.AEROVECB.get(3,1);
  double pEXT = env_nonconst.AEROMOMENTB.get(1,1);
  double qEXT = env_nonconst.AEROMOMENTB.get(2,1);
  double rEXT = env_nonconst.AEROMOMENTB.get(3,1);
  double EXT = 1.0;
  
  //Calculate Forces
  double force_max = 50;
  double s = 0.007681;
  double dpwm = ((motor1+motor2)/2.0-STICK_MID);
  force = copysign(1.0,dpwm)*force_max*(1-exp(-s*fabs(dpwm)));
  double xforce = force - 2.65*u + EXT*uEXT;  
  double yforce = -10.0*v + EXT*vEXT;
  double zforce = -10.0*w + EXT*wEXT;

  //Calculate Moments
  double Lmoment = -75*0.4*p + EXT*pEXT;
  double Mmoment = -75*0.4*q + EXT*qEXT;
  double dsteer = ((motor1-motor2));
  double steer_angle = 45*PI/180.0 * dsteer / (STICK_MAX - STICK_MID);
  double Nmoment = 75*(steer_angle-0.4*r) + EXT*rEXT;

  //Forces
  FB.plus_eq1(1,1,xforce);
  FB.plus_eq1(2,1,yforce);
  FB.plus_eq1(3,1,zforce); 

  //Moments
  MB.plus_eq1(1,1,Lmoment);
  MB.plus_eq1(2,1,Mmoment);
  MB.plus_eq1(3,1,Nmoment);

  //FB.disp();
  //MB.disp();
  //PAUSE();
  
}


