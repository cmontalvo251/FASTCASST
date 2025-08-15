/* Forces Template 2021

This forces file is a template for a fictitious portalcube
with thrusters and a simple  model. The Dynamics.cpp module
will call a few candidate functions. If you make your own 
file with header and cpp file you must conform to the following
functions otherwise the software will completely break.

*/

#include "cubesat_forces.h"

//Constructor
forces::forces() {
  //The constructor must create these 3x1 vectors
  FB.zeros(3,1,"Force in Body Frame");
  MB.zeros(3,1,"Moment in Body Frame");
  pqr_PWM.zeros(3,1,"PQR converted to PWM");
}

void forces::ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB actuatorStates,environment env) {
  //The only thing this function needs to do is populate FB and MB. 
  //You can do whatever you want in here but you must create those two vectors.
  FB.mult_eq(0); //Zero these out just to make sure something is in here
  MB.mult_eq(0);

  //state.disp();
  pqr_PWM.mult_eq(0);
  //pqr_PWM.plus_eq(STICK_MID);
  //actuatorStates.disp();
  for (int i = 0;i<NUMTORQUERS;i++){
    pqr_PWM.set(i+1,1,actuatorStates.get(i+1,1));
    //printf("pwm_array = %d \n",pwm_array[i]);
  }
  pqr_PWM.minus_eq(STICK_MID);
  pqr_PWM.mult_eq(IpwmC);
  //pqr_PWM.disp();
  
  //PAUSE();
  
  //torque placed on the satellite
  MB.vecset(1,3,pqr_PWM,1);
  //MB.disp();
}
