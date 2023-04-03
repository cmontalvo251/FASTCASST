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
  MMTVEC.zeros(3,1,"Magnetometer Momemt");
}

void forces::ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB pwm_out,environment env) {
  //The only thing this function needs to do is populate FB and MB. 
  //You can do whatever you want in here but you must create those two vectors.
  FB.mult_eq(0); //Zero these out just to make sure something is in here
  MB.mult_eq(0);
  
  //This is where magnetorquer torque is computed
  //actuators.disp();
  //Cannot overwrite because actuators might be less than 3
  MMTVEC.mult_eq(0);
  //MMTVEC.overwrite(actuators);
  //MMTVEC.overwrite(pwm_out);
  for (int i = 0;i<NUMTORQUERS;i++){
    //printf("%d ",pwm_array[i]);
    //MMTVEC.set(i+1,1,pwm_array[i]);
    MMTVEC.set(i+1,1,pwm_out.get(i+1,1));
  }
  //Substract the offset
  MMTVEC.minus_eq(STICK_MID);
  //Convert to Current (not quite working)
  MMTVEC.mult_eq(IpwmC);
  //Saturation on current
  double sum = MMTVEC.abssum();
  if (sum > MAXCURRENT) {
    MMTVEC.mult_eq(MAXCURRENT/sum);
    // for (int i = 0;i<3;i++) {
    //   double val = ctlcomms.get(i+1,1);
    //   ctlcomms.set(i+1,val/sum*maxcurrent);
  }
  //printf("\n");
  MMTVEC.mult_eq(AREA*NUMTURNS);
  //Once you have the magnetic moment and magnetic field you can compute the total
  //torque placed on the satellite
  MB.cross(MMTVEC, env.BVECB_Tesla);
}


