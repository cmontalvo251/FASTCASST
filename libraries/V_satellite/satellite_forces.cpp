/* Forces Template 2021

This forces file is a template for a fictitious portalcube
with thrusters and a simple  model. The Dynamics.cpp module
will call a few candidate functions. If you make your own 
file with header and cpp file you must conform to the following
functions otherwise the software will completely break.

*/

#include "satellite_forces.h"

//Constructor
forces::forces() {
  //The constructor must create these 3x1 vectors
  FB.zeros(3,1,"Force in Body Frame");
  MB.zeros(3,1,"Moment in Body Frame");
  MMTVEC.zeros(3,1,"Magnetometer Momemt");
  omega_RWS.zeros(3,1,"Angular Velocity of RWs");

  //Here are the default parameters for the reaction wheels
  //#define NUMRWS 3 //These are the number of reaction wheels in the system. 3 is the minimum for full 3-axis control.
  //#define RADRW 0.07 //Radius of reaction wheel (m)
  //#define MAXMOMENT 0.1 //Maximum momentum Nms
  //#define MASSRW 0.33 //Mass of reaction wheel (kg) //These are based on BCT RWp100
  //#define MAXTORQUE 0.007 //Maximum Torque N-m
  //Inertia = m/2*r^2
  Irw = MASSRW/2.0*RADRW*RADRW;
  //Torque = Inertia * angular acceleration
  MAXACCEL = MAXTORQUE/Irw;
  //Angular momenttum = Inertia * angular velocity
  MAXOMEGA = MAXMOMENT/Irw;
  //printdouble(MAXOMEGA,"Maximum angular velocity");
  //printdouble(Irw,"Irw");
  //printdouble(MAXACCEL,"MAXACCEL");
  //PAUSE();
}

void forces::ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB pwm_out,const environment& env) {
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
  for (int i = 0;i<3;i++){ //Maximum of 3 magnetorquers. If you have less than 3 then the rest are zero
    //printf("%d ",pwm_array[i]);
    //MMTVEC.set(i+1,1,pwm_array[i]);
    if (i < NUMTORQUERS) { //NUMTORQUERS is set in params.h
      //Subtract the offset here and convert to current. 
      //This is a simple model and not quite right but it works for now
      double pwm = pwm_out.get(i+1,1);
      double pwm_offset = pwm-STICK_MID;
      double current = IpwmC*pwm_offset;
      MMTVEC.set(i+1,1,current);
    } else {
      MMTVEC.set(i+1,1,0);
    }
  }
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

  //Then we need to add in the torque from reaction wheels
  //FB.disp();
  //pwm_out.disp();
  
  //Ok so according to Control.m our pwm_out is an angular acceleration value
  //So we take said angular acceleration and compute the torque on the system
  //But before we do that we need to make sure our angular velocity isn't too big - this is accomplished by our two if loops
  //Which means we need to keep track of angular velocity somehow -- omega_RWS will be our vector
  for (int i = 4;i<=6;i++) {
    double pwm_offset;
    pwm_offset = pwm_out.get(i,1)-OUTMID;
    //if (i == 4){
    //  printdouble(pwm_offset,"pwm_offset");
    //  omega_RWS.disp();
    //}
    if ((pwm_offset>0) && (omega_RWS.get(i-3,1)>=MAXOMEGA)) {
      pwm_offset = 0; //set this angular velocity to zero because the reaction wheel is maxed out
    }
    if ((pwm_offset<0) && (omega_RWS.get(i-3,1)<=-MAXOMEGA)) {
      pwm_offset=0; //set this angular velocity to zero because the reaction wheel is maxed out
    }
    
    //Alright now that we know our pwm_out is correct we then convert the pwm_out to actual rad/s^2
    double alfa = pwm_offset/dPWM*MAXACCEL; 
    //Then we can integrate the angular velocity
    omega_RWS.plus_eq1(i-3,1,alfa*(time-prev_time));
    //Finally we can compute the torque based on the angular acceleration
    MB.plus_eq1(i-3,1,alfa*Irw);
    //if (i == 4) {
    //  printdouble(alfa,"alfa");
    //  printdouble(omega_RWS.get(i-3,1),"angular velocity");
    //  PAUSE();
      //MB.disp();
    //}
  }
  //Reset time
  prev_time = time;
  //omega_RWS.disp();
  //pwm_out.disp();
  //MB.disp();
}


