/* Forces Template 2021

This forces file is a template for a fictitious portalcube
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
  		
  //Get States
  double z = state.get(3,1);
  double u = state.get(8,1);
  double v = state.get(9,1);
  double w = state.get(10,1);
  double p = state.get(11,1);
  double q = state.get(12,1);
  double r = state.get(13,1);
  //state.disp();

  //Total Velocity
  double vinf=sqrt(u*u+v*v+w*w); //#Stream line velocity: Total velocity
  
  //Angle of Attack and sideslip
  double beta=asin(v/vinf); //#Equation 21: Sideslip angle beta
  double alpha=atan(w/u); //#Equation 20: Angle of attack alpha
  
  //printf("AOA,BETA = %lf %lf \n",beta,alpha);

  //Non-deminnsion angular velocity
  double phat=(p*aeropack.bws)/(2*vinf);
  double qhat=(q*aeropack.cbar)/(2*vinf);
  double rhat=(r*aeropack.bws)/(2*vinf);
  
  //printf("pqrhat = %lf %lf %lf \n",phat,qhat,rhat);

  //Extract Controls
  //Extract Actuator Values
  //Remember that control is in PWM (us)
  double mewt = pwm_array[0];
  double delaUS = pwm_array[1];
  double deleUS = pwm_array[2];
  double delrUS = pwm_array[3];
  double mewt2 = pwm_array[4];
  double delaUS2 = pwm_array[5];
  double deleUS2 = pwm_array[6];
  double delrUS2 = pwm_array[7];

  //Convert US to degrees
  double dela = 2*30*PI/(180.0*(OUTMAX-OUTMIN))*(delaUS-OUTMID);
  double dele = 2*30*PI/(180.0*(OUTMAX-OUTMIN))*(deleUS-OUTMID); 
  double delr = 2*30*PI/(180.0*(OUTMAX-OUTMIN))*(delrUS-OUTMID);
  double dela2 = 2*30*PI/(180.0*(OUTMAX-OUTMIN))*(delaUS2-OUTMID);
  double dele2 = 2*30*PI/(180.0*(OUTMAX-OUTMIN))*(deleUS2-OUTMID); 
  double delr2 = 2*30*PI/(180.0*(OUTMAX-OUTMIN))*(delrUS2-OUTMID);
  
  //printf("DELE = %lf DELE2 = %lf \n",dele,dele2);
  
  //actuators.disp();
  
  //Compute Thrust - 1
  double omega = aeropack.spin_slope*(mewt - OUTMIN);
  double Thrust = 0.5*RHOSLSI*aeropack.AREA*pow(omega*aeropack.Rrotor,2.0)*aeropack.ct;
  if (vinf < aeropack.max_speed) {
    Thrust *= vinf/aeropack.max_speed;
  } else {
    Thrust = 0.0;
    //printf("Velocity = %lf \n",vinf);
  }
  //printf("Thrust = %lf \n",Thrust);

  //Compute Thrust - 2
  double omega2 = aeropack.spin_slope*(mewt2 - OUTMIN);
  double Thrust2 = 0.5*RHOSLSI*aeropack.AREA*pow(omega2*aeropack.Rrotor,2.0)*aeropack.ct;
  if (vinf < aeropack.max_speed) {
    Thrust2 *= vinf/aeropack.max_speed;
  } else {
    Thrust2 = 0.0;
    //printf("Velocity = %lf \n",vinf);
  }
		
  //forces Coefficients
  double CL = aeropack.CLzero + (aeropack.CLalpha*alpha) + (aeropack.CLq*qhat) + (aeropack.CLdele*dele); //#Equation 19: Coefficient of Lift
  double CD = aeropack.CDzero + (aeropack.CDalpha*(alpha*alpha)); //#Equation 19: Coefficient of Drag
  double CY = (aeropack.Cybeta*beta)+(aeropack.Cydelr*delr)+(aeropack.Cyp*phat)+(aeropack.Cyr*rhat);
  //printf("CY = %lf \n",CY);
  double Cm = (aeropack.Cmzero)+(aeropack.Cmalpha*alpha)+(aeropack.Cmq*qhat)+(aeropack.Cmdele*dele);
  double Cl = (aeropack.CLbeta*beta)+(aeropack.CLp*phat)+(aeropack.CLr*rhat)+(aeropack.CLdela*dela)+(aeropack.CLdelr*delr);
  double Cn = (aeropack.Cnp*phat)+(aeropack.Cnbeta*beta)+(aeropack.Cnr*rhat)+(aeropack.Cndela*dela)+(aeropack.Cndelr*delr);

  //Compute Forces and moments
  double fx = 0.5*RHOSLSI*(vinf*vinf)*aeropack.S*(CL*sin(alpha)-CD*cos(alpha)) + Thrust;
  double fy = 0.5*RHOSLSI*(vinf*vinf)*aeropack.S*CY;
  double fz = 0.5*RHOSLSI*(vinf*vinf)*aeropack.S*(-CL*cos(alpha)-CD*sin(alpha));

  double mx = aeropack.bws*Cl;
  double my = aeropack.cbar*Cm;
  double mz = aeropack.bws*Cn;

  //forces Coefficients
  double CL2 = aeropack.CLzero + (aeropack.CLalpha*alpha) + (aeropack.CLq*qhat) + (aeropack.CLdele*dele2); //#Equation 19: Coefficient of Lift
  double CD2 = aeropack.CDzero + (aeropack.CDalpha*(alpha*alpha)); //#Equation 19: Coefficient of Drag
  double CY2 = (aeropack.Cybeta*beta)+(aeropack.Cydelr*delr2)+(aeropack.Cyp*phat)+(aeropack.Cyr*rhat);
  //printf("CY = %lf \n",CY);
  double Cm2 = (aeropack.Cmzero)+(aeropack.Cmalpha*alpha)+(aeropack.Cmq*qhat)+(aeropack.Cmdele*dele2);
  double Cl2 = (aeropack.CLbeta*beta)+(aeropack.CLp*phat)+(aeropack.CLr*rhat)+(aeropack.CLdela*dela2)+(aeropack.CLdelr*delr2);
  double Cn2 = (aeropack.Cnp*phat)+(aeropack.Cnbeta*beta)+(aeropack.Cnr*rhat)+(aeropack.Cndela*dela2)+(aeropack.Cndelr*delr2);

  //Compute Forces and moments
  double fx2 = 0.5*RHOSLSI*(vinf*vinf)*aeropack.S*(CL2*sin(alpha)-CD2*cos(alpha)) + Thrust2;
  double fy2 = 0.5*RHOSLSI*(vinf*vinf)*aeropack.S*CY2;
  double fz2 = 0.5*RHOSLSI*(vinf*vinf)*aeropack.S*(-CL2*cos(alpha)-CD2*sin(alpha));

  double mx2 = aeropack.bws*Cl2;
  double my2 = aeropack.cbar*Cm2;
  double mz2 = aeropack.bws*Cn2;
  
  //Forces and Moments
  FB.set(1,1,fx+fx2);
  FB.set(2,1,fy+fy2);
  FB.set(3,1,fz+fz2);
  double l = aeropack.bws/2.0;
  MB.set(1,1,mx+mx2-l*fz+l*fz2);
  MB.set(2,1,my+my2);
  MB.set(3,1,mz+mz2+l*fx-l*fx2);

  //FB.disp();
  //MB.disp();
  //PAUSE();
}


