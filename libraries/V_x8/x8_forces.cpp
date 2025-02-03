//This is for an x8 quad with 4 motors on top and 4 motors on bottom

#include "x8_forces.h"

//Constructor
forces::forces() {
  //The constructor must create these 3x1 vectors
  FB.zeros(3,1,"Force in Body Frame");
  MB.zeros(3,1,"Moment in Body Frame");
  thrust_motors.zeros(NUMMOTORS,1,"Force Motors");
  torque_motors.zeros(NUMMOTORS,1,"Torque Motors");

  //Quadcopter Aerodynamic Parameters
  double thrust_max = 0.4*GEARTH; //kilograms * gravity
  //Rotor Size
  Rrotor = (9.5/12.0)/(2*3.28); //9.5 inch props to meters
  //Area
  AREA = PI*pow(Rrotor,2.0);  //m^2
  //Battery Size
  double Voltage = 3.7*4; //4 Cell
  //KV Rating
  double KV = 950.0;
  //angular velocity is
  double omegaRPM = KV*Voltage;
  double omegarads = omegaRPM*2.0*PI/180.0;
  spin_slope = omegarads/(STICK_MAX-STICK_MIN);
  //printf("omegarads = %lf \n",omegarads);
  
  //These come from data sheets
  //at a signal of
  //double pwm_datapt = STICK_MID;
  //thrust is
  //double Tdatapt = 0.735*GEARTH/4.0; //Newtons to kg to lbf
  	
  //Compute Kt
  //double dpwm = pwm_datapt - STICK_MIN;
  //kt = Tdatapt/(dpwm*dpwm);
  
  //Angular Velocity computation
  //double a = (omegaRPMdatapt*2*PI/60.0)/dpwm;
  
  //Compute ct and cq
  ct = thrust_max/(0.5*RHOSLSI*AREA*pow(Rrotor*omegarads,2.0)); //.0335
  cq = pow(ct,3.0/2.0)/sqrt(2.0); 
  //printf("CT/CQ = %lf/%lf \n",ct,cq);
  //PAUSE();

  //Distance from Cg to rotor
  rx = (9.0/12.0)/3.28; //meters
  ry = (9.0/12.0)/3.28; 
  rz = 0.0;
}

void forces::compute_thrust_and_torque(MATLAB pwm_out) {
  for (int i = 1;i<=NUMMOTORS;i++) {
    double omega = spin_slope*(pwm_out.get(i,1) - OUTMIN);
    double thrust = 0.5*RHOSLSI*AREA*pow(omega*Rrotor,2.0)*ct;
    double torque = 0.5*RHOSLSI*AREA*pow(omega*Rrotor,2.0)*Rrotor*cq;
    thrust_motors.set(i,1,thrust);
    torque_motors.set(i,1,torque);
  }
  //thrust_motors.disp();
  //torque_motors.disp();
}

void forces::ForceMoment(double time,MATLAB state,MATLAB statedot,MATLAB pwm_out,environment env) {
  //The only thing this function needs to do is populate FAEROB and MAEROB. 
  //You can do whatever you want in here but you must create those two vectors.
  FB.mult_eq(0); //Zero these out just to make sure something is in here
  MB.mult_eq(0);
  //return;
  
  //ctlcomms.disp();
  //PAUSE();

  //First we need to convert the microsecond pulse to Newtons
  compute_thrust_and_torque(pwm_out);

  //Thrust on the body is simply the total thrust
  double thrust = thrust_motors.sum();
  //printf("thrust = %lf \n",thrust);
  FB.set(3,1,-thrust);

  //Torque is a bit more complex
  //First we need to make sure we understand the right order
  //Then we extract the four forces
  //From the controller - us signals
  //ctlcomms.set(1,1,motor_upper_left_bottom);
  //ctlcomms.set(2,1,motor_upper_right_bottom);
  //ctlcomms.set(3,1,motor_lower_right_bottom);
  //ctlcomms.set(4,1,motor_lower_left_bottom);
  //ctlcomms.set(5,1,motor_upper_left_top);
  //ctlcomms.set(6,1,motor_upper_right_top);
  //ctlcomms.set(7,1,motor_lower_right_top);
  //ctlcomms.set(8,1,motor_lower_left_top);
  //thrust_motors.disp();
  
  //We then extract the motors
  double motor_upper_left_bottom = thrust_motors.get(1,1);
  double motor_upper_right_bottom = thrust_motors.get(2,1);
  double motor_lower_right_bottom = thrust_motors.get(3,1);
  double motor_lower_left_bottom = thrust_motors.get(4,1);
  double motor_upper_left_top = thrust_motors.get(5,1);
  double motor_upper_right_top = thrust_motors.get(6,1);
  double motor_lower_right_top = thrust_motors.get(7,1);
  double motor_lower_left_top = thrust_motors.get(8,1);

  //then we add up all the torques in a specific way
  //note the commands from the controller
  //motor_upper_left_top = throttle - droll - dpitch - dyaw;
  //motor_upper_right_top = throttle + droll - dpitch + dyaw;
  //motor_lower_left_top = throttle - droll + dpitch + dyaw;
  //motor_lower_right_top = throttle + droll + dpitch - dyaw;
  
  //motor_upper_left_bottom = throttle - droll - dpitch + dyaw;
  //motor_upper_right_bottom = throttle + droll - dpitch - dyaw;
  //motor_lower_left_bottom = throttle - droll + dpitch - dyaw;
  //motor_lower_right_bottom = throttle + droll + dpitch + dyaw;
  double yaw_torque_top = motor_upper_left_top - motor_upper_right_top - motor_lower_left_top + motor_lower_right_top;
  double yaw_torque_bottom = -motor_upper_left_bottom + motor_upper_right_bottom + motor_lower_left_bottom - motor_lower_right_bottom;
  MB.set(3,1,yaw_torque_top+yaw_torque_bottom);

  //Now we compute torque on roll and pitch
  double roll_torque_top = (motor_upper_left_top+motor_lower_left_top)*ry - (motor_upper_right_top+motor_lower_right_top)*ry;
  double pitch_torque_top = (motor_upper_left_top+motor_upper_right_top)*rx - (motor_lower_right_top+motor_lower_left_top)*rx;

  //Now we compute torque on roll and pitch
  double roll_torque_bottom = (motor_upper_left_bottom+motor_lower_left_bottom)*ry - (motor_upper_right_bottom+motor_lower_right_bottom)*ry;
  double pitch_torque_bottom = (motor_upper_left_bottom+motor_upper_right_bottom)*rx - (motor_lower_right_bottom+motor_lower_left_bottom)*rx;
  
  MB.set(1,1,roll_torque_top+roll_torque_bottom);
  MB.set(2,1,pitch_torque_top+pitch_torque_bottom);

  //MB.disp();
  //FB.disp();
  //PAUSE();
}


