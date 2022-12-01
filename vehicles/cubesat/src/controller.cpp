//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "controller.h"

controller::controller() {
};

void controller::init(MATLAB in_configuration_matrix) {
  control_matrix.zeros(NUMSIGNALS,1,"Control Signals"); //The standards must be TAERA1A2A3A4
  set_defaults();
  printf("Controller Received Configuration Matrix \n");
  //in_configuration_matrix.disp();
  CONTROLLER_FLAG = in_configuration_matrix.get(11,1);
  printf("Controller Setup \n");
  pqr.zeros(3,1,"PQR");
  mxyz.zeros(3,1,"MXYZ");
  desired_moments.zeros(NUMSIGNALS,1,"desired moment");
  fully_controlled.zeros(3,1,"fully controlled");
  //FEEDBACK LIN
  gamma.zeros(3,1,"Gamma for FL");
  I.zeros(3,3,"CONTROLLER INERTIA");
  I_pqr.zeros(3,1,"I*pqr");
  I_gamma.zeros(3,1,"I*gamma");
  pqrskew_I_pqr.zeros(3,1,"pqr x I*pqr");
  double Ixx = in_configuration_matrix.get(13,1);
  double Iyy = in_configuration_matrix.get(14,1);
  double Izz = in_configuration_matrix.get(15,1);
  I.set(1,1,Ixx);
  I.set(2,2,Iyy);
  I.set(3,3,Izz);
  if (in_configuration_matrix.length() > 15) {
    double Ixy = in_configuration_matrix.get(16,1);
    double Ixz = in_configuration_matrix.get(17,1);
    double Iyz = in_configuration_matrix.get(18,1);
    I.set(1,2,Ixy);
    I.set(2,1,Ixy);
    I.set(1,3,Ixz);
    I.set(3,1,Ixz);
    I.set(2,3,Iyz);
    I.set(3,2,Iyz);
  }
}

void controller::set_defaults() {
  control_matrix.mult_eq(0);
  control_matrix.plus_eq(STICK_MID);
}

void controller::print() {
  for (int i = 1;i<=NUMSIGNALS;i++) {
    printf("%d ",int(control_matrix.get(i,1)));
  }
}

void controller::loop(double currentTime,int rx_array[],MATLAB sense_matrix) {
  //The sensor state is a 12x1 of standard 6DOF sensors
  //At a minimum you need to just feed through the rxcomms into the ctlcomms
  //Which means you can't have more control signals than receiver signals
  
  //Default Control Signals
  set_defaults();

  //I want to keep track of timeElapsed so that I can run integrators
  //and compute derivates
  elapsedTime = currentTime - lastTime;
  lastTime = currentTime;
  
  //First extract the relavent commands from the receiver.
  double throttle = rx_array[0];
  double aileron = rx_array[1];
  double elevator = rx_array[2];
  double rudder = rx_array[3];
  double autopilot = rx_array[4];
  int icontrol = 0;

  //Check for user controlled
  if (CONTROLLER_FLAG < 0) {
    if (autopilot > STICK_MID) {
      icontrol = -CONTROLLER_FLAG;
    } else {
      icontrol = 0;
    }
  } else {
    icontrol = CONTROLLER_FLAG;
  }
  //printf("CONTROLLER FLAG = %d \n",CONTROLLER_FLAG);

  //Aircraft Control cases
  // 0 = No Control
  // 1 = Proportional Control
  // 2 = Feedback Linearized Control
  // 3 = Two-Stage Control
  //Initialize commands
  p_command = 0.0;
  q_command = 0.0;
  r_command = 0.0;

  switch (icontrol) {
    case 3:
      //Two-Stage
      TwoStageLoop(sense_matrix);
      //printf("Two-Stage \n");
      break;
    case 2:
      //Feedback Linearized
      FeedbackLinearizedLoop(sense_matrix);
      //printf("FL \n");
      break;
    case 1:
      //Proportional
      ProportionalLoop(sense_matrix);
      //printf("PID \n");
      break;
    case 0:
      //No Control
      //sense_matrix.disp();
      //pqr.vecset(1,3,sense_matrix,10);
      //pqr.disp();
      //desired_moments.vecset(1,3,pqr,1);
      //desired_moments.disp();
      desired_moments.mult_eq(0.0);
      //desired_moments.disp();
      desired_moments.plus_eq(STICK_MID);
      //desired_moments.disp();
      for (int i = 0;i<NUMSIGNALS;i++) {
        control_matrix.set(i+1,1,desired_moments.get(i+1,1));
      }
      //control_matrix.disp();
      //printf("MANUAL CONTROL \n");
      break;
  }
}

void controller::TwoStageLoop(MATLAB sense_matrix) {
  //Underactuated:
  //1U: -4, -2, -1
  //2U,upright: -4, -2, -1
  //2U,sideways: -4, -2, -1
  //6U: -1.5, -0.5, -1.25
  double kpp = -4.;
  double kpq = -2.;
  double kpr = -1.;
  pqr.vecset(1,3,sense_matrix,10);
  double p = pqr.get(1,1);
  double q = pqr.get(2,1);
  double r = pqr.get(3,1);
  pqr.disp();
  double Ixx = I.get(1,1);
  double Iyy = I.get(2,2);
  double Izz = I.get(3,3);
  //Stage 1
  if(abs(q) > 0.009 || abs(r) > 0.009){
    //printf("First Stage \n");
    p_command = 0.5;
    r_command = 0.0;
    //Start with P
    double p_error = p - p_command;
    double gammaP = kpp*p_error;
    double L = Ixx*gammaP - q*r*(Iyy-Izz);
    //Move to R
    double r_error = r - r_command;
    double gammaR = kpr*r_error;
    q_command = (Izz*gammaR)/(p*(Ixx-Iyy));
    //printf("Q Command: %lf \n",q_command);
    if (abs(q_command) > 1.0) {
      q_command = copysign(1.0,q_command)*1.0;
    }
    //printf("QCommand = %lf \n",q_command);
    //End with Q
    double q_error = q - q_command;
    double gammaQ = kpq*q_error;
    double M = gammaQ*Iyy - p*r*(Izz-Ixx);
    //FULLY CONTORLLED MATRIX
    fully_controlled.set(1,1,L);
    fully_controlled.set(2,1,M);
  }
  //Stage 2
  else{
    //printf("Second Stage \n");
    p_command = 0.;
    q_command = 0.;
    //Start with P
    double p_error = p - p_command;
    double gammaP = kpp*p_error;
    double L = Ixx*gammaP - q*r*(Iyy-Izz);
    //End with Q
    double q_error = q - q_command;
    double gammaQ = kpq*q_error;
    double M = gammaQ*Iyy - p*r*(Izz-Ixx);
    //FULLY CONTORLLED MATRIX
    fully_controlled.set(1,1,L);
    fully_controlled.set(2,1,M);
    }
  CleanControl();
}


void controller::FeedbackLinearizedLoop(MATLAB sense_matrix) {
  //Fully Actuated:
  //1U: -35, -35, -35
  //2U,upright: -25, -25, -30
  //2U,sideways: -30, -25, -25
  //6U: -5, -6.5, -12
  double kpp = -5.0;
  double kpq = -6.5;
  double kpr = -12.0;
  pqr.vecset(1,3,sense_matrix,10);
  double p = pqr.get(1,1);
  double q = pqr.get(2,1);
  double r = pqr.get(3,1);
  //pqr.disp();
  //Calculate Error
  double p_error = p - p_command;
  double q_error = q - q_command;
  double r_error = r - r_command;
  //GAMMA
  gamma.set(1,1,kpp*p_error);
  gamma.set(2,1,kpq*q_error);
  gamma.set(3,1,kpr*r_error); 
  // I * gamma + omega cross I omega 
  //I.disp();
  //gamma.disp();
  I_gamma.mult(I,gamma);
  //I_gamma.disp();
  I_pqr.mult(I,pqr);
  //I_pqr.disp();
  pqrskew_I_pqr.cross(pqr,I_pqr);
  //pqrskew_I_pqr.disp();
  I_gamma.plus_eq(pqrskew_I_pqr);
  fully_controlled.overwrite(I_gamma);
  //fully_controlled.disp();

  CleanControl();
}

void controller::ProportionalLoop(MATLAB sense_matrix) {
  //Fully Actuated:
  //1U: -0.1, -0.1, -0.1
  //2U,upright: -0.5, -0.5, -0.2
  //2U,sideways: -0.2, -0.5, -0.5
  //6U: -0.6, -0.55, -0.55
  double kpp = -0.6;
  double kpq = -0.55;
  double kpr = -0.55;
  pqr.vecset(1,3,sense_matrix,10);
  double p = pqr.get(1,1);
  double q = pqr.get(2,1);
  double r = pqr.get(3,1);
  //pqr.disp();
  //Calculate Error
  double p_error = p - p_command;
  double q_error = q - q_command;
  double r_error = r - r_command;
  //FULLY CONTORLLED MATRIX
  fully_controlled.set(1,1,kpp*p_error);
  fully_controlled.set(2,1,kpq*q_error);
  fully_controlled.set(3,1,kpr*r_error);

  CleanControl();
}

void controller::CleanControl() {
  //Apply P Control
  for (int i = 0;i<NUMSIGNALS;i++) {
    desired_moments.set(i+1,1,fully_controlled.get(i+1,1));
  }

  //MAXIMUM MOMENT
  //Using VACCO's Integrated Prop System
  //https://cubesat-propulsion.com/integrated-propulsion-system/
  //Max Thrust = 2N
  //Moment Arm = 0.5U = 0.05
  //Max Moment = Max Thrust * Moment Arm
  double max_moment = 0.1; //Nm

  for (int i = 0;i<NUMSIGNALS;i++) {
    double input = desired_moments.get(i+1,1);
    double output = saturation(input,max_moment);
    desired_moments.set(i+1,1,output);
  }

  //desired_moments.disp();
  desired_moments.mult_eq(pwmC);
  //desired_moments.disp();
  desired_moments.plus_eq(STICK_MID);
  //desired_moments.disp();
  for (int i = 0;i<NUMSIGNALS;i++) {
    control_matrix.set(i+1,1,desired_moments.get(i+1,1));
  }
  //control_matrix.disp();
}
