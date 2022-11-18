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
  desired_moments.zeros(3,1,"desired moment");
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
  bool icontrol = 0;

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
    //case 3:
      //Two-Stage
      //TwoStageLoop(sense_matrix);
      //break;
    //case 2:
      //Feedback Linearized
      //FeedbackLinearizedLoop(sense_matrix);
      //break;
    case 1:
      //Proportional
      ProportionalLoop(sense_matrix);
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
      break;
  }
}

//void controller::TwoStageLoop(MATLAB sense_matrix) {}

//void controller::FeedbackLinearizedLoop(MATLAB sense_matrix) {}

void controller::ProportionalLoop(MATLAB sense_matrix) {
  double kpp = -0.02;
  double kpq = -0.2;
  double kpr = -0.05;
  pqr.vecset(1,3,sense_matrix,10);
  double p = pqr.get(1,1);
  double q = pqr.get(2,1);
  double r = pqr.get(3,1);
  //pqr.disp();
  //Calculate Error
  double p_error = p - p_command;
  double q_error = q - q_command;
  double r_error = r - r_command;
  //Apply P Control
  desired_moments.set(1,1,kpp*p_error);
  desired_moments.set(2,1,kpq*q_error);
  desired_moments.set(3,1,kpr*r_error);
  //desired_moments.disp();
  desired_moments.mult_eq(pwmC);
  //desired_moments.disp();
  desired_moments.plus_eq(STICK_MID);
  //desired_moments.disp();
  for (int i = 0;i<NUMSIGNALS;i++) {
    control_matrix.set(i+1,1,desired_moments.get(i+1,1));
  }
}
