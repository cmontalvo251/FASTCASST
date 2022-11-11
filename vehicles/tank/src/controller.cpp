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
}

void controller::set_defaults() {
  control_matrix.set(1,1,OUTMID);
  control_matrix.set(2,1,OUTMID);
}

void controller::print() {
  for (int i = 1;i<=NUMSIGNALS;i++) {
    printf("%d ",int(control_matrix.get(i,1)));
  }
}

void controller::loop(double currentTime,int rx_array[],MATLAB sense_matrix) {
  //The sensor matrix is a 29x1. See sensors.cpp for list of sensors
  //At a minimum you need to just feed through the rxcomms into the control_matrix
  //Which means you can't have more control signals than receiver signals

  //Default Control Signals
  set_defaults();

  //I want to keep track of timeElapsed so that I can run integrators
  //and compute derivates
  elapsedTime = currentTime - lastTime;
  lastTime = currentTime;

  double motor1_us = STICK_MID; //Left Motor
  double motor2_us = STICK_MID; //Right Motor
  double DIFFERENTIAL = 1.0;

  //First extract the relavent commands from the receiver.
  double throttle = rx_array[0];
  double aileron = rx_array[1];
  //printf("Elevator = %lf \n",elevator);
  double autopilot = rx_array[4];
  bool icontrol = 0;

  switch (CONTROLLER_FLAG) {
  case -1:
    //User decides
    if (autopilot > STICK_MID) {
      icontrol = 1;
    } else {
      icontrol = 0;
    }
    break;
  case 0:
    //Always off
    icontrol = 0;
    break;
  case 1:
    //Always on
    icontrol = 1;
    break;
  }

  double aileron_command = 0,throttle_command = 0;

  //Then you can run any control loop you want.
  if (icontrol) {
    //printf("Auto ON \n");
    double yaw = sense_matrix.get(20,1);
    printf("yaw = %lf \n",yaw);
    throttle_command = STICK_MID + 0.5*(STICK_MAX-STICK_MID);
    double yaw_command = 0;
    aileron_command = STICK_MID - 50*(yaw-yaw_command);
  } else {
    throttle_command = throttle;
    aileron_command = aileron;
  }

  motor1_us = STICK_MID + (STICK_MID - throttle_command) - DIFFERENTIAL*(aileron_command - STICK_MID);
  motor2_us = throttle_command - DIFFERENTIAL*(aileron_command - STICK_MID);

  //Saturation
  if (motor1_us < STICK_MIN) {
    motor1_us = STICK_MIN;
  }
  if (motor1_us > STICK_MAX) {
    motor1_us = STICK_MAX;
  }
  if (motor2_us < STICK_MIN) {
    motor2_us = STICK_MIN;
  }
  if (motor2_us > STICK_MAX) {
    motor2_us = STICK_MAX;
  }
  
  //Set motor commands to the ctlcomms values
  control_matrix.set(1,1,motor1_us);
  control_matrix.set(2,1,motor2_us);
  //control_matrix.disp();
}
