//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "controller.h"

controller::controller() {
  control_matrix.zeros(NUMSIGNALS,1,"PWM Control Signals"); //The standards must be TAERA1A2A3A4
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

  double motor = STICK_MID; //Left Motor
  double servo = STICK_MID; //Right Motor
  double DIFFERENTIAL = 1.0;

  //First extract the relavent commands from the receiver.
  double throttle = rx_array[0];
  double aileron = rx_array[1];
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

  //Then you can run any control loop you want.
  if (icontrol) {
    //Control system on
    //printf("Auto ON \n");
    motor = STICK_MID;
    servo = aileron;
  } else {
    //Control system off
    motor = throttle;
    servo = aileron;
  }

  //Saturation
  if (motor < STICK_MIN) {
    motor = STICK_MIN;
  }
  if (motor > STICK_MAX) {
    motor = STICK_MAX;
  }
  if (servo < STICK_MIN) {
    servo = STICK_MIN;
  }
  if (servo > STICK_MAX) {
    servo = STICK_MAX;
  }
  
  //Set motor commands to the ctlcomms values
  control_matrix.set(1,1,motor);
  control_matrix.set(2,1,servo);
  //ctlcomms.disp();
}
