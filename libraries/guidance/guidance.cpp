//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "guidance.h"

guidance::guidance() {
};

void guidance::init(MATLAB in_configuration_matrix) {
  guidance_matrix.zeros(NUMSIGNALS,1,"Control Signals"); //The standards must be TAERA1A2A3A4
  set_defaults();
  printf("Guidance Received Configuration Matrix \n");
  //in_configuration_matrix.disp();
  int CONTROLLER_FLAG = in_configuration_matrix.get(11,1);
  if abs(CONTROLLER_FLAG) >= 10 {
    GUIDANCE_FLAG = 1;
  }
  printf("Guidance Setup Complete \n");
}

void guidance::print() {
  for (int i = 1;i<=NUMSIGNALS;i++) {
    printf("%d ",int(guidance_matrix.get(i,1)));
  }
}

void guidance::loop(double currentTime,int rx_array[],MATLAB sense_matrix) {
  //The sensor matrix is a 29x1. See sensors.cpp for list of sensors
  //At a minimum you need to just feed through the rxcomms into the ctlcomms
  //Which means you can't have more control signals than receiver signals

  //I want to keep track of timeElapsed so that I can run integrators
  //and compute derivates
  elapsedTime = currentTime - lastTime;
  lastTime = currentTime;

  //First extract the relavent commands from the receiver.
  throttle_rx = rx_array[0];
  aileron_rx = rx_array[1];
  elevator_rx = rx_array[2];
  rudder_rx = rx_array[3];
  arm_switch_rx = rx_array[4];

  //Initialize commands
  //Right now these are just zero but we will change these for another project
  roll_command = 0;
  pitch_command = 0;
  yaw_rate_command = 0;
  velocity_command = 0; 
  //Hardcoded to 100 but eventually need to have this update via telemetry
  altitude_command = 100; 

 //Send the motor commands to the guidance_matrix values
 guidance_matrix.set(1,1,roll_command);
 guidance_matrix.set(2,1,pitch_command);
 guidance_matrix.set(3,1,yaw_rate_command);
 guidance_matrix.set(4,1,velocity_command);
 guidance_matrix.set(5,1,altitude_command);
 guidance_matrix.set(6,1,arm_switch_rx);
 //guidance_matrix.disp();
}