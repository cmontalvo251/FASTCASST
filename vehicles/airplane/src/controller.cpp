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
  control_matrix.set(1,1,OUTMIN);
  control_matrix.set(2,1,OUTMID);
  control_matrix.set(3,1,OUTMID);
  control_matrix.set(4,1,OUTMID);
}

void controller::print() {
  for (int i = 1;i<=NUMSIGNALS;i++) {
    printf("%d ",int(control_matrix.get(i,1)));
  }
}

void controller::loop(double currentTime,int rx_array[],MATLAB sense_matrix) {
  //The sensor matrix is a 29x1. See sensors.cpp for list of sensors
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

  //sense_matrix.disp();
  //while(1){};

  //Then you can run any control loop you want.
  if (icontrol) {
    #ifdef SIL 
    printf("Auto ON \n");
    #endif

    //For now just pitch and roll commands
    double roll_command = (aileron-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
    double pitch_command = -(elevator-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
    
    //Get States
    double roll = sense_matrix.get(4,1);
    double pitch = sense_matrix.get(5,1);
    double roll_rate = sense_matrix.get(10,1); //For SIL/SIMONLY see Sensors.cpp
    double pitch_rate = sense_matrix.get(11,1); //These are already in deg/s
    //printf("PQR Rate in Controller %lf %lf %lf \n",roll_rate,pitch_rate,yaw_rate);
    double kp = 10.0;
    double kd = 2.0;
    double aileron = kp*(roll-roll_command) + kd*(roll_rate);
    aileron = -CONSTRAIN(aileron,-500,500) + OUTMID;
    double elevator = kp*(pitch-pitch_command) + kd*(pitch_rate);
    elevator = CONSTRAIN(elevator,-500,500) + OUTMID;
    
    control_matrix.set(2,1,aileron);
    control_matrix.set(3,1,elevator);
  } else {
    //printf("Passing RX to PWM \n");
    for (int i = 0;i<4;i++) {
      control_matrix.set(i+1,1,rx_array[i]);
    }
  }
}
