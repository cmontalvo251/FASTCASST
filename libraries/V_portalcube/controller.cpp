//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "controller.h"

//Constructor
controller::controller() {
};

//Initialization
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

//Main Control Loop
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
  
  //Then you can run any control loop you want.
  if (icontrol) {
    #ifdef SIL 
    printf("Auto ON \n");
    #endif
    //For this portal cube we want an altitude controller
    double z = sense_matrix.get(3,1);
    //Compute zdot
    double zdot = 0;
    if (zprev == -99) {
      zprev = z;
    } else {
      zdot = (z-zprev)/elapsedTime;
    }
    double zcommand = -50;
    double kpz = 100;
    double kdz = 50;
    double thrust_comm = kpz*(z-zcommand) + kdz*(zdot) + STICK_MIN;
    control_matrix.set(1,1,thrust_comm);
    //We are then going to code a roll and pitch contoller
    double roll = sense_matrix.get(4,1)*PI/180.0;
    double pitch = sense_matrix.get(5,1)*PI/180.0;  //convert to radians
    double yaw = sense_matrix.get(6,1)*PI/180.0;
    double p = sense_matrix.get(10,1);
    double q = sense_matrix.get(11,1);
    double r = sense_matrix.get(12,1)*PI/180.0;
    double kpE = -100;
    double kdE = -1000;
    double rollcommand = 0;
    double pitchcommand = 0;
    double yawcommand = 0;
    double roll_comm = kpE*(roll-rollcommand) + kdE*p + OUTMID;
    double pitch_comm = kpE*(pitch-pitchcommand) + kdE*q + OUTMID;
    double yaw_comm = kpE*(yaw-yawcommand) + kdE*r + OUTMID;
    control_matrix.set(2,1,roll_comm);
    control_matrix.set(3,1,pitch_comm);
    control_matrix.set(4,1,yaw_comm);
    //printf("Yaw Comm = %lf \n",ctlcomms.get(4,1));
  }
  //ctlcomms.disp();

}

