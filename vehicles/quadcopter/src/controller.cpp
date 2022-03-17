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

  //At a minimum you need to compute the 4 motor signals based on 
  //the standard acro mode
  double motor_upper_left = STICK_MIN;
  double motor_upper_right = STICK_MIN;
  double motor_lower_left = STICK_MIN;
  double motor_lower_right = STICK_MIN;		
  
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
    //STABILIZE MODE
    //printf(" STAB ");
    double roll_command = (aileron-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
    double pitch_command = -(elevator-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
    double yaw_rate_command = (rudder-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
    double roll = sense_matrix.get(4,1);
    double pitch = sense_matrix.get(5,1);
    double yaw = sense_matrix.get(6,1);
    double roll_rate = sense_matrix.get(10,1); //For SIL/SIMONLY see Sensors.cpp
    double pitch_rate = sense_matrix.get(11,1); //These are already in deg/s
    double yaw_rate = sense_matrix.get(12,1); //Check IMU.cpp to see for HIL
    //printf("PQR Rate in Controller %lf %lf %lf \n",roll_rate,pitch_rate,yaw_rate);
    double kp = 10.0;
    double kd = 2.0;
    double kyaw = 5.0;
    double droll = kp*(roll-roll_command) + kd*(roll_rate);
    droll = CONSTRAIN(droll,-500,500);
    double dpitch = kp*(pitch-pitch_command) + kd*(pitch_rate);
    dpitch = CONSTRAIN(dpitch,-500,500);
    double dyaw = kyaw*(yaw_rate-yaw_rate_command);
    //printf("YAW RATE = %lf YAW RATE COMMAND = %lf DYAW = %lf \n",yaw_rate,yaw_rate_command,dyaw);
    dyaw = CONSTRAIN(dyaw,-500,500);

    //Throttle controller on climb rate
    /*double climb_rate_command = (throttle-STICK_MID)*10/((STICK_MAX-STICK_MIN/2.0));
    //printf("Climb Rate Command = %lf \n",climb_rate_command);
    double climb_rate = sense_matrix.get(9,1);
    double kpt = 100.0;
    double dthrottle = -kpt*(climb_rate_command - climb_rate);*/
    //printf("dthrottle = %lf zdot = %lf \n",dthrottle,climb_rate);
    
    double altitude_command = -100.0;
    double z = sense_matrix.get(3,1);
    double zdot = sense_matrix.get(9,1);
    double kpt = 10.0;
    double kdt = 50.0;
    double dthrottle = kpt*(z-altitude_command) + kdt*(zdot);
    
    //printf("d = %lf %lf %lf ",droll,dpitch,dyaw);
    //printf(" Roll Command = %lf ",roll_command);
    motor_upper_left = throttle + dthrottle - droll - dpitch - dyaw;
    motor_upper_right = throttle + dthrottle + droll - dpitch + dyaw;
    motor_lower_left = throttle + dthrottle - droll + dpitch + dyaw;
    motor_lower_right = throttle + dthrottle + droll + dpitch - dyaw;
  } else {
    //ACRO MODE
    motor_upper_left = throttle + (aileron-STICK_MID) - (elevator-STICK_MID) + (rudder-STICK_MID);
    motor_upper_right = throttle - (aileron-STICK_MID) - (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_lower_left = throttle + (aileron-STICK_MID) + (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_lower_right = throttle - (aileron-STICK_MID) + (elevator-STICK_MID) + (rudder-STICK_MID);
  }
  
  //Send the motor commands to the control_matrix values
  control_matrix.set(1,1,motor_upper_left);
  control_matrix.set(2,1,motor_upper_right);
  control_matrix.set(3,1,motor_lower_left);
  control_matrix.set(4,1,motor_lower_right);
  //control_matrix.disp();
}
