//This is an x8 quad with 4 rotors on top and 4 rotors on bottom
//Check notes at bottom of file for useful information

#include "x8_controller.h"

/////////////////////////////////////////////////////////////Start Controller Class//////////////////////////////////////////////////////////////
//controller class constructor - sets system parameters and sets up motors
controller::controller() {
};

//Function to initialize control matrix
void controller::init(MATLAB in_configuration_matrix) {
  control_matrix.zeros(NUMSIGNALS,1,"Control Signals"); //The standards must be TAERA1A2A3A4
  set_defaults();
  printf("Controller Received Configuration Matrix \n");
  //in_configuration_matrix.disp();
  CONTROLLER_FLAG = in_configuration_matrix.get(11,1);
  printf("Controller Setup \n");
}

//Set control matrix to minimum pwm
void controller::set_defaults() {
  control_matrix.set(1,1,OUTMIN);
  control_matrix.set(2,1,OUTMIN);
  control_matrix.set(3,1,OUTMIN);
  control_matrix.set(4,1,OUTMIN);
  control_matrix.set(5,1,OUTMIN);
  control_matrix.set(6,1,OUTMIN);
  control_matrix.set(7,1,OUTMIN);
  control_matrix.set(8,1,OUTMIN);
}

//Print control matrix
void controller::print() {
  for (int i = 1;i<=NUMSIGNALS;i++) {
    printf("%d ",int(control_matrix.get(i,1)));
  }
}

//Main controller loop
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

  //First extract the relevant commands from the receiver.
  throttle = rx_array[0];
  aileron = rx_array[1];
  elevator = rx_array[2];
  rudder = rx_array[3];
  autopilot = rx_array[4]; //Autopilot - OUTMIN = cutoff, OUTMID = ACRO, OUTMAX = AUTOPILOT
  bool icontrol = 0;

  //Debug
  //printf("rx [5] [6] [7] [8] %lf %lf %lf %lf \n",rx_array[5], rx_array[6], rx_array[7], rx_array[8]);

  switch (CONTROLLER_FLAG) {
  case -1:
    //User decides
    if (autopilot > 1.2 * STICK_MID) {
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

  roll_command = -99;
  pitch_command = -99;
  yaw_command = -99;
  altitude_command = -99;

  //Then you can run any control loop you want.
  switch (icontrol) {   
    case 2:
      //Run the altitude loop
      if (altitude_command == -99) {
        altitude_command = 50; //meters
      }
      AltitudeLoop(sense_matrix);
    case 1:
      //Run the Attitude Loop
      if (roll_command == -99) {
        roll_command = (aileron-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
      }
      if (pitch_command == -99) {
        pitch_command = -(elevator-STICK_MID)*30.0/((STICK_MAX-STICK_MIN)/2.0);
      }
      if (yaw_command == -99) {
        yaw_command = (rudder-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
      }
      AttitudeLoop(sense_matrix);
    case 0:
    //ACRO MODE - Unstable, so replacing with old stabilize mode
    motor_upper_left_bottom = throttle + (aileron-STICK_MID) - (elevator-STICK_MID) + (rudder-STICK_MID);
    motor_upper_right_bottom = throttle - (aileron-STICK_MID) - (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_lower_right_bottom = throttle - (aileron-STICK_MID) + (elevator-STICK_MID) + (rudder-STICK_MID);
    motor_lower_left_bottom = throttle + (aileron-STICK_MID) + (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_upper_left_top = throttle + (aileron-STICK_MID) - (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_upper_right_top = throttle - (aileron-STICK_MID) - (elevator-STICK_MID) + (rudder-STICK_MID);
    motor_lower_right_top = throttle - (aileron-STICK_MID) + (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_lower_left_top = throttle + (aileron-STICK_MID) + (elevator-STICK_MID) + (rudder-STICK_MID);
    break;
  }
  
  //Send the motor commands to the control_matrix values
  //control_matrix.mult_eq(0);
  //control_matrix.plus_eq(STICK_MIN);

  control_matrix.set(1, 1, motor_upper_left_top);     //OUTMIN); //motor_upper_left_top);
  control_matrix.set(2, 1, motor_upper_right_top);    //OUTMIN); //motor_upper_right_top);
  control_matrix.set(3, 1, motor_lower_right_top);    //OUTMIN); //motor_lower_right_top);
  control_matrix.set(4, 1, motor_lower_left_top);     //OUTMIN); //motor_lower_left_top);
  control_matrix.set(5, 1, motor_upper_left_bottom);  //OUTMIN); //motor_upper_left_bottom);
  control_matrix.set(6, 1, motor_upper_right_bottom); //OUTMIN); //motor_upper_right_bottom);
  control_matrix.set(7, 1, motor_lower_right_bottom); //OUTMIN); //motor_lower_right_bottom);
  control_matrix.set(8, 1, motor_lower_left_bottom);  //OUTMIN); //motor_lower_left_bottom);

  //Debug
  /*  for (int i = 0;i<5;i++) {
    printf(" %d ",rx_array[i]);
  }
  printf("\n");*/
  //control_matrix.disp();
}

void controller::AttitudeLoop(MATLAB sense_matrix) {
  //STABILIZE MODE
  double roll = sense_matrix.get(4,1);
  double pitch = sense_matrix.get(5,1);
  double yaw = sense_matrix.get(6,1);
  double roll_rate = sense_matrix.get(10,1); //For SIL/SIMONLY see Sensors.cpp
  double pitch_rate = sense_matrix.get(11,1); //These are already in deg/s
  double yaw_rate = sense_matrix.get(12,1); //Check IMU.cpp to see for HIL
  //state.disp();
  //printf("PQR Rate in Controller %lf %lf %lf \n",roll_rate,pitch_rate,yaw_rate);
  double kp = 10.0;
  double kd = 2.0;
  double kyaw = 0.2;
  double droll = kp*(roll-roll_command) + kd*(roll_rate);
  droll = CONSTRAIN(droll,-500,500);
  double dpitch = kp*(pitch-pitch_command) + kd*(pitch_rate);
  dpitch = CONSTRAIN(dpitch,-500,500);
  double dyaw = kp*(yaw-yaw_command) + kyaw*(yaw_rate);
  dyaw = CONSTRAIN(dyaw,-500,500);
  //printf("d = %lf %lf %lf ",droll,dpitch,dyaw);
  aileron = droll + OUTMID;
  elevator = dpitch + OUTMID;
  rudder = dyaw + OUTMID;
}

void controller::AltitudeLoop(MATLAB sense_matrix) {
  //Probably a good idea to use pressure altitude but might need to use 
  //GPS altitude if the barometer isn't good or perhaps even a KF approach
  //Who knows. Just simulating this now.
  double altitude = sense_matrix.get(28,1); //This is 28,1 which is baro altitude
  //Initialize altitude_dot to zero
  double altitude_dot = 0;
  //If altitude_prev has been set compute a first order derivative
  if (altitude_prev != -999) {
    altitude_dot = (altitude - altitude_prev) / elapsedTime;
  }
  //Then set the previous value
  altitude_prev = altitude;

  //Compute Pitch Command in Degrees
  double kp = 1.0;
  double kd = 0.5;
  double dup = kp*(altitude_command - altitude) + kd*(0-altitude_dot);
  throttle = dup + OUTMIN;
  //printf("T, ALT, ALT DOT = %lf %lf %lf \n",lastTime,altitude,altitude_dot);  
}