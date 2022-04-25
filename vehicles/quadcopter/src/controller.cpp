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

  //First extract the relavent commands from the receiver.
  double throttle = rx_array[0];
  double aileron = rx_array[1];
  double elevator = rx_array[2];
  double rudder = rx_array[3];
  double autopilot = rx_array[4];
  int icontrol = 0;

  //Check for user controlled
  if (CONTROLLER_FLAG == -1) {
    if (autopilot > STICK_MID) {
      icontrol = 2;
    } else {
      icontrol = 0; 
    }
  } else {
    icontrol = CONTROLLER_FLAG;
  }

  //Quadcopter Control cases
  // 0 = fully manual (ACRO)
  // 1 = Inner loop on (roll and pitch on STAB)
  // 2 = Inner Loop + Yaw on RATE mode
  // 3 = Altitude control
  // 4 = Waypoint mode
  
  //Initialize commands
  roll_command = -99;
  pitch_command = -99;
  yaw_rate_command = -99;
  altitude_command = 100; //Hardcode to 100?
  //And controls
  droll = 0;
  dpitch = 0;
  dyaw = 0;
  dthrottle = 0;

  switch (icontrol) {
  case 4:
    //Waypoint Mode
    printf("Waypoint + ");
  case 3:
    //printf("Altitude + ");
    AltitudeLoop(sense_matrix);
  case 2:
    //printf("YawRateLoop + ");
    if (yaw_rate_command == -99) {
      yaw_rate_command = (rudder-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
    }
    YawRateLoop(sense_matrix);
  case 1:
    //printf("Inner Loop + ");
    //Run the Innerloop
    //Check to see if you need inner loop guidance or not
    if (roll_command == -99) {
      roll_command = (aileron-STICK_MID)*30.0/((STICK_MAX-STICK_MIN)/2.0);
    }
    if (pitch_command == -99) {
      pitch_command = -(elevator-STICK_MID)*30.0/((STICK_MAX-STICK_MIN)/2.0);
    }
    InnerLoop(sense_matrix);
  case 0:
    //printf("Motor Mixing \n");
    //Acro mode if controls not set
    if (droll == 0) {
      droll = (aileron-STICK_MID);
    }
    if (dpitch == 0) {
      dpitch = (elevator-STICK_MID);
    }
    if (dyaw == 0) {
      dyaw = (rudder-STICK_MID);
    }
    //This means control is off but we need a bit of thrust to stay in the air
    #ifndef SIL
    if (dthrottle == 0) {
      dthrottle = 1480-992;
    }
    #endif
    motor_upper_left = throttle + dthrottle - droll - dpitch - dyaw;
    motor_upper_right = throttle + dthrottle + droll - dpitch + dyaw;
    motor_lower_left = throttle + dthrottle - droll + dpitch + dyaw;
    motor_lower_right = throttle + dthrottle + droll + dpitch - dyaw;
    break;
  }
 //Send the motor commands to the control_matrix values
 control_matrix.set(1,1,motor_upper_left);
 control_matrix.set(2,1,motor_upper_right);
 control_matrix.set(3,1,motor_lower_left);
 control_matrix.set(4,1,motor_lower_right);
 //control_matrix.disp();
}

void controller::YawRateLoop(MATLAB sense_matrix) {
  double yaw_rate = sense_matrix.get(12,1); //Check IMU.cpp to see for HIL
  double kyaw = 5.0;
  dyaw = kyaw*(yaw_rate-yaw_rate_command);
  dyaw = CONSTRAIN(dyaw,-500,500);
}

void controller::AltitudeLoop(MATLAB sense_matrix) {
    //Throttle controller on climb rate
  /*double climb_rate_command = (throttle-STICK_MID)*10/((STICK_MAX-STICK_MIN/2.0));
  //printf("Climb Rate Command = %lf \n",climb_rate_command);
  double climb_rate = sense_matrix.get(9,1);
  double kpt = 100.0;
  double dthrottle = -kpt*(climb_rate_command - climb_rate);*/
  //printf("dthrottle = %lf zdot = %lf \n",dthrottle,climb_rate);
  double altitude  = sense_matrix.get(28,1);

  //Initialize altitude_dot to zero
  double altitude_dot = 0;
  //If altitude_prev has been set compute a first order derivative
  if (altitude_prev != -999) {
    altitude_dot = (altitude - altitude_prev) / elapsedTime;
  }
  //Then set the previous value
  altitude_prev = altitude;

  double kpt = -10.0;
  double kdt = -10.0;
  double kit = -1.0;
  double altitude_error = (altitude-altitude_command);
  dthrottle = kpt*altitude_error + kdt*(altitude_dot) + kit*altitude_int;

  //Integrate but prevent integral windup
  if ((dthrottle < (STICK_MAX-STICK_MIN)) && (dthrottle > 0)) {
    altitude_int += elapsedTime*altitude_error;
  }
  dthrottle = CONSTRAIN(dthrottle,-(STICK_MAX-STICK_MIN),(STICK_MAX-STICK_MIN));
  //dthrottle = 0;
  //printf("Altitude = %lf Altitude Dot = %lf dthrottle = %lf \n",altitude,altitude_dot,dthrottle);
}

void controller::InnerLoop(MATLAB sense_matrix) {
  //STABILIZE MODE
  //printf(" STAB ");
  double roll = sense_matrix.get(4,1);
  double pitch = sense_matrix.get(5,1);
  double roll_rate = sense_matrix.get(10,1); //For SIL/SIMONLY see Sensors.cpp
  double pitch_rate = sense_matrix.get(11,1); //These are already in deg/s
  //printf("PQR Rate in Controller %lf %lf %lf \n",roll_rate,pitch_rate,yaw_rate);
  double kp = 2.0;
  double kd = 10.0;
  droll = kp*(roll-roll_command) + kd*(roll_rate);
  droll = CONSTRAIN(droll,-500,500);
  dpitch = kp*(pitch-pitch_command) + kd*(pitch_rate);
  dpitch = CONSTRAIN(dpitch,-500,500);    
  //printf("d = %lf %lf %lf ",droll,dpitch,dyaw);
  //printf(" Roll Command = %lf ",roll_command);
}
