//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "controller.h"

controller::controller() {
}

void controller::init(int CONTROLLERTYPE_IN) {
  CONTROLLER_FLAG = CONTROLLERTYPE_IN;
  control_matrix.zeros(NUMSIGNALS,1,"Control Signals"); //The standards must be TAERA1A2A3A4
  set_defaults();
  printstdout("Controller Received Configuration Matrix \n");
  printstdout("Controller Setup \n");
  if (abs(CONTROLLER_FLAG) >= 10) {
    guid.init();
  }

}

void controller::init(MATLAB in_configuration_matrix) {
  //in_configuration_matrix.disp();
  int CONTROLLERTYPE_IN = in_configuration_matrix.get(11,1);
  init(CONTROLLERTYPE_IN);
}

void controller::set_defaults() {
  control_matrix.set(1,1,OUTMIN);
  control_matrix.set(2,1,OUTMIN);
  control_matrix.set(3,1,OUTMIN);
  control_matrix.set(4,1,OUTMIN);
  control_matrix.set(5,1,OUTMIN);
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

  //Extract the autopilot flag -- autopilot is only used for controller.cpp
  //throttle = rx_array[0];
  //aileron = rx_array[1];
  //elevator = rx_array[2];
  //rudder = rx_array[3];
  //arm_switch = rx_array[4];
  double autopilot = rx_array[5];
  int icontrol = 0,iguidance = 0;

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

  if (abs(CONTROLLER_FLAG) >= 10) {
    iguidance=1;
    if (icontrol >= 10) {
      icontrol-=10;
    }
  }

  //At this point icontrol < 10 and iguidance is a 0 or a 1
  if (iguidance == 1) {
    //There are a couple scenarios here since guidance is on

    //The autopilot switch is up (I want to be able to independtly
    //control the flight controller logic and the guidance module
    if (autopilot > STICK_MID) {
      //This means we need to run the guidance module
      //printstdout("Running guidance loop \n");
      guid.loop(rx_array,currentTime,sense_matrix);
    } else { 
      guid.anti_windup();
    }
    //Then we pass the signals through to the control matrix
    //If the autopilot is down the rx_array just goes straight to the control_matrix
    //Otherwise the loop above sets the receiver array to something different 
    //and then that is sent to the control_matrix  
    for (int i = 0;i<5;i++) {
      control_matrix.set(i+1,1,rx_array[i]);
    }  
  }

  //Now this is where things get weird. If we're running in SIMONLY or
  //SIL mode, There is no flight controller. This means that the
  //rx_array either altered or unaltered above needs to run through
  //the flight control system below. So we need a #ifdef
  //If we're running in AUTO or anything but SIMONLY or SIL the
  //routine just breaks 
  
  #if defined (SIMONLY) || (SIL)

  //Initialize commands
  roll_command = -99;
  pitch_command = -99;
  yaw_rate_command = -99;
  //And controls
  droll = -99;
  dpitch = -99;
  dyaw = -99;
  dthrottle = -99;

  //Extract the relavent commands from the receiver.
  double throttle = rx_array[0];
  double aileron = rx_array[1];
  double elevator = rx_array[2];
  double rudder = rx_array[3];
  double arm_switch = rx_array[4];

  //Quadcopter Control cases
  // 0 = fully manual (ACRO)
  // 1 = Inner loop on (roll and pitch on STAB)
  // 2 = Inner Loop + Yaw on RATE mode
  // Note that Altitude and waypoint loops moved to guidance
  // 1X = This means we want the control module here to create
  // commands to send to a betaflight/cleanflight flight controller

  switch (icontrol) {
  case 2:
    //printf("YawRateLoop + ");
    if (yaw_rate_command == -99) {
      yaw_rate_command = (rudder-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
    }
    YawRateLoop(sense_matrix);
  case 1:
    //printf("Inner Loop + ");
    //Run the Innerloop
    //Check to see if you need inner loop control or not
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
    if (droll == -99) {
      droll = (aileron-STICK_MID);
    }
    if (dpitch == -99) {
      dpitch = (elevator-STICK_MID);
    }
    if (dyaw == -99) {
      dyaw = (rudder-STICK_MID);
    }
    //This means control is off but we need a bit of thrust to stay in the air
    #ifndef SIL
    if (dthrottle == -99) {
      dthrottle = 1480-992;
    }
    #endif
    motor_lower_right = throttle + dthrottle - droll + dpitch + dyaw;
    motor_upper_right = throttle + dthrottle - droll - dpitch - dyaw;
    motor_lower_left = throttle + dthrottle + droll + dpitch - dyaw;
    motor_upper_left = throttle + dthrottle + droll - dpitch + dyaw;
    break;
  }
 //Send the motor commands to the control_matrix values
 control_matrix.set(1,1,motor_lower_right);
 control_matrix.set(2,1,motor_upper_right);
 control_matrix.set(3,1,motor_lower_left);
 control_matrix.set(4,1,motor_upper_left);
 control_matrix.set(5,1,arm_switch);
 //control_matrix.disp();

 #endif //SIMONLY or SIL
}

void controller::YawRateLoop(MATLAB sense_matrix) {
  double yaw_rate = sense_matrix.get(12,1); //Check IMU.cpp to see for HIL
  double kyaw = 50.0;
  dyaw = kyaw*(yaw_rate-yaw_rate_command);
  dyaw = -CONSTRAIN(dyaw,-500,500);
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
  droll = -CONSTRAIN(droll,-500,500);
  dpitch = kp*(pitch-pitch_command) + kd*(pitch_rate);
  dpitch = CONSTRAIN(dpitch,-500,500);    
  //printf("d = %lf %lf %lf ",droll,dpitch,dyaw);
  //printf(" Roll Command = %lf ",roll_command);
}
