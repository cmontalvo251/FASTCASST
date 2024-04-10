//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "hybrid_controller.h"

controller::controller() {
}

void controller::init(int CONTROLLERTYPE_IN) {
  CONTROLLER_FLAG = CONTROLLERTYPE_IN;
  control_matrix.zeros(NUMSIGNALS,1,"Control Signals"); //The standards must be TAERA1A2A3A4
  set_defaults();
  printstdout("Controller Received Configuration Matrix \n");
  printstdout("Controller Setup \n");
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
  //Think about what defaults you want OUTMIN, OUTMID, OUTMAX
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
  double rx_throttle = rx_array[0];
  double rx_aileron = rx_array[1];
  double rx_elevator = rx_array[2];
  double rx_rudder = rx_array[3];
  double rx_arm_switch = rx_array[4];
  double rx_autopilot = rx_array[5];
  int icontrol = 0;

  //Check for user controlled
  if (CONTROLLER_FLAG < 0) {
    if (rx_autopilot > STICK_MID) {
      icontrol = -CONTROLLER_FLAG;
    } else {
      icontrol = 0;
    }
  } else {
    icontrol = CONTROLLER_FLAG;
  }

  //Now this is where things get weird. If we're running in SIMONLY or
  //SIL mode, There is no flight controller for the quad. This means that the
  //rx_array above needs to run through
  //the flight control system below before being sent out to the motors. So we need a #ifdef
  //to convert the rx signals above to motor signals
  #if defined (SIMONLY) || (SIL)
  autoloop(currentTime,rx_array,sense_matrix,icontrol);
  #else
  //Otherwise we just send those signals straight out to the flight controller
  //FOR THIS HYBRID VEHICLE WE HAVE THROTTLE,AILERON,ELEVATOR,RUDDER and ARM SWITCH PASS DIRECTLY TO THE QUAD'S 
  //FLIGHT CONTROLLER
  for (int i = 0;i<5;i++) {
    control_matrix.set(i+1,1,rx_array[i]);
  }
  #endif

  //NOW THIS HYBRID VEHICLE HAS 3 EXTRA SERVOS So....we need an innerloop controller from the airplane codes
  //WE WILL JUST COPY THEM HERE SO THEY ARE DIFFERENT
  //Note that the Airplane autopilot though needs some roll,pitch and yaw commands
  roll_command = (rx_aileron-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
  pitch_command = -(rx_elevator-STICK_MID)*30.0/((STICK_MAX-STICK_MIN)/2.0);
  //THen we run innnerloop on the airplane
  InnerLoop_AIRPLANE(sense_matrix);

  //Then those last 3 signals can be sent to the control_matrix
  //1 - quad throttle 
  //2 - quad aileron
  //3 - quad elevator
  //4 - quad rudder
  //5 - quad arm switch
  //6 - airplane aileron
  control_matrix.set(6,1,airplane_aileron);
  //7 - airplane elevator
  control_matrix.set(7,1,airplane_elevator);
  //8 - airplane rudder 
  control_matrix.set(8,1,airplane_rudder);
}

void controller::autoloop(double currentTime,int rx_array[],MATLAB sense_matrix,int icontrol) {
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
  // 1 = Inner loop on (roll and pitch on STAB and Yaw on Rate mode)
  
  //printf("Running Autoloop (icontrol) = %d ",icontrol);
  //printf("TAERAG = ");
  //for (int i = 0;i<6;i++) {
  //printf("%d ",rx_array[i]);
  //}
  //printf("\n");

  switch (icontrol) {
  case 1:
    //printf("Inner Loop + ");
    //printf("YawRateLoop + ");
    if (yaw_rate_command == -99) {
      yaw_rate_command = (rudder-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
    }
    YawRateLoop_QUAD(sense_matrix);
    //Run the Innerloop
    //Check to see if you need inner loop control or not
    if (roll_command == -99) {
      roll_command = (aileron-STICK_MID)*30.0/((STICK_MAX-STICK_MIN)/2.0);
    }
    if (pitch_command == -99) {
      pitch_command = -(elevator-STICK_MID)*30.0/((STICK_MAX-STICK_MIN)/2.0);
    }
    InnerLoop_QUAD(sense_matrix);
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
      //dthrottle = 1480-992;
      dthrottle = 0; //Reinvestigate this. I don't like this for auto flying
    }
    #endif
    motor_lower_right = throttle + dthrottle - droll + dpitch + dyaw;
    motor_upper_right = throttle + dthrottle - droll - dpitch - dyaw;
    motor_lower_left = throttle + dthrottle + droll + dpitch - dyaw;
    motor_upper_left = throttle + dthrottle + droll - dpitch + dyaw;
    break;
  }
  //Send the motor commands to the control_matrix values only if the
  //arm_switch is set
  if (arm_switch > STICK_MID) {
    control_matrix.set(1,1,motor_lower_right);
    control_matrix.set(2,1,motor_upper_right);
    control_matrix.set(3,1,motor_lower_left);
    control_matrix.set(4,1,motor_upper_left);
    control_matrix.set(5,1,arm_switch);
  }
  //control_matrix.disp();
}

void controller::YawRateLoop_QUAD(MATLAB sense_matrix) {
  double yaw_rate = sense_matrix.get(12,1); //Check IMU.cpp to see for HIL
  double kyaw = 50.0;
  dyaw = kyaw*(yaw_rate-yaw_rate_command);
  dyaw = -CONSTRAIN(dyaw,-500,500);
}

void controller::InnerLoop_QUAD(MATLAB sense_matrix) {
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
  //printf("d = %lf %lf %lf %lf \n",droll,dpitch,roll_command,pitch_command);
  //printf(" Roll Command = %lf ",roll_command);
}

void controller::InnerLoop_AIRPLANE(MATLAB sense_matrix) {
    //Get States
    double roll = sense_matrix.get(4,1);
    double pitch = sense_matrix.get(5,1);
    //printf("PITCH = %lf \n",pitch);
    double roll_rate = sense_matrix.get(10,1); //For SIL/SIMONLY see Sensors.cpp
    double pitch_rate = sense_matrix.get(11,1); //These are already in deg/s
    //printf("PQR Rate in Controller %lf %lf %lf \n",roll_rate,pitch_rate,yaw_rate);

    //ORIGINAL GAINS 
    //kpa = 10.0;
    //kda = 2.0;
    //kpe = 25.0;
    //kde = 1.0;
    //kr = 1.0;
    //SIMONLY GAINS
    //kpa = 5.0
    //kda = 0.1
    //kpe = 10.0
    //kde = 0.5
    //kr = 0.1

    //AIRPLANE Airplane airplane
    double kpa = 10.0;
    double kda = 2.0;
    double kpe = 10.0; //Tested Meta 1 with 25 and 1.0 and kinda wobbly so lowering gains to 10 and 0.5 - 11/12/2023
    double kde = 0.5; 
    double kr = 1.0;
    
    //Apprentice APPRENTICE apprentice
    /*double kpa = 5.0;
    double kda = 0.1;
    double kpe = 10.0;
    double kd = 0.5;
    double kr = 0.1;*/

    airplane_aileron = kpa*(roll-roll_command) + kda*(roll_rate);
    airplane_elevator = kpe*(pitch-pitch_command) + kde*(pitch_rate);

    //Rudder signal will be proportional to aileron
    airplane_rudder = kr*airplane_aileron;

    //CONSTRAIN
    airplane_rudder = CONSTRAIN(airplane_rudder,-500,500) + OUTMID;
    airplane_aileron = -CONSTRAIN(airplane_aileron,-500,500) + OUTMID;
    airplane_elevator = CONSTRAIN(airplane_elevator,-500,500) + OUTMID;
}
