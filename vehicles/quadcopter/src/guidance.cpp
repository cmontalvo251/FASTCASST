#include "guidance.h"

guidance::guidance() {
  //Constructor
}

guidance::init() {
  guidance_matrix.zeros(4,1,"Guidance Matrix");
  printstdout("Guidance Module Initialized \n");
}

guidance::pass_through(int rx_array,MATLAB control_matrix) {
  //Signals are
  // 0 = throttle
  // 1 = aileron
  // 2 = elevator
  // 3 = rudder
  // 4 = arm switch
  // 5 = autopilot which is not used by the flight controller
  for (int i = 0;i<5;i++) {
    control_matrix.set(i+1,1,rx_array[0]);
  }
}

guidance::loop(double currentTime,MATLAB sense_matrix) {
  //I want to keep track of timeElapsed so that I can run integrators
  //and compute derivates
  elapsedTime = currentTime - lastTime;
  lastTime = currentTime;
  
  //I'm assuming if this routine is running the guidance module is on
  //First create the commands
  double roll_command = 0;
  double pitch_command = 0;
  double yaw_rate_command = 0;
  double altitude_command = 100; //eventually will need to update with telemetry

  //But we need to convert that to a dthrottle command
  //So we run the Altitude Command Loop
  AltitudeLoop(sense_matrix); //Now dthrottle is populated
  //When Guidance is on we actually need to pass through controls
  //rather than sending motor commands.
  //First command is throttle augmented with dthrottle
  guidance_matrix.set(1,1,throttle + dthrottle);
  //Second command is aileron
  double aileron = roll_command*((STICK_MAX-STICK_MIN)/2.0)/30.0+STICK_MID;
  guidance_matrix.set(2,1,aileron);
  //Third command is elevator
  double elevator = pitch_command*((STICK_MAX-STICK_MIN)/2.0)/30.0+STICK_MID;
  guidance_matrix.set(3,1,elevator);
  //Fourth command is rudder
  double rudder = yaw_rate_command*((STICK_MAX-STICK_MIN)/2.0)/50.0+STICK_MID;
  guidance_matrix.set(4,1,rudder);
}

void guidance::AltitudeLoop(MATLAB sense_matrix) {
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
  //Altitude command set with the guidance laws or hardcoded in preamble
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

