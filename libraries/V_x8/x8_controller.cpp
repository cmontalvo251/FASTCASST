//This is an x8 quad with 4 rotors on top and 4 rotors on bottom
//Check notes at bottom of file for useful information

#include "x8_controller.h"

/////////////////////////////////////////////////////////////Start Controller Class//////////////////////////////////////////////////////////////
//controller class constructor - sets system parameters and sets up motors
controller::controller() {
  WAYINDEX = 0;
  PHASE = 0;
  elapsedTime = 0.0;
  lastTime = 0.0;
  altitude_int = 0.0;
  velocity_int = 0.0;
  velocity_intX = 0.0;
  velocity_intY = 0.0;
  altitude_prev = -999.0;
  distance_dot_X = 0.0;
  distance_dot_Y = 0.0;
  distance_prev_X = -999.0;
  distance_prev_Y = -999.0;
  prevTime = 0.0;
};

//Function to initialize control matrix
void controller::init(MATLAB in_configuration_matrix) {
  control_matrix.zeros(NUMSIGNALS,1,"Control Signals"); //The standards must be TAERA1A2A3A4
  set_defaults();
  printf("Controller Received Configuration Matrix \n");
  //in_configuration_matrix.disp();
  CONTROLLER_FLAG = in_configuration_matrix.get(11,1);
  //printf("CONTROLLER FLAG = %d \n",CONTROLLER_FLAG);

  if (in_configuration_matrix.length() >= 12) {
    NUMWAYPOINTS = in_configuration_matrix.get(12,1);
    WAYPOINTS_X.resize(NUMWAYPOINTS);
    WAYPOINTS_Y.resize(NUMWAYPOINTS);
    int idx = 13;
    for (int i = 0; i < NUMWAYPOINTS; i++) {
      WAYPOINTS_X[i] = in_configuration_matrix.get(idx++, 1);
      WAYPOINTS_Y[i] = in_configuration_matrix.get(idx++, 1);
    }
  }

  WAYINDEX = 0;
  PHASE = 0;
  velocity_intX = 0.0;
  velocity_intY = 0.0;
  distance_prev_X = -999.0;
  distance_prev_Y = -999.0;
  distance_dot_X = 0.0;
  distance_dot_Y = 0.0;
  prevTime = 0.0;

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
  autopilot = rx_array[5]; //Autopilot - OUTMIN = cutoff, OUTMID = ACRO, OUTMAX = AUTOPILOT
  int icontrol = 0;

  //Debug
  //printf("rx [5] [6] [7] [8] %lf %lf %lf %lf \n",rx_array[5], rx_array[6], rx_array[7], rx_array[8]);

 //Check for user controlled
  if (CONTROLLER_FLAG < 0) {
    if (autopilot > STICK_MID) {
      icontrol = -CONTROLLER_FLAG;
      //printf("ICONTROL = %d \n",icontrol);
    } else {
      icontrol = 0;
    }
  } else {
    icontrol = CONTROLLER_FLAG;
  }

  //printf("CONTROLLER FLAG = %d \n",CONTROLLER_FLAG);
  //printf("ICONTROL = %d \n",icontrol);

  roll_command = -99;
  pitch_command = -99;
  yaw_command = -99;
  altitude_command = -99;
  velocity_command = -99;
  velocity_command_X = -99;
  velocity_command_Y = -99;

  //Then you can run any control loop you want.
  switch (icontrol) { 
    case 4:
      //Run the Waypoint Loop
      WaypointLoop(sense_matrix,currentTime);
    case 3:
      //Run the velocity loop
      if (velocity_command_X == -99) {
        if (velocity_command != -99) {
          velocity_command_X = velocity_command;
        } else {
          velocity_command_X = 10.0; //m/s
        }
        velocity_command_Y = 0.0;
      }
      VelocityLoop(sense_matrix);
    case 2:
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
    case 1:
      //Run the altitude loop
      if (altitude_command == -99) {
        altitude_command = 200; //meters
      }
      //printf("Altitude Loop + \n");
      AltitudeLoop(sense_matrix);
    case 0:
    motor_upper_left_bottom = throttle - (aileron-OUTMID) - (elevator-OUTMID) + (rudder-OUTMID);
    motor_upper_right_bottom = throttle + (aileron-OUTMID) - (elevator-OUTMID) - (rudder-OUTMID);
    motor_lower_right_bottom = throttle + (aileron-OUTMID) + (elevator-OUTMID) + (rudder-OUTMID);
    motor_lower_left_bottom = throttle - (aileron-OUTMID) + (elevator-OUTMID) - (rudder-OUTMID);
    motor_upper_left_top = throttle - (aileron-OUTMID) - (elevator-OUTMID) - (rudder-OUTMID);
    motor_upper_right_top = throttle + (aileron-OUTMID) - (elevator-OUTMID) + (rudder-OUTMID);
    motor_lower_right_top = throttle + (aileron-OUTMID) + (elevator-OUTMID) - (rudder-OUTMID);
    motor_lower_left_top = throttle - (aileron-OUTMID) + (elevator-OUTMID) + (rudder-OUTMID);
    break;
  }
  
  //Send the motor commands to the control_matrix values
  control_matrix.set(1, 1, motor_upper_left_bottom);   
  control_matrix.set(2, 1, motor_upper_right_bottom);    
  control_matrix.set(3, 1, motor_lower_right_bottom);    
  control_matrix.set(4, 1, motor_lower_left_bottom);     
  control_matrix.set(5, 1, motor_upper_left_top);  
  control_matrix.set(6, 1, motor_upper_right_top); 
  control_matrix.set(7, 1, motor_lower_right_top); 
  control_matrix.set(8, 1, motor_lower_left_top);  

  //Constrain the control matrix to be within the min and max values
  for (int i = 1;i<=NUMSIGNALS;i++) {
    double val = control_matrix.get(i,1);
    val = CONSTRAIN(val,OUTMIN,OUTMAX);
    control_matrix.set(i,1,val);
  }

  //Debug
  /*  for (int i = 0;i<5;i++) {
    printf(" %d ",rx_array[i]);
  }
  printf("\n");*/
  //printf("Throttle = %lf Ail = %lf Elev = %lf Rudd = %lf \n",throttle,aileron,elevator,rudder);
  //control_matrix.disp();
  //PAUSE();
}

void controller::WaypointLoop(MATLAB sense_matrix,double currentTime) {
  if (NUMWAYPOINTS == 0 || WAYPOINTS_X.empty() || WAYPOINTS_Y.empty()) {
    return;
  }

  // Extract relevant parameters
  double LAT = sense_matrix.get(16,1);
  double LON = sense_matrix.get(17,1);
  double X = sense_matrix.get(1,1);
  double Y = sense_matrix.get(2,1);
  double speed = sense_matrix.get(7,1);
  double yaw = sense_matrix.get(20,1) * PI / 180.0;

  double LAT_COMMAND, LON_COMMAND, DLAT, DLON;

  // Determine whether WAYPOINTS are in local Cartesian meters (X, Y) or GPS degrees (LAT, LON)
  if (fabs(WAYPOINTS_X[WAYINDEX]) > 90.0 || (fabs(LAT) > 1.0 && fabs(WAYPOINTS_X[WAYINDEX] - LAT) > 5.0)) {
    // Local meters (X, Y)
    double distanceX_raw = WAYPOINTS_X[WAYINDEX] - X;
    double distanceY_raw = WAYPOINTS_Y[WAYINDEX] - Y;
    DLAT = distanceX_raw / GPSVAL;
    DLON = distanceY_raw / GPSVAL;
    LAT_COMMAND = LAT + DLAT;
    LON_COMMAND = LON + DLON;
  } else {
    // GPS coordinates (LAT, LON)
    LAT_COMMAND = WAYPOINTS_X[WAYINDEX];
    LON_COMMAND = WAYPOINTS_Y[WAYINDEX];
    DLAT = LAT_COMMAND - LAT;
    DLON = LON_COMMAND - LON;
  }

  // COMPUTE HEADING TO TARGET
  // HEADING ANGLE COMPUTATION FROM GEMINI
  double phi2 = LAT_COMMAND * PI / 180.0;
  double phi1 = LAT * PI / 180.0;
  double delta_lambda = DLON * PI / 180.0;
  // Formula components
  double y = sin(delta_lambda) * cos(phi2);
  double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lambda);
  // Calculate initial bearing in radians and convert to degrees
  double headingToTarget = atan2(y, x) * 180.0 / PI;
  // And delta to target
  double dheading = fabs(delpsi(yaw, headingToTarget * PI / 180.0) * 180.0 / PI);

  // COMPUTE DISTANCE TO TARGET
  double distanceToTarget = sqrt(DLAT * DLAT + DLON * DLON) * GPSVAL;

  // COMPUTE DISTANCE DOT
  double distanceX = DLAT * GPSVAL;
  double distanceY = DLON * GPSVAL;
  double distanceBodyX = distanceX * cos(yaw) + distanceY * sin(yaw);
  double distanceBodyY = -distanceX * sin(yaw) + distanceY * cos(yaw);

  if (distance_prev_X != distanceBodyX) {
    if (distance_prev_X != -999.0 && (currentTime - prevTime) > 0) {
      distance_dot_X = (distanceBodyX - distance_prev_X) / (currentTime - prevTime);
    }
    distance_prev_X = distanceBodyX;
    if (distance_prev_Y != -999.0 && (currentTime - prevTime) > 0) {
      distance_dot_Y = (distanceBodyY - distance_prev_Y) / (currentTime - prevTime);
    }
    distance_prev_Y = distanceBodyY;
    prevTime = currentTime;
  }

  //printf("%lf %d %d %lf %lf\n", currentTime, PHASE, WAYINDEX, distanceToTarget, dheading);

  if (PHASE == 0) {
    // POINT TO TARGET!!!
    yaw_command = headingToTarget;
    velocity_command_X = 0.0;
    velocity_command_Y = 0.0;
    // If we're not moving and pointing towards target proceed to phase 1
    if (dheading < 0.1 && fabs(speed) < 0.1) {
      PHASE = 1;
      //printf("Moving to Phase 1\n");
    }
    return;
  }

  if (PHASE == 1) {
    // INCREASE SPEED TOWARDS TARGET!!!
    // Velocity PID
    double kp = 20.0 / 200.0;
    double kd = 100.0 / 200.0;
    velocity_command_X = kp * distanceBodyX + kd * distance_dot_X;
    velocity_command_Y = kp * distanceBodyY + kd * distance_dot_Y;
    double topSpeed = 10.0;
    velocity_command_X = CONSTRAIN(velocity_command_X, 0.0, topSpeed);
    velocity_command_Y = CONSTRAIN(velocity_command_Y, 0.0, topSpeed);
    yaw_command = headingToTarget; // make sure we're still pointing at waypoint
    // if we've arrived at the target move to phase 2
    if (fabs(distanceToTarget) < 20.0) {
      PHASE = 2;
    }
    return;
  }

  if (PHASE == 2) {
    // STOP AND REMAIN POINTED IN THE SAME DIRECTION
    velocity_command_X = 0.0;
    velocity_command_Y = 0.0;
    // yaw_command remains unchanged from previous PHASE
    // Once we have stopped moving we can proceed to the next waypoint and PHASE 0
    if (fabs(speed) < 0.1) {
      WAYINDEX++;
      if (WAYINDEX > NUMWAYPOINTS - 1) {
        WAYINDEX = 0;
      }
      PHASE = 0;
    }
    return;
  }
}

void controller::VelocityLoop(MATLAB sense_matrix) {
  double u = sense_matrix.get(7,1);
  double v = sense_matrix.get(8,1);

  double velocityerrorX = velocity_command_X - u;
  double velocityerrorY = velocity_command_Y - v;
  double kp = 1.0;
  double ki = 0.0; // 8.0 * 0
  pitch_command = -kp * velocityerrorX - ki * velocity_intX;
  pitch_command = CONSTRAIN(pitch_command, -20.0, 20.0);
  roll_command = kp * velocityerrorY + ki * velocity_intY;
  roll_command = CONSTRAIN(roll_command, -20.0, 20.0);

  // Integrate but prevent integral windup
  if ((pitch_command > -20.0) && (pitch_command < 20.0)) {
    velocity_intX += elapsedTime * velocityerrorX;
  }
  if ((roll_command > -20.0) && (roll_command < 20.0)) {
    velocity_intY += elapsedTime * velocityerrorY;
  }
}

void controller::AttitudeLoop(MATLAB sense_matrix) {
  //STABILIZE MODE
  double roll = sense_matrix.get(4,1);
  double pitch = sense_matrix.get(5,1);
  double yaw = sense_matrix.get(20,1); //6,1 is compass which is gps + imu, 20 is just imu yaw
  double roll_rate = sense_matrix.get(10,1); //For SIL/SIMONLY see Sensors.cpp
  double pitch_rate = sense_matrix.get(11,1); //These are already in deg/s
  double yaw_rate = sense_matrix.get(12,1); //Check IMU.cpp to see for HIL
  //state.disp();
  //printf("PQR Rate in Controller %lf %lf %lf \n",roll_rate,pitch_rate,yaw_rate);
  double kp = 10.0;
  double kd = 2.0;
  double kpyaw = 10.0;
  double kdyaw = 5.0;
  //roll_command = 20;
  double droll = kp*(roll-roll_command) + kd*(roll_rate);
  droll = CONSTRAIN(droll,-500,500);
  //pitch_command = 20;
  double dpitch = kp*(pitch-pitch_command) + kd*(pitch_rate);
  dpitch = CONSTRAIN(dpitch,-500,500);
  //yaw_command = 45;
  //Fix the wrap issue here
  double dheading = delpsi(yaw*PI/180.0,yaw_command*PI/180.0)*180.0/PI;
  if (dheading > 180) {
    dheading -= 180;
    dheading *= -1;
  }
  if (dheading < -180) {
    dheading += 180;
    dheading *= -1;
  }
  double dyaw = kpyaw*dheading + kdyaw*(yaw_rate);
  dyaw = CONSTRAIN(dyaw,-500,500);
  //printf("d = %lf %lf %lf ",droll,dpitch,dyaw);
  aileron = droll + OUTMID;
  elevator = dpitch + OUTMID;
  rudder = dyaw + OUTMID;
  //printf("AIL, ELEV, RUDD = %lf %lf %lf \n",aileron,elevator,rudder);
  //PAUSE();
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
  double kp = -100.0;
  double kd = -50.0;
  double ki = -50.0;
  //printf("Altitude Command = %lf Altitude = %lf Altitude Dot = %lf \n",altitude_command,altitude,altitude_dot);
  //PAUSE();
  double dup = kp*(altitude - altitude_command) + kd*(altitude_dot-0) + ki*altitude_int;  
  dup = CONSTRAIN(dup,-(OUTMAX-OUTMIN),(OUTMAX-OUTMIN));
  throttle = OUTMIN + dup;
  //throttle = OUTMAX;

  //Integral Windup
  if ((throttle > OUTMIN) && (throttle < OUTMAX)) {
    altitude_int += elapsedTime*(altitude-altitude_command);
  }
  //printf("T, ALT, ALT DOT, ALT COMM, dup = %lf %lf %lf %lf %lf \n",lastTime,altitude,altitude_dot,altitude_command,dup);  
}
