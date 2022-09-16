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
}

void controller::print() {
  for (int i = 1;i<=NUMSIGNALS;i++) {
    printf("%d ",int(control_matrix.get(i,1)));
  }
}

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
  
  double motor = STICK_MID; //motor
  double servo = STICK_MID; //steering

  //First extract the relavent commands from the receiver.
  throttle = rx_array[0];
  aileron = rx_array[1];
  autopilot = rx_array[4];
  int icontrol = 0;

  //Check for user controlled
  if (CONTROLLER_FLAG == -1) {
    if (autopilot > STICK_MID) {
      //printf("AUTO !!!! ");
      icontrol = 1;
    } else {
      icontrol = 0;
    }
  } else {
    icontrol = CONTROLLER_FLAG;
  }

  //Car Control cases
  // 0 = fully manual
  // 1 = Throttle manual, heading control
  // 2 = Throttle manual, waypoint control
  // 3 = Speed and waypoint control
  //Initialize commands
  velocity_command = -99;
  heading_command = -99;
  
  WAYPOINTS_X[0] = 500;
  WAYPOINTS_Y[0] = 0;

  WAYPOINTS_X[1] = 500;
  WAYPOINTS_Y[1] = 500;

  WAYPOINTS_X[2] = 0;
  WAYPOINTS_Y[2] = 500;

  WAYPOINTS_X[3] = 0;
  WAYPOINTS_Y[3] = 0;

  switch (icontrol) {
    case 3:
      //waypoint control
      //printf("WAYPOINT \n");
      WaypointLoop(sense_matrix);
    case 2:
      if (heading_command == -99) {
        heading_command = 45;
      }
      //roll (rudder mixing), velocity and altitude control
      //printf("Altitude + ");
      HeadingLoop(sense_matrix);
    case 1:
      //velocity
      VelocityLoop(sense_matrix);
    case 0:
      //If the velocity controller is off we send 75% speed to the motor
      if (velocity_command == -99) {
        throttle = STICK_MID + 0.75*(STICK_MAX-STICK_MID); 
      }
      //printf("Passing signals \n");
      //Pass the receiver signals to the control_matrix and then break
      motor = throttle;
      servo = aileron;
      //control_matrix.disp();
      break;
  }

  //Saturation
  if (motor < STICK_MIN) {
    motor = STICK_MIN;
  }
  if (motor > STICK_MAX) {
    motor = STICK_MAX;
  }
  if (servo < STICK_MIN) {
    servo = STICK_MIN;
  }
  if (servo > STICK_MAX) {
    servo = STICK_MAX;
  }
  
  //Set motor commands to the ctlcomms values
  control_matrix.set(1,1,motor);
  control_matrix.set(2,1,servo);
  //control_matrix.disp();
}

void controller::WaypointLoop(MATLAB sense_matrix) {
  double X = sense_matrix.get(1,1);
  double Y = sense_matrix.get(2,1);
  double DY = WAYPOINTS_Y[WAYINDEX]-Y;
  double DX = WAYPOINTS_X[WAYINDEX]-X;
  heading_command = atan2(DY,DX)*180.0/PI;
  double distance = sqrt(DY*DY + DX*DX);
  //PAUSE();
  if (PRINTER == 4*100000) {
    printf("WAY (X,Y) = (%lf,%lf) GPS (X,Y) = %lf %lf HCOMM = %lf DIST = %lf \n",WAYPOINTS_X[WAYINDEX],WAYPOINTS_Y[WAYINDEX],X,Y,heading_command,distance);
    PRINTER = 0;
  }
  PRINTER+=1;
  if (distance < 150) {
    printf("WAY (X,Y) = (%lf,%lf) GPS (X,Y) = %lf %lf HCOMM = %lf DIST = %lf \n",WAYPOINTS_X[WAYINDEX],WAYPOINTS_Y[WAYINDEX],X,Y,heading_command,distance);
    WAYINDEX += 1;
    if (WAYINDEX > NUMWAYPOINTS-1) {
      WAYINDEX = 0;
    }    
  }
}

void controller::HeadingLoop(MATLAB sense_matrix) {
  double kp = 2.5;
  double heading = sense_matrix.get(6,1);
  //I think the wrap issue is here
  double dheading = -delpsi(heading*PI/180.0,heading_command*PI/180.0)*180.0/PI;
  if (dheading > 180) {
    dheading -= 180;
    dheading *= -1;
  }
  if (dheading < -180) {
    dheading += 180;
    dheading *= -1;
  }
  double daileron = kp*dheading;
  double dpwm = STICK_MAX - STICK_MID;
  daileron = CONSTRAIN(daileron,-dpwm,dpwm);
  aileron = STICK_MID + daileron;
  //printf("T, HEADING = %lf %lf \n",lastTime,heading,aileron);
  //PAUSE();
}

void controller::VelocityLoop(MATLAB sense_matrix) {
  //printf("------\n");
  //sense_matrix.disp();
  double u = sense_matrix.get(7,1);
  velocity_command = 5;
  double velocityerror = velocity_command - u;
  //printf("Sense U = %lf \n",u);
  //printf("Velocity Error = %lf \n",velocityerror);
  double kp = 60.0;
  double ki = 20.0;
  throttle = OUTMID + kp*velocityerror + ki*velocityint;
  throttle = CONSTRAIN(throttle,OUTMIN,OUTMAX);
  //Integrate but prevent integral windup
  if ((throttle > OUTMIN) && (throttle < OUTMAX)) {
    velocityint += elapsedTime*velocityerror;
  }
  //printf("Throttle = %lf \n",throttle);
}

