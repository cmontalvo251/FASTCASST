//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "airplane_controller.h"

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
  throttle = rx_array[0];
  aileron = rx_array[1];
  //printf("Ail RX Ch = %lf ",aileron);
  elevator = rx_array[2];
  rudder = rx_array[3];
  autopilot = rx_array[4];
  int icontrol = 0;

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

  //Aircraft Control cases
  // 0 = fully manual
  // 1 = roll (rudder mixing) and pitch control
  // 2 = roll (rudder mixing), pitch, and velocity control
  // 3 = roll (rudder), velocity and altitude control
  // 4 = velocity, altitude and heading control
  // 5 = velocity, altitude and waypoint control
  //Initialize commands
  roll_command = -99;
  pitch_command = -99;
  velocity_command = 15; //Hardcode to 20?
  altitude_command = 50; //Hardcode to 100?
  heading_command = -99;

  double SQUAREWIDTH = 1000;
  
  WAYPOINTS_X[0] = SQUAREWIDTH;
  WAYPOINTS_Y[0] = 0;

  WAYPOINTS_X[1] = SQUAREWIDTH;
  WAYPOINTS_Y[1] = SQUAREWIDTH;

  WAYPOINTS_X[2] = 0;
  WAYPOINTS_Y[2] = SQUAREWIDTH;

  WAYPOINTS_X[3] = 0;
  WAYPOINTS_Y[3] = 0;

  //DEBUG PRINTS
  //double X = sense_matrix.get(1,1);
  //double Y = sense_matrix.get(2,1);
  //double Z = sense_matrix.get(3,1);
  //printf("XYZ = %lf %lf %lf \n",X,Y,Z);
  //double roll = sense_matrix.get(4,1);
  //double pitch = sense_matrix.get(5,1);
  //double yaw = sense_matrix.get(6,1);
  //printf("RPY = %lf %lf %lf \n",roll,pitch,yaw);
  //double u = sense_matrix.get(7,1);
  //printf("U = %lf \n",u);
  //double rollr = sense_matrix.get(10,1);
  //double pitchr = sense_matrix.get(11,1);
  //double yawr = sense_matrix.get(12,1);
  //printf("GXYZ = %lf %lf %lf \n",rollr,pitchr,yawr);

  switch (icontrol) {
    case 5:
      //velocity, altitude and waypoint control
      //printf("Waypoint Loop + ");
      WaypointLoop(sense_matrix);
    case 4:
      if (heading_command == -99) {
        heading_command = 0;
        //if (currentTime > 0) {
        //  heading_command = -90; //degrees
        //}
      }
      //velocity, altitude and heading control
      //printf("Heading + ");
      HeadingLoop(sense_matrix);
    case 3:
      //roll (rudder mixing), velocity and altitude control
      //printf("Altitude + ");
      AltitudeLoop(sense_matrix);
      //roll_command = -2;
    case 2:
      //roll (rudder mixing), pitch and velocity control
      //printf("Velocity + ");
      VelocityLoop(sense_matrix);
    case 1:
      //roll (rudder mixing) and pitch control
      //Run the Innerloop
      //with inner loop guidance 
      //printf("INNER LOOP CONTROL -- ");
      //Check for inner loop only guidance
      if (roll_command == -99) {
        roll_command = (aileron-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
      }
      if (pitch_command == -99) {
        pitch_command = -(elevator-STICK_MID)*30.0/((STICK_MAX-STICK_MIN)/2.0);
	//printf("Theta and Theta Command: %lf, %lf \n",elevator,pitch_command);
      }
      InnerLoop(sense_matrix);
    case 0:
      //printf(" Ail RX Out = %lf ",aileron);
      //printf(" Phi_c from RX Ch = %lf ",roll_command);
      //double kpa = 2.5;
      //double kda = 0.05;
      //double roll_rate = sense_matrix.get(10,1);
      //double roll = sense_matrix.get(4,1);
      //double roll_command_out = roll - (-(aileron - OUTMID) - kda*roll_rate)/kpa;
      //printf(" Phi_c from RX Out = %lf \n",roll_command_out);
      //printf("time = %lf phi,p = %lf %lf aileron = %lf \n",currentTime,roll,roll_rate,aileron);
      
      //printf("Passing signals \n");
      //Pass the receiver signals to the control_matrix and then break
      control_matrix.set(1,1,throttle);
      control_matrix.set(2,1,aileron);
      control_matrix.set(3,1,elevator);
      control_matrix.set(4,1,rudder);
      //control_matrix.disp();
      break;
  }
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
    //printf("WAY (X,Y) = (%lf,%lf) GPS (X,Y) = %lf %lf HCOMM = %lf DIST = %lf \n",WAYPOINTS_X[WAYINDEX],WAYPOINTS_Y[WAYINDEX],X,Y,heading_command,distance);
    PRINTER = 0;
  }
  PRINTER+=1;
  if (distance < 10) {
    printf("WAY (X,Y) = (%lf,%lf) GPS (X,Y) = %lf %lf HCOMM = %lf DIST = %lf \n",WAYPOINTS_X[WAYINDEX],WAYPOINTS_Y[WAYINDEX],X,Y,heading_command,distance);
    WAYINDEX += 1;
    if (WAYINDEX > NUMWAYPOINTS-1) {
      WAYINDEX = 0;
    }    
  }
}

void controller::HeadingLoop(MATLAB sense_matrix) {
  //Airplane AIRPLANE airplane
  double kp = 0.4;
  double kd = 0.25;
  double ki = 0.001;

  //Apprentice APPRENTICE apprentice
  /*double kp = 0.4;
  double kd = 0.25;
  double ki = 0.0;*/

  
  double heading = sense_matrix.get(6,1);
  double yaw_rate = sense_matrix.get(12,1);
  //I think the wrap issue is here
  //printf("HEADING COMMAND = %lf \n",heading_command);
  double dheading = -delpsi(heading*PI/180.0,heading_command*PI/180.0)*180.0/PI;
  //double dheading = -heading+heading_command;
  if (dheading > 180) {
    dheading -= 180;
    dheading *= -1;
  }
  if (dheading < -180) {
    dheading += 180;
    dheading *= -1;
  }
  roll_command = kp*dheading + kd*(0-yaw_rate) + ki*headingint;
  //Integrate but prevent integral windup
  if ((roll_command < 45) && (roll_command > -45)) {
    headingint += elapsedTime*dheading;
  }
  roll_command = CONSTRAIN(roll_command,-45,45);
  //printf("T, HEADING = %lf %lf \n",lastTime,heading,roll_command);
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
  pitch_command = kp*(altitude_command - altitude) + kd*(0-altitude_dot);
  pitch_command = CONSTRAIN(pitch_command,-45,45);
  //printf("T, ALT, ALT DOT = %lf %lf %lf \n",lastTime,altitude,altitude_dot);  
}

void controller::VelocityLoop(MATLAB sense_matrix) {
  //printf("------\n");
  //sense_matrix.disp();
  double u = sense_matrix.get(7,1);
  double velocityerror = velocity_command - u;
  //printf("Sense U = %lf \n",u);
  //printf("Velocity Error = %lf \n",velocityerror);
  double kp = 120.0; //120
  double ki = 8.0; //8.0
  throttle = OUTMIN + kp*velocityerror + ki*velocityint;
  throttle = CONSTRAIN(throttle,OUTMIN,OUTMAX);
  //Integrate but prevent integral windup
  if ((throttle > OUTMIN) && (throttle < OUTMAX)) {
    velocityint += elapsedTime*velocityerror;
  }
}

void controller::InnerLoop(MATLAB sense_matrix) {
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
    double kpe = 25.0;
    double kde = 1.0;
    double kr = 1.0;
    
    //Apprentice APPRENTICE apprentice
    /*double kpa = 5.0;
    double kda = 0.1;
    double kpe = 10.0;
    double kd = 0.5;
    double kr = 0.1;*/

    aileron = kpa*(roll-roll_command) + kda*(roll_rate);
    elevator = kpe*(pitch-pitch_command) + kde*(pitch_rate);

    //Rudder signal will be proportional to aileron
    rudder = kr*aileron;

    //CONSTRAIN
    rudder = CONSTRAIN(rudder,-500,500) + OUTMID;
    aileron = -CONSTRAIN(aileron,-500,500) + OUTMID;
    elevator = CONSTRAIN(elevator,-500,500) + OUTMID;
}
