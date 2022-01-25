#include "sensors.h"

//Constructor
sensors::sensors() {
  //Make sure this is the same as the modeling matrix-1
  NUMVARS = 30;
  sense_matrix.zeros(30,1,"Full State From Sensors");
  //sense_matrix_dot.zeros(15,1,"Full Statedot From Sensors");
  //Set names of headers
  headernames = (char**)malloc(NUMVARS*sizeof(char*));
  headernames[0] = "Sense X(m)";
  headernames[1] = "Sense Y(m)";
  headernames[2] = "Sense Z(m)";
  headernames[3] = "Q0";
  headernames[4] = "Q1";
  headernames[5] = "Q2";
  headernames[6] = "Q3";
  headernames[7] = "Sense U(m/s)";
  headernames[8] = "Sense V(m/s)";
  headernames[9] = "Sense W(m/s)";
  headernames[10] = "Sense P(deg/s)";
  headernames[11] = "Sense Q(deg/s)";
  headernames[12] = "Sense R(deg/s)";
  headernames[13] = "Sense Mx(Gauss)";
  headernames[14] = "Sense My(Gauss)";
  headernames[15] = "Sense Mz(Gauss)";
  headernames[16] = "Sense GPS Latitude (deg)";
  headernames[17] = "Sense GPS Longitude (deg)";
  headernames[18] = "Sense GPS Altitude (m)";
  headernames[19] = "Sense GPS Heading (deg)";
  headernames[20] = "IMU Heading (deg)";
  headernames[21] = "Sense Analog 1 (V)";
  headernames[22] = "Sense Analog 2 (V)";
  headernames[23] = "Sense Analog 3 (V)";
  headernames[24] = "Sense Analog 4 (V)";
  headernames[25] = "Sense Analog 5 (V)";
  headernames[26] = "Sense Analog 6 (V)";
  headernames[27] = "Sense Pressure (Pa)";
  headernames[28] = "Sense Pressure Altitude (m)";
  headernames[29] = "Sense Temperature (C)";
}

//Get numvars
int sensors::getNumVars() {
  return NUMVARS;
}

///Initialize the IMU
void sensors::initIMU(int sensor_type) {
  //sensor_type == 0 -> MPU9250
  //sensor_type == 1 -> LSM9DS1
  orientation.init(sensor_type);
}

//Polling routine to read all sensors
void sensors::poll(double currentTime,double elapsedTime) {
  //First Analog to Digital Converter
  analog.get_results();
  
  //Then we poll the barometer and temperature sensor (needs current time)
  atm.poll(currentTime);

  ///Read the IMU (ptp,pqr)
  orientation.loop(elapsedTime); 

  //Read the GPS
  satellites.poll(currentTime,1); //This will compute XYZ as well. For now we are using 
  //hardcoded GPS coordinates

  ///////////////////??THEN POPULATE STATE VECTOR////////////////////

  //Initialize everything to -99
  sense_matrix.mult_eq(0);
  sense_matrix.plus_eq(-99);
  //sense_matrix_dot.mult_eq(0);
  //sense_matrix_dot.plus_eq(-99);

  ///XYZ
  sense_matrix.set(1,1,satellites.X);
  sense_matrix.set(2,1,satellites.Y);
  sense_matrix.set(3,1,satellites.Z);

  //Quaternions
  sense_matrix.set(4,1,orientation.ahrs.q0);
  sense_matrix.set(5,1,orientation.ahrs.q1);
  sense_matrix.set(6,1,orientation.ahrs.q2);
  sense_matrix.set(7,1,orientation.ahrs.q3);

  //UWV
  //Assume that the vehicle is traveling straight so V and W are zero
  sense_matrix.set(8,1,satellites.speed);
  sense_matrix.set(9,1,0);
  sense_matrix.set(10,1,0);

  //PQR
  sense_matrix.set(11,1,orientation.roll_rate);
  sense_matrix.set(12,1,orientation.pitch_rate);
  sense_matrix.set(13,1,orientation.yaw_rate);

  //MXYZ
  sense_matrix.set(14,1,orientation.mx);
  sense_matrix.set(15,1,orientation.my);
  sense_matrix.set(16,1,orientation.mz);

  //GPS
  sense_matrix.set(17,1,satellites.latitude);
  sense_matrix.set(18,1,satellites.longitude);
  sense_matrix.set(19,1,satellites.altitude);
  sense_matrix.set(20,1,satellites.heading);

  //IMU
  sense_matrix.set(21,1,orientation.yaw);

  //Analog
  sense_matrix.vecset(22,27,analog.results,1);

  //Barometer
  sense_matrix.set(28,1,atm.pressure);
  sense_matrix.set(29,1,atm.altitude);
  sense_matrix.set(30,1,atm.temperature);

  /* //XYZDOT - Probably would need to run the GPS coordinates through a derivative
  //filter. If we really need these I can get them. 
  //???

  //PTPDOT
  sense_matrix_dot.set(4,1,orientation.roll_rate);
  sense_matrix_dot.set(5,1,orientation.pitch_rate);
  sense_matrix_dot.set(6,1,orientation.yaw_rate);

  //UVWDOT
  //???

  //PQRDOT
  //????

  //MXYZDOT
  //???? */

}

//Printing Routine
void sensors::print() {
  //XYZ
  //Q0123
  printf("Q0123 = %lf %lf %lf %lf ",orientation.ahrs.q0,orientation.ahrs.q1,orientation.ahrs.q2,orientation.ahrs.q3);
  //Euler
  printf("PTP = %lf %lf %lf ",orientation.roll,orientation.pitch,orientation.yaw);  
  //UVW
  //PQR
  printf("PQR = %lf %lf %lf ",orientation.roll_rate,orientation.pitch_rate,orientation.yaw_rate);
  //MXYZ
  //GPS
  //printf("%lf %lf %lf ",satellites.latitude,satellites.longitude,satellites.altitude);
  //Analog signals
  //analog.print_results();
  //Barometer and Temperature
  //printf("%lf %lf %lf ",atm.pressure,atm.altitude,atm.temperature);
}
