#include "sensors.h"

//Constructor
sensors::sensors() {
  sense_matrix.zeros(15,1,"Full State From Sensors");
  sense_matrix_dot.zeros(15,1,"Full Statedot From Sensors");
  NUMVARS = 30; //15 + 15
  //Set names of headers
  headernames = (char**)malloc(NUMVARS*sizeof(char*));
  headernames[0] = "Sense X(m)";
  headernames[1] = "Sense Y(m)";
  headernames[2] = "Sense Z(m)";
  headernames[3] = "Sense Roll(deg)";
  headernames[4] = "Sense Pitch(deg)";
  headernames[5] = "Sense Yaw(deg)";
  headernames[6] = "Sense U(m/s)";
  headernames[7] = "Sense V(m/s)";
  headernames[8] = "Sense W(m/s)";
  headernames[9] = "Sense P(deg/s)";
  headernames[10] = "Sense Q(deg/s)";
  headernames[11] = "Sense R(deg/s)";
  headernames[12] = "Sense Mx(Gauss)";
  headernames[13] = "Sense My(Gauss)";
  headernames[14] = "Sense Mz(Gauss)";
  headernames[15] = "Sense X dot (m)";
  headernames[16] = "Sense Y dot (m)";
  headernames[17] = "Sense Z dot (m)";
  headernames[18] = "Sense Roll dot (deg)";
  headernames[19] = "Sense Pitch dot (deg)";
  headernames[20] = "Sense Yaw dot (deg)";
  headernames[21] = "Sense U dot (m/s)";
  headernames[22] = "Sense V dot (m/s)";
  headernames[23] = "Sense W dot (m/s)";
  headernames[24] = "Sense P dot (deg/s)";
  headernames[25] = "Sense Q dot (deg/s)";
  headernames[26] = "Sense R dot (deg/s)";
  headernames[27] = "Sense Mx dot (Gauss)";
  headernames[28] = "Sense My dot (Gauss)";
  headernames[29] = "Sense Mz dot (Gauss)";
}

//Get numvars
int sensors::getNumVars() {
  return NUMVARS;
}

//Polling routine to read all sensors
void sensors::poll(double currentTime,double elapsedTime) {
  //First Analog to Digital Converter
  analog.get_results();
  
  //Then we poll the barometer and temperature sensor (needs current time)
  atm.poll(currentTime);

  ///Read the IMU (ptp,pqr)
  orientation.FilterConstant = 0.0; //0 for no filtering and 1.0 for overfiltering
  orientation.loop(elapsedTime); 

  //Read the GPS
  satellites.poll(currentTime,1); //This will compute XYZ as well. For now we are using 
  //hardcoded GPS coordinates

  ///////////////////??THEN POPULATE STATE VECTOR////////////////////

  //Initialize everything to -99
  sense_matrix.mult_eq(0);
  sense_matrix.plus_eq(-99);
  sense_matrix_dot.mult_eq(0);
  sense_matrix_dot.plus_eq(-99);

  ///XYZ
  sense_matrix_dot.set(1,1,satellites.X);
  sense_matrix_dot.set(2,1,satellites.Y);
  sense_matrix_dot.set(3,1,satellites.Z);

  //PTP
  sense_matrix.set(4,1,orientation.roll);
  sense_matrix.set(5,1,orientation.pitch);
  sense_matrix.set(6,1,orientation.yaw);

  //UWV
  //Assume that the vehicle is traveling straight so V and W are zero
  sense_matrix.set(7,1,satellites.speed);
  sense_matrix.set(8,1,0);
  sense_matrix.set(9,1,0);

  //PQR
  sense_matrix.set(10,1,orientation.roll_rate);
  sense_matrix.set(11,1,orientation.pitch_rate);
  sense_matrix.set(12,1,orientation.yaw_rate);

  //MXYZ
  sense_matrix.set(13,1,orientation.mx);
  sense_matrix.set(14,1,orientation.my);
  sense_matrix.set(15,1,orientation.mz);

  //XYZDOT - Probably would need to run the GPS coordinates through a derivative
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
  //????

}

//Printing Routine
void sensors::print() {
  //Analog signals
  analog.print_results();
  //Barometer and Temperature
  printf("%lf %lf %lf ",atm.pressure,atm.altitude,atm.temperature);
  //Roll Pitch Yaw
  printf("%lf %lf %lf ",orientation.roll,orientation.pitch,orientation.yaw);
  //PQR
  printf("%lf %lf %lf ",orientation.roll_rate,orientation.pitch_rate,orientation.yaw_rate);
  //GPS
  printf("%lf %lf %lf ",satellites.latitude,satellites.longitude,satellites.altitude);
}
