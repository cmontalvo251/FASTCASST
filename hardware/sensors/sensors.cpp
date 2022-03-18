#include "sensors.h"

//Constructor
sensors::sensors() {
  //Make sure this is the same as the modeling matrix-1
  NUMVARS = 29;
  sense_matrix.zeros(NUMVARS,1,"Full State From Sensors");
  //sense_matrix_dot.zeros(15,1,"Full Statedot From Sensors");
  //Set names of headers
  headernames = (char**)malloc(NUMVARS*sizeof(char*));
  headernames[0] = "Sense X(m)";
  headernames[1] = "Sense Y(m)";
  headernames[2] = "Sense Z(m)";
  headernames[3] = "Sense Roll (deg)";
  headernames[4] = "Sense Pitch (deg)";
  headernames[5] = "Sense Yaw (deg)";
  headernames[6] = "Sense U(m/s)";
  headernames[7] = "Sense V(m/s)";
  headernames[8] = "Sense W(m/s)";
  headernames[9] = "Sense P(rad/s)";
  headernames[10] = "Sense Q(rad/s)";
  headernames[11] = "Sense R(rad/s)";
  headernames[12] = "Sense Mx(Gauss)";
  headernames[13] = "Sense My(Gauss)";
  headernames[14] = "Sense Mz(Gauss)";
  headernames[15] = "Sense GPS Latitude (deg)";
  headernames[16] = "Sense GPS Longitude (deg)";
  headernames[17] = "Sense GPS Altitude (m)";
  headernames[18] = "Sense GPS Heading (deg)";
  headernames[19] = "IMU Heading (deg)";
  headernames[20] = "Sense Analog 1 (V)";
  headernames[21] = "Sense Analog 2 (V)";
  headernames[22] = "Sense Analog 3 (V)";
  headernames[23] = "Sense Analog 4 (V)";
  headernames[24] = "Sense Analog 5 (V)";
  headernames[25] = "Sense Analog 6 (V)";
  headernames[26] = "Sense Pressure (Pa)";
  headernames[27] = "Sense Pressure Altitude (m)";
  headernames[28] = "Sense Temperature (C)";
}

//Get numvars
int sensors::getNumVars() {
  return NUMVARS;
}

void sensors::setBias(MATLAB bias_matrix,double bias,double std) {
  for (int i = 1;i<4;i++) {
    double val = bias + randnum(-std,std);
    bias_matrix.set(i,1,val);
  }
}

double sensors::pollute(double bias,double noise) {
  return bias + randnum(-noise,noise);
}

void sensors::init(MATLAB in_configuration_matrix,MATLAB in_simulation_matrix) {
  //Get Rates of All Sensors
  GPS_RATE = in_configuration_matrix.get(5,1);
  IMU_RATE = in_configuration_matrix.get(6,1);
  ANALOG_RATE = in_configuration_matrix.get(7,1);
  IBARO = in_configuration_matrix.get(8,1);

  //Pick the IMU you want to use
  //0 = MPU9250
  //1 = LSM9DS1
  int IMUTYPE = in_configuration_matrix.get(9,1);
  initIMU(IMUTYPE);

  //Set the Filter Constant
  orientation.FilterConstant = in_configuration_matrix.get(10,1);

  //Initialize q0123 and ptp
  q0123.zeros(4,1,"Sense Quaternions");
  ptp.zeros(3,1,"Sense Roll Pitch Yaw");

  //Sensor Errors
  IERROR = in_simulation_matrix.get(21,1);
  //POSITION
  bias_pos = in_simulation_matrix.get(22,1);
  std_pos = in_simulation_matrix.get(23,1);
  noise_pos = in_simulation_matrix.get(24,1);
  bias_pos_matrix.zeros(3,1,"Bias Position");
  setBias(bias_pos_matrix,bias_pos,std_pos);
  //ANGLE
  bias_angle = in_simulation_matrix.get(25,1);
  std_angle = in_simulation_matrix.get(26,1);
  noise_angle = in_simulation_matrix.get(27,1);
  bias_angle_matrix.zeros(3,1,"Bias Angle");
  setBias(bias_angle_matrix,bias_angle,std_angle);
  //VELOCITY
  bias_velocity = in_simulation_matrix.get(28,1);
  std_velocity = in_simulation_matrix.get(29,1);
  noise_velocity = in_simulation_matrix.get(30,1);
  bias_velocity_matrix.zeros(3,1,"Bias Velocity");
  setBias(bias_velocity_matrix,bias_velocity,std_velocity);
  //GYRO
  bias_gyro = in_simulation_matrix.get(31,1);
  std_gyro = in_simulation_matrix.get(32,1);
  noise_gyro = in_simulation_matrix.get(33,1);
  bias_gyro_matrix.zeros(3,1,"Bias Gyro");
  setBias(bias_gyro_matrix,bias_gyro,std_gyro);  
  //MAG
  bias_mag = in_simulation_matrix.get(34,1);
  std_mag = in_simulation_matrix.get(35,1);
  noise_mag = in_simulation_matrix.get(36,1);
  bias_mag_matrix.zeros(3,1,"Bias Mag");
  setBias(bias_mag_matrix,bias_mag,std_mag);
}

///Initialize the IMU
void sensors::initIMU(int sensor_type) {
  //sensor_type == 0 -> MPU9250
  //sensor_type == 1 -> LSM9DS1
  orientation.init(sensor_type);
}

void sensors::send(MATLAB model_matrix) {
  //X,Y,Z
  double X = model_matrix.get(1,1);
  double Y = model_matrix.get(2,1);
  double Z = model_matrix.get(3,1);
  //Add Errors if present
  if (IERROR) {
    X += pollute(bias_pos_matrix.get(1,1),noise_pos);
    Y += pollute(bias_pos_matrix.get(2,1),noise_pos);
    Z += pollute(bias_pos_matrix.get(3,1),noise_pos);
  }
  satellites.X = X;
  satellites.Y = Y;
  satellites.Z = Z;
  //We need to reset the GPS values if we haven't gotten a valid coorinate yet
  satellites.reset();
  //printf("%lf %lf %lf \n",sense.satellites.X,sense.satellites.Y,sense.satellites.Z);
  //PAUSE();

  //Quaternions
  //Convert the quaternions to Euler Angles if ERRORS present
  double q0,q1,q2,q3;
  if (IERROR) {
    q0123.vecset(1,4,model_matrix,4);
    //q0123.disp();
    ptp.quat2euler(q0123);
    //ptp.disp();
    double roll = ptp.get(1,1)*180.0/PI;
    double pitch = ptp.get(2,1)*180.0/PI;
    double yaw = ptp.get(3,1)*180.0/PI;
    roll += pollute(bias_angle_matrix.get(1,1),noise_angle);
    pitch += pollute(bias_angle_matrix.get(2,1),noise_angle);
    yaw += pollute(bias_angle_matrix.get(3,1),noise_angle);
    ptp.set(1,1,roll*PI/180.0);
    ptp.set(2,1,pitch*PI/180.0);
    ptp.set(3,1,yaw*PI/180.0);
    q0123.euler2quat(ptp);
    q0 = q0123.get(1,1);
    q1 = q0123.get(2,1);
    q2 = q0123.get(3,1);
    q3 = q0123.get(4,1);
  } else {
      //model_matrix.disp();
    q0 = model_matrix.get(4,1);
    q1 = model_matrix.get(5,1);
    q2 = model_matrix.get(6,1);
    q3 = model_matrix.get(7,1);
  }
  orientation.ahrs.q0 = q0;
  orientation.ahrs.q1 = q1;
  orientation.ahrs.q2 = q2;
  orientation.ahrs.q3 = q3;
  //printf("send() %lf %lf %lf \n",sense.orientation.roll,sense.orientation.pitch,sense.orientation.yaw);

  //Set gx,gy,gz from sense_matrix for filtering 
  double gy = model_matrix.get(11,1);
  double gx = model_matrix.get(12,1);
  double gz = -model_matrix.get(13,1);
  if (IERROR) {
    printf("GXYZ Before = %lf %lf %lf \n",gx,gy,gz);
    gx += pollute(bias_gyro_matrix.get(1,1),noise_gyro);
    gy += pollute(bias_gyro_matrix.get(2,1),noise_gyro);
    gz += pollute(bias_gyro_matrix.get(3,1),noise_gyro);
    printf("GXYZ After = %lf %lf %lf \n",gx,gy,gz);
  }
  orientation.gx = gx;
  orientation.gy = gy;
  orientation.gz = gz;

  //Still need all the other sensor states but not right now
}

//Polling routine to read all sensors
void sensors::poll(double currentTime,double elapsedTime) {
  //First Analog to Digital Converter
  if (currentTime >= nextANALOGtime) {
    //printf("Reading Analog %lf \n",currentTime);
    analog.get_results();
    nextANALOGtime=currentTime+ANALOG_RATE;
  }
  
  //Then we poll the barometer and temperature sensor (needs current time)
  if (IBARO) {
    //printf("Polling Barometer %lf \n",currentTime);
    atm.poll(currentTime);
  }

  ///Read the IMU (ptp,pqr)
  //IMU must be read as fast as possible due to the elapsedTime
  //and the integrator on board
  orientation.loop(elapsedTime); 

  //Read the GPS
  if (currentTime >= nextGPStime) {
    //printf("Polling GPS %lf \n",currentTime);
    satellites.poll(currentTime); //This will compute XYZ as well. For now we are using 
    //hardcoded GPS coordinates
    nextGPStime = currentTime + GPS_RATE;
  }

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

  //GPS
  sense_matrix.set(16,1,satellites.latitude);
  sense_matrix.set(17,1,satellites.longitude);
  sense_matrix.set(18,1,satellites.altitude);
  sense_matrix.set(19,1,satellites.heading);

  //IMU
  sense_matrix.set(20,1,orientation.yaw);

  //Analog
  sense_matrix.vecset(21,26,analog.results,1);

  //Barometer
  sense_matrix.set(27,1,atm.pressure);
  sense_matrix.set(28,1,atm.altitude);
  sense_matrix.set(29,1,atm.temperature);

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
  //printf("Q0123 = %lf %lf %lf %lf ",orientation.ahrs.q0,orientation.ahrs.q1,orientation.ahrs.q2,orientation.ahrs.q3);
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
