#ifndef SENSORS_H
#define SENSORS_H

///////////INPUTS TO SENSORS CLASS///////////
// 1 - in_error_matrix (MATLAB)
// 2 - model_matrix (MATLAB) - SIL
// 3 - UART_sense_matrix (MATLB) - HIL

//////////OUTPUTS FROM SENSORS CLASS////////////
// 1 - sense_matrix and semse_matrix_dot x3 (one to datalogger, one to uart, one to controller)

//Helper Modules
#include <MATLAB/MATLAB.h>
#include <Mathp/mathp.h>

//Hardware
//Analog Signals
#include <ADC/ADC.h>
//PTH Sensor
#include <PTH/PTH.h>
//IMU
#include <IMU/IMU.h>
//GPS
#include <GPS/GPS.h>

class sensors {
 private:
  ADC analog;
  int NUMVARS;
  //Rates and baro flag
  double GPS_RATE,IMU_RATE,ANALOG_RATE,heading_offset_new=0;
  double nextGPStime=0,nextIMUtime=0,nextANALOGtime=0;
  double nextGPSOffset=0;
  int IBARO;
  MATLAB q0123,ptp;
  //Errors
  int IERROR;
  MATLAB bias_pos_matrix,bias_gyro_matrix,bias_mag_matrix,bias_angle_matrix;
  double bias_pressure_val;
  double bias_angle,bias_gyro,bias_mag,bias_pos,bias_pressure;
  double std_angle,std_gyro,std_mag,std_pos,std_pressure;
  double noise_angle,noise_gyro,noise_mag,noise_pos,noise_pressure;
  void setBias(MATLAB,double,double);
  double pollute(double,double);
  void sendXYZ2GPS(double t,double x,double y,double z);
  void getCompassHeading();
 public:
  //Public variables and classes
  IMU orientation;
  GPS satellites;
  PTH atm;
  MATLAB sense_matrix,sense_matrix_dot;
  char** headernames,headernames_dot;
  double Heading_Mag,x_tilt,y_tilt,Hm,compass=0.0;
  double heading_offset=0;
  //constructor
  sensors();
  //Send model matrix
  void send(double,MATLAB);
  //Initialize
  void init(MATLAB,MATLAB);
  //Initialize the IMU
  void initIMU(int);
  //Polling routine
  void poll(double currentTime,double elapsedTime);
  //Populate Matrices Routine
  void populate(double currentTime,double elapsedTime);
  //Magnetometer Heading
  double mag_heading(double,double,double,double,double);
  //Printing routine
  void print();
  //Get number of variabls routine
  int getNumVars();
  double getTemperature();
};

#endif
