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

//Hardware
//Analog Signals
#include <ADC/ADC.h>
//Barometer and Thermometer
#include <BaroTemp/BaroTemp.h>
//IMU
#include <IMU/IMU.h>
//GPS
#include <GPS/GPS.h>

class sensors {
 private:
  ADC analog;
  BaroTemp atm;
  int NUMVARS;
  //Rates and baro flag
  double GPS_RATE,IMU_RATE,ANALOG_RATE;
  double nextGPStime=0,nextIMUtime=0,nextANALOGtime=0;
  int IBARO;
  //Errors
  int IERROR;
  MATLAB q0123,ptp;
  MATLAB bias_pos_matrix,bias_gyro_matrix,bias_mag_matrix,bias_angle_matrix;
  double bias_pressure_val;
  double bias_angle,bias_gyro,bias_mag,bias_pos,bias_pressure,compass=0;
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
  MATLAB sense_matrix,sense_matrix_dot;
  char** headernames,headernames_dot;
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
  //Printing routine
  void print();
  //Get number of variabls routine
  int getNumVars();
  double getTemperature();
};

#endif
