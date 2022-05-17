#ifndef HARDWARE_H
#define HARDWARE_H

//Helper Modules
#include <MATLAB/MATLAB.h>

//Datalogger class
//Need datalogger to log data to disk
//and import data files
#include <Datalogger/Datalogger.h>

//RCIO will take care of the receiver and pwm signals
#include <RCIO/RCIO.h>

//Need Sensors class to read all sensors
#include <sensors/sensors.h>

//Time for pause function
#include <Timer/timer.h>

//UART for HIL and Telemetry (Telemetry is always on so this is always include)
//Eventually I'm going to get Telemetry working in SIL and SIMONLY modes
#include <UART/uart.h>

//Need some logic here to determine any hardware in the loop
//modes
#ifdef HIL
//Running in HIL mode
#ifdef DESKTOP
//Running in desktop mode HIL so CONTROLLOOP is off
#define CONTROLLOOP 0
#endif
#ifdef RPI
//Running HIL on RPI so CONTROLLOOP is on but MAIN LOOP IS OFF
#define CONTROLLOOP 1
#endif
#else
//Hardware in the loop is off so run everything
#define CONTROLLOOP 1
#endif

///////////Inputs to Hardware Class///////////////
// 1 - Root Folder name (char*)
// 2 - Control Matrix (MATLAB)
// 3 - UART Control Matrix (MATLAB) - HIL
// 4 - UART Sense Matrix (MATLAB) - HIL

//////////Outputs to From Hardware Class//////////////
// 1 - Telemetry Matrix to Ground Station (MATLAB)
// 2 - RX Matrix to System Controller (MATLAB)
// 3 - Sense Matrix to System Controller (MATLAB)
// 4 - Configuration Matrix to System Controller (MATLAB)
// 5 - Simulation Matrix to Modeling (MATLAB)
// 6 - UART Control Matrix (MATLAB) - HIL (x2 - One to Modeling and one to System Controller)
// 7 - UART Sense Matrix (MATLAB) - HIL

class hardware {
 private:
  bool sendOK = 1;
  bool recOK = 1;
  double nextLOGtime = 0;
  double nextRCtime = 0;
  double nextTELEMtime = 0;
  MATLAB telemetry_matrix,uart_sense_matrix,uart_ctl_matrix;
  MATLAB q0123,ptp;
  Datalogger logger;
  UART ser;
  //Unfortunately telemetry values are going to be hardcoded
  //Rather than use input files you'll have to edit the code
  //in the init() and loop() functions
  int NUMTELEMETRY,NUMSENSE,NUMCTL;
  char** pwmnames;
 public:
  //RCIO Class
  RCIO rc;
  //Sensors class
  sensors sense;
  //Status
  int ok = 1;
  //Rates
  double PRINTRATE=1.0,RCRATE=1.0,LOGRATE=1.0,TELEMRATE=1.0;
  //Outputs
  MATLAB in_simulation_matrix,in_configuration_matrix;
  //Initialization routine needs the root folder name
  void init(char root_folder_name[],int NUMSIGNALS);
  //Send routine that sends model matrix to hardware to emulate sensors
  void send(double time,MATLAB model_matrix,double keyboardVars[]);
  //Main hardware loop
  void loop(double currentTime,double elapsedTime,MATLAB control_matrix);
  //HIL function
  void hil();
  //Constructor
  hardware();
};

#endif
