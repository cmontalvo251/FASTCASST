#ifndef HARDWARE_H
#define HARDWARE_H

#ifdef ARDUINO
#include "MATLAB.h"
#include "Datalogger.h"
#include "RCIO.h"
#include "timer.h"
#else
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
//Comms class for communication via HIL and Telemetry (Telemetry is always on so this is always include)
//Eventually I'm going to get Telemetry working in SIL and SIMONLY modes
#include <Comms/Comms.h>
#endif

//In order to get this to work on linux you need to run sudo apt-get install libboost-all-dev
//You need boost to run the hil functions because the serial read/write must run as fast as possible 
//to not lose data
#ifdef HIL
#include <boost/thread.hpp> 
using namespace boost;
extern boost::mutex HILmutex; //Mutex for passing data b/t HIL asynchronous threads
#endif

//Externs are to avoid multiple declaration errors during module compilation

//Asynchronous HIL thread -- Moved to class and synchronous function call
//void hil(comms,double);
//Global vars to pass info back and forth
extern MATLAB comms_sense_matrix,comms_ctl_matrix,comms_sense_matrix_copy,comms_ctl_matrix_copy;

///////////Inputs to Hardware Class///////////////
// 1 - Root Folder name (char*)
// 2 - Control Matrix (MATLAB)
// 3 - comms Control Matrix (MATLAB) - HIL
// 4 - comms Sense Matrix (MATLAB) - HIL

//////////Outputs to From Hardware Class//////////////
// 1 - Telemetry Matrix to Ground Station (MATLAB)
// 2 - RX Matrix to System Controller (MATLAB)
// 3 - Sense Matrix to System Controller (MATLAB)
// 4 - Configuration Matrix to System Controller (MATLAB)
// 5 - Simulation Matrix to Modeling (MATLAB)
// 6 - comms Control Matrix (MATLAB) - HIL (x2 - One to Modeling and one to System Controller)
// 7 - comms Sense Matrix (MATLAB) - HIL

class hardware {
 private:
  bool sendOK = 1; //Set to 1 and DESKTOP will send first
  bool recOK = 1; //Set to 1 and RPI will receive first
  double nextLOGtime = 0;
  double nextRCtime = 0;
  double nextTELEMtime = 0;
  double nextHILtime = 0;
  MATLAB telemetry_matrix;
  MATLAB q0123,ptp;
  Datalogger logger;
  Comms serTelem,serHIL;
  //Unfortunately telemetry values are going to be hardcoded
  //Rather than use input files you'll have to edit the code
  //in the init() and loop() functions
  int NUMTELEMETRY,NUMSENSE,NUMCTL;
  char** pwmnames;
  char** rcnames;
  //double HILtime;
 public:
  //RCIO Class
  RCIO rc;
  //Sensors class
  sensors sense;
  //Status
  int ok = 1;
  //Rates
  double PRINTRATE=1.0,RCRATE=1.0,LOGRATE=1.0,TELEMRATE=1.0;
  //Set these values in hardware init so that make recompiles
  double HILRATE=0.0;  
  double SERIALLOOPRATE=1.0;
  //Outputs
  MATLAB in_simulation_matrix,in_configuration_matrix;
  //Initialization routine needs the root folder name
  void init(char root_folder_name[],int NUMSIGNALS);
  //Send routine that sends model matrix to hardware to emulate sensors
  void send(double time,MATLAB model_matrix,double keyboardVars[]);
  //Main hardware loop
  void loop(double currentTime,double elapsedTime,MATLAB control_matrix);
  //void hilsend(double);
  void hil(double,int);
  //Constructor
  hardware();
};

#endif
