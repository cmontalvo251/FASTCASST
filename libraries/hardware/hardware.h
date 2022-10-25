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

//In order to get this to work on linux you need to run sudo apt-get install libboost-all-dev
//You need boost to run the hil functions because the serial read/write must run as fast as possible 
//to not lose data
#ifdef HIL
#include <boost/thread.hpp> 
using namespace boost;
extern boost::mutex HILmutex; //Mutex for passing data b/t HIL asynchronous threads
#endif

//Externs are to avoid multiple declaration errors during module compilation

//Asynchronous HIL thread
void hil(UART,double);
//Global vars to pass info back and forth
extern MATLAB uart_sense_matrix,uart_ctl_matrix,uart_sense_matrix_copy,uart_ctl_matrix_copy;

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
  double nextLOGtime = 0;
  double nextRCtime = 0;
  double nextTELEMtime = 0;
  double nextHILtime = 0;
  MATLAB telemetry_matrix;
  MATLAB q0123,ptp;
  Datalogger logger;
  UART serTelem,serHIL;
  //Unfortunately telemetry values are going to be hardcoded
  //Rather than use input files you'll have to edit the code
  //in the init() and loop() functions
  int NUMTELEMETRY,NUMSENSE,NUMCTL;
  char** pwmnames;
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
  #ifdef DESKTOP
  //HILRATE IS NOW HARDCODED. Note that HILRATE is just the time that the sense matrices
  //are updated. The serial hil loop runs as fast as possible to read data and not miss anything
  //Now we only need to send data to these matrices at like 10 Hz
  //So HILRATE is hardcoded for the moment
  //HILRATE is how often the uart matrices are updated
  double HILRATE=1.0; //NOTE THIS NEEDS TO BE CHANGED TO 10 Hz but for now it is set to 1 Hz
  //SERIALLOOPRATE is how fast the threaded hil loop is running 
  //For now we'll have the desktop blast at 1 Hz
  double SERIALLOOPRATE=1.0;
  #elif RPI
  double HILRATE=0.1; //Right now I have these set differently just so I can debug
  //I'm assuming in the future the HIL AND SERIALLOOP RATES will be the same but I might be wrong
  //But we'll have the RPI read at 10 Hz
  double SERIALLOOPRATE=0.1;
  #endif
  //Outputs
  MATLAB in_simulation_matrix,in_configuration_matrix;
  //Initialization routine needs the root folder name
  void init(char root_folder_name[],int NUMSIGNALS);
  //Send routine that sends model matrix to hardware to emulate sensors
  void send(double time,MATLAB model_matrix,double keyboardVars[]);
  //Main hardware loop
  void loop(double currentTime,double elapsedTime,MATLAB control_matrix);
  void hilsend(double);
  //Constructor
  hardware();
};

#endif
