#ifndef COMMS_H
#define COMMS_H

/////////////INPUTS TO comms CLASS///////////
// 1 - sense_matrix (MATLAB) - For telemetry and HIL
// 2 - ctl_matrix (MATLAB) - For telemetry and HIL
// 3 - comms_ctl_array (double*) - For HIL
// 4 - comms_sense_array (double*) - For HIL
////////////////////////////////////////////

/////////////OUTPUTS FROM comms CLASS////////
// 1 - telemetry_array (double*) - For telemetry
// 2 - comms_ctl_array (double*) - For HIL
// 3 - comms_sense_array (double*) - For HIL
//////////////////////////////////////////////

#ifdef ARDUINO
#include "MATLAB.h"
#include "SerialComms.h"
#else
//Helper Modules
#include <MATLAB/MATLAB.h>
//Serial Module
#include <SerialComms/SerialComms.h>
//Timer for Pause
#include <Timer/timer.h>
#endif

class Comms {
 private:
  float *comms_ctl_array=NULL,*comms_sense_array=NULL,*comms_telemetry_array=NULL;
  int NUMTELEMETRY,NUMCTL,NUMSENSE;
  SerialComms uart;
  SerialComms hiluart;
  int baudRateWireless; //Hardcode. I don't think we ever need to change
  int baudRate; //Dr. Russ told me to change the baudrate to 115200
 public:
  Comms();
  void TelemInit(int);
  void HILInit(int,int);
  void readSense(MATLAB);
  void sendSense(MATLAB);
  int readControl(MATLAB);
  void sendControl(MATLAB);
  void sendTelemetry(MATLAB,int);
};

#endif
