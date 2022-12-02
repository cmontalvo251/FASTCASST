#ifndef UART_H
#define UART_H


/////////////INPUTS TO UART CLASS///////////
// 1 - sense_matrix (MATLAB) - For telemetry and HIL
// 2 - ctl_matrix (MATLAB) - For telemetry and HIL
// 3 - uart_ctl_array (double*) - For HIL
// 4 - uart_sense_array (double*) - For HIL
////////////////////////////////////////////

/////////////OUTPUTS FROM UART CLASS////////
// 1 - telemetry_array (double*) - For telemetry
// 2 - uart_ctl_array (double*) - For HIL
// 3 - uart_sense_array (double*) - For HIL
//////////////////////////////////////////////

//Helper Modules
#include <MATLAB/MATLAB.h>

//Serial Module
#include "Serial.h"

//Timer for Pause
#include <Timer/timer.h>

class UART {
 private:
  float *uart_ctl_array=NULL,*uart_sense_array=NULL,*uart_telemetry_array=NULL;
  int NUMTELEMETRY,NUMCTL,NUMSENSE;
  Serial comms;
  Serial hilcomms;
  //int baudRate = 57600; //Hardcode. I don't think we ever need to change
  int baudRate = 115200; //Dr. Russ told me to change the baudrate to 115200
 public:
  UART();
  void TelemInit(int);
  void HILInit(int,int);
  void readSense(MATLAB);
  void sendSense(MATLAB);
  int readControl(MATLAB);
  void sendControl(MATLAB);
  void sendTelemetry(MATLAB,int);
};

#endif
