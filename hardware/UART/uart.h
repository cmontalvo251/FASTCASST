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

class UART {
 private:
  float *uart_ctl_array=NULL,*uart_sense_array=NULL,*telemetry_array=NULL;
  int NUMTELEMETRY;
  Serial comms;
  int baudRate = 57600; //Hardcode. I don't think we ever need to change
 public:
  UART();
  void init(MATLAB sense_matrix,int NUMSIGNALS,int num);
  void readSense(MATLAB);
  void sendSense(MATLAB);
  void readControl(MATLAB);
  void sendControl(MATLAB);
  void sendTelemetry(MATLAB);
};




#endif
