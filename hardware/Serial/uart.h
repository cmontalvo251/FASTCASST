#ifdef UART_H
#define UART_H


/////////////INPUTS TO UART CLASS///////////
// 1 - sense_matrix (MATLAB) - For telemetry and HIL
// 2 - ctl_matrix (MATLAB) - For telemetry and HIL
// 3 - uart_ctl_matrix (MATLAB) - For HIL
// 4 - uart_sense_matrix (MATLAB) - For HIL
////////////////////////////////////////////

/////////////OUTPUTS FROM UART CLASS////////
// 1 - telemetry_matrix (MATLAB) - For telemetry
// 2 - uart_ctl_matrix (MATLAB) - For HIL
// 3 - uart_sense_matrix (MATLAB) - For HIL
//////////////////////////////////////////////

//Helper Modules
#include <MATLAB/MATLAB.h>

//Serial Module
#include "serial.h"

//Telemetry Module
#include "telemetry.h"

class uart {
 private:
  MATLAB uart_ctl_matrix,uart_sense_matrix;
 public:
  uart();
  void init(MATLAB sense_matrix,MATLAB ctl_matrix);
}




#endif
