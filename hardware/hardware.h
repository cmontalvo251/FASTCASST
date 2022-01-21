#ifndef HARDWARE_H
#define HARDWARE_H

//Helper Modules
#include <MATLAB/MATLAB.h>

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
 public:
  //Initialization routine needs the root folder name
  void init(char root_folder_name);
  //Constructor
  hardware();
}

#endif
