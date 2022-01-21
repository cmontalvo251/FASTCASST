#ifndef MODELING_H
#define MODELING_H

////Modeling Routine that only runs in SIMONLY, SIL and HIL modes

//Helper Modules
#include <MATLAB/MATLAB.h>

/////INPUTS TO MODELING
// 1 - Root Folder Name
// 2 - UART Control Matrix (MATLAB) or Control_Matrix
// 3 - Simulation Matrix

/////OUTPUTS FROM MODELING
// 1 - Model Matrix (MATLAB)

class modeling {
 private:
 public:
  //constructor
  modeling();
  //initialization routine
  void init(char root_folder_name,MATLAB in_simulation_matrix);
};

#endif
