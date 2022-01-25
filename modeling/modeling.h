#ifndef MODELING_H
#define MODELING_H

////Modeling Routine that only runs in SIMONLY, SIL and HIL modes

//Helper Modules
#include <MATLAB/MATLAB.h>

///If running SIL or HIL and on a desktop (anything with a rendering environment)
//we need to turn on OPENGL this way the PIC actually has something to fly.
//Note the rendering environment does not include the 
#if defined (SIL) || (HIL)
#if defined (DESKTOP)
#define OPENGL
#include <modeling/opengl.h>
#endif
#endif

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
