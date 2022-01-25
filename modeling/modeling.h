#ifndef MODELING_H
#define MODELING_H

////Modeling Routine that only runs in SIMONLY, SIL and HIL modes

//Helper Modules
#include <MATLAB/MATLAB.h>
#include <Timer/timer.h>

///If running SIL or HIL and on a desktop (anything with a rendering environment)
//we need to turn on OPENGL this way the PIC actually has something to fly.
//Note the rendering environment does not include the 
#if defined (SIL) || (HIL)
#if defined (DESKTOP)
#define OPENGL
#include <modeling/opengl/opengl.h>
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
    int NUMVARS,NUMINTEGRATIONSTATES;
    void rk4step();
 public:
  //Matrices
  MATLAB model_matrix,integration_matrix,model_sense_matrix;
  //Status
  int ok=1;
  //Timing
  double TFINAL=1,TIMESTEP=0.1;
  //constructor
  modeling();
  //initialization routine
  void init(char root_folder_name[],MATLAB in_simulation_matrix,MATLAB in_configuration_matrix);
  //Loop
  void loop(double currentTime,int pwm_array[]);
  //RK4step
  void rk4step(int pwm_array[]);
};

#endif
