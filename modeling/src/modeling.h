#ifndef MODELING_H
#define MODELING_H

////Modeling Routine that only runs in SIMONLY, SIL and HIL modes

//Helper Modules
#include <MATLAB/MATLAB.h>
#include <Timer/timer.h>
#include <RK4/RK4.h>
#include <Rotation/Rotation3.h>
#include <Datalogger/Datalogger.h>

///If running SIL or HIL and on a desktop (anything with a rendering environment)
//we need to turn on OPENGL this way the PIC actually has something to fly.
//Note the rendering environment does not include the 
#if defined (SIL) || (HIL)
#if defined (DESKTOP)
#define OPENGL
#include <opengl/opengl.h>
#endif
#endif

///Specific header files for the vehicle being simulated
#include "forces.h"

/////INPUTS TO MODELING
// 1 - Root Folder Name
// 2 - UART Control Matrix (MATLAB) or Control_Matrix
// 3 - Simulation Matrix

/////OUTPUTS FROM MODELING
// 1 - Model Matrix (MATLAB)

class modeling {
 private:
  double LOGRATE = 0.1;
  double nextLOGtime = 0;
  char** headernames;
  Datalogger logger;
  RK4 integrator;
  int NUMVARS,NUMINTEGRATIONSTATES;
  MATLAB State,k,I,pqr,cgdotI,cgdotB,ptpdot,FTOTALI,FTOTALB,FGNDB,MGNDB,MTOTALI,MTOTALB;
  MATLAB pqrdot,Iinv,q0123,I_pqr,uvwdot,pqrskew_I_pqr,Kuvw_pqr,state,statedot;
  MATLAB actuatorState,actuatorStatedot,actuatorErrorPercentage,cg,ptp,BVECB;
  MATLAB actuatorTimeConstants,BVECB_Tesla;
  double mass,tlastRCread=-99,tlastCTL=-99,tRC,tCTL;
  Rotation3 ine2bod321;
  //environment env;
  //double ACTUATOR_ERROR_PERCENT;
  void rk4step(double,int[]);
  void Derivatives(double,int[]);
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
