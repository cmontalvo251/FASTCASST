#ifndef MODELING_H
#define MODELING_H

////Modeling Routine that only runs in SIMONLY, SIL and HIL modes

//Helper Modules
#include <MATLAB/MATLAB.h>
#include <Timer/timer.h>
#include <Rotation/Rotation3.h>
#include <Mathp/mathp.h>
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
#ifdef airplane
#include "airplane_forces.h"
#endif

///Headers required to simulate the virtual environment
#include <RK4/RK4.h>
#include <Environment/environment.h>

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
  int NUMACTUATORS;
  int IACTUATORERROR;
  double ACTUATORPERCENTERROR;
  MATLAB pwm_error,pwm_out;
  char** headernames;
  char** pwmnames;
  char** rcnames;
  Datalogger logger;
  RK4 integrator;
  int NUMVARS,NUMINTEGRATIONSTATES,FORCES_FLAG;
  MATLAB State,k,I,pqr,cgdotI,cgdotB,ptpdot,FTOTALI,FTOTALB,FGNDB,MGNDB,MTOTALI,MTOTALB;
  MATLAB pqrdot,Iinv,q0123,I_pqr,uvwdot,pqrskew_I_pqr,Kuvw_pqr,state,statedot;
  MATLAB cg,ptp,BVECB,settling_time_matrix;
  MATLAB BVECB_Tesla,actuatorStates;
  double mass,tlastRCread=-99,tlastCTL=-99,tRC,tCTL;
  Rotation3 ine2bod321;
  environment env;
  forces extforces;
  //double ACTUATOR_ERROR_PERCENT;
  void rk4step(double time,MATLAB control_matrix);
  void Derivatives(double time,MATLAB control_matrix);
  double integrationTime=0;
  MATLAB output_matrix;
  //GPS - origin set in Mathp.h
  double X_origin=GPSORIGINX,Y_origin=GPSORIGINY;
  double latitude,longitude,altitude,X,Y,Z;
  double XYZ[3],LLH[3];
  void SetGPS();
  double xprev,yprev,zprev;
  //boost::thread render;
 public:
  double keyboardVars[4];
  //Matrices
  MATLAB model_matrix,integration_matrix,model_sense_matrix;
  //Status
  int ok=1;
  //Timing
  double TFINAL=1,TIMESTEP=0.1;
  //constructor
  modeling();
  //initialization routine
  void init(char root_folder_name[],MATLAB in_simulation_matrix,MATLAB in_configuration_matrix,int argc,char** argv);
  //Loop
  void loop(double currentTime,int rx_array[],MATLAB control_matrix);
};

//Render Loop function
void renderloop(char* root_folder_name,int argc,char** argv);

#endif
