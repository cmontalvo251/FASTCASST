#include "modeling.h"

//Constructor
modeling::modeling() {
}

//Initialization
void modeling::init(char root_folder_name[],MATLAB in_simulation_matrix,MATLAB in_configuration_matrix) {
  printf("Modeling Root Folder Name = %s \n",root_folder_name);
  //in_simulation_matrix.disp();

  ////////EXTRACT SIMULATION MATRIX VARIABLES
  TFINAL=in_simulation_matrix.get(1,1);
  TIMESTEP=in_simulation_matrix.get(2,1);

  //Initialize Matrices
  NUMVARS = 30; //Make sure this is the same as the sense states
  model_matrix.zeros(NUMVARS,1,"Model Matrix"); 
  NUMINTEGRATIONSTATES=13; //Only integrating 13 states for a 6DOF system
  integration_matrix.zeros(NUMINTEGRATIONSTATES,1,"Integration Matrix");

  //Set initial conditions of integration matrix
  for (int i = 1;i<13;i++) {
    integration_matrix.set(i,1,in_simulation_matrix.get(i+2,1));
  }
  //integration_matrix.disp();
  //PAUSE();

  //Copy the states over to the model matrix for opengl and hardware loops
  model_matrix.vecset(1,13,integration_matrix,1);
}

///Loop
void modeling::loop(double currentTime,int pwm_array[]) {
  //printf("Modeling Loop \n");
  if (currentTime > TFINAL) {
    //Simulation is over.
    ok = 0;
    //break
    return;
  }

  //Run RK4 Loop
  rk4step(pwm_array);

  //Copy the states over to the model matrix for opengl and hardware loops
  model_matrix.vecset(1,13,integration_matrix,1);

  //Add sensor noise if needed
}

void modeling::rk4step(int pwm_array[]) {

}