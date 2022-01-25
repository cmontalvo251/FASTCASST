#include "modeling.h"

//Constructor
modeling::modeling() {
}

//Initialization
void modeling::init(char root_folder_name[],MATLAB in_simulation_matrix) {
  printf("Modeling Root Folder Name = %s \n",root_folder_name);
  //in_simulation_matrix.disp();

  ////////EXTRACT SIMULATION MATRIX VARIABLES
  TFINAL=in_simulation_matrix.get(1,1);
  TIMESTEP=in_simulation_matrix.get(2,1);
}

///Loop
void modeling::loop(double currentTime) {
  //printf("Modeling Loop \n");
  if (currentTime > TFINAL) {
    //Simulation is over.
    ok = 0;
    //break
    return;
  }
}
