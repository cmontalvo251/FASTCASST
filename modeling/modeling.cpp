#include "modeling.h"

//Constructor
modeling::modeling() {
}

//Initialization
void modeling::init(char root_folder_name,MATLAB in_simulation_matrix) {
  printf("Modeling Root Folder Name = %s \n",root_folder_name);
  in_simulation_matrix.disp();
}
