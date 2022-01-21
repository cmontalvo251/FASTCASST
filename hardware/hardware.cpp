#include "hardware.h"

//Constructor
hardware::hardware() {
}

//Initialization Routine
void hardware::init(char root_folder_name) {
  printf("Root Folder Name = %s \n",root_folder_name);
}

//Initialize Hardware Control Matrix
void harware::initControlMatrix(MATLAB control_matrix) {
  hw_control_matrix.init(control_matrix.row_,control_matrix.col_,"hw_control_matrix");
}
			
//Receive Control Matrix
void hardware::getControlMatrix(MATLAB control_matrix) {
  hw_control_matrix.overwrite(control_matrix);
}

