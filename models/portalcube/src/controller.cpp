//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "controller.h"


//Constructor
controller::controller() {
};

//Initialization
void controller::init(MATLAB in_configuration_matrix) {
  control_matrix.zeros(NUMSIGNALS,1,"Control Signals"); //The standards must be TAERA1A2A3A4
  printf("Controller Received Configuration Matrix \n");
  in_configuration_matrix.disp();
}

//Main Control Loop
void controller::loop(MATLAB rx_matrix,MATLAB sense_matrix) {
  printf("Control Loop \n");
}

