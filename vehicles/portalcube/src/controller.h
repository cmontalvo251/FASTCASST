#ifndef CONTROLLER_H
#define CONTROLLER_H

//This is another class that is craft dependent and as such
//must adhere to specific standards

/////////////INPUTS TO CONTROLLER/////////////////
// 1 - RX Matrix (MATLAB)
// 2 - Sense Matrix (MATLAB)
// 3 - Configuration Matrix (MATLAB)

////////////OUTPUTS OF CONTROLLER////////////////
// 1 - Control Matrix (MATLAB) x2 (One to Modeling and one to hardware)

//Needed for matrices
#include <MATLAB/MATLAB.h>

class controller {
private:
  int NUMSIGNALS=4; //Number of control signals
public:
  MATLAB control_matrix;
  void loop(MATLAB rx_matrix,MATLAB sense_matrix);
  void init(MATLAB in_configuration_matrix); //var is open ended right now
  //constructor
  controller();
};

#endif
