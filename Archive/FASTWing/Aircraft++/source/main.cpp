//Alright. Time for some c++ code.
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include "Vehicle.h"
#include "Connection.h"

using namespace std;

char input_file[256];
Connection meta;

//First thing to do is grab the input argument if it exists.
int main(int argc,char* argv[]) {

  cout << "Number of Input Arguments: " << argc << endl;

  //Compute the number of aircraft based on number of input arguments
  int num_aircraft = int(argc-1);

  if (num_aircraft == 0) {
    cout << "Sorry no input file given" << endl;
    cout << "Defaulting to Input_Files/A++.files" << endl;
    num_aircraft = 1;
    sprintf(input_file,"Input_Files/A++.files");
  } else if (num_aircraft == 1) {
    cout << "Input File Given = " << argv[1] << endl;
    sprintf(input_file,argv[1]);
  }

  cout << "Number of Aircraft to Simulation: " << num_aircraft << endl;

  //Then we can create the vehicle class which also imports the input deck
  if (num_aircraft == 1) {
    Vehicle a;
    a.init(input_file);
    //Then we can integrate the equations of motion
    if (a.ok) {
      cout << "Integrating..." << endl;
      a.Integrate();
    } else {
      cout << "Error in Vehicle Class" << endl;
    }
  } else {
    int ok = meta.init(num_aircraft,argv);
    if (ok == num_aircraft) {
      cout << "Meta Import Successful" << endl;
      meta.Integrate();
    } else {
      cout << "Error import one or more vehilces" << endl;
    }
  }
  
  
  
}
