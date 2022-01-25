#include "hardware.h"

//Constructor
hardware::hardware() {
}

//Initialization Routine
void hardware::init(char root_folder_name[]) {
  printf("Root Folder Name = %s \n",root_folder_name);

  //The hardware initialization needs to import the Simulation.txt file
  //the config.txt file and the errors.txt file
  //This will create the in_configuration_matrix and in_simulation_matrix
  
  //////////////////////////SIMULATION.TXT///////////////////////
  char simfile[256]={NULL};
  strcat(simfile,root_folder_name);
  strcat(simfile,"Input_Files/Simulation.txt");
  int ok = logger.ImportFile(simfile,&in_simulation_matrix,"simulation_matrix",-99);
  if (!ok) { exit(1); } else {in_simulation_matrix.disp();}
  //////////////////////////CONFIG.TXT////////////////////////////
  char configfile[256]={NULL};
  strcat(configfile,root_folder_name);
  strcat(configfile,"Input_Files/Config.txt");
  ok = logger.ImportFile(configfile,&in_configuration_matrix,"configuration_matrix",-99);
  if (!ok) { exit(1); } else {in_configuration_matrix.disp();}
  
  //Initialize Logger
  //logger.init("data/",sense.getNumVars());
  //Set and log headers
  //logger.appendheaders(sense.headernames,sense.getNumVars());
  //logger.printheaders();
}

