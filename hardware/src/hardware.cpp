#include "hardware.h"

//Constructor
hardware::hardware() {
}

//Initialization Routine
void hardware::init(char root_folder_name[],int NUMSIGNALS) {
  printf("Root Folder Name = %s \n",root_folder_name);

  //Initialize the RCIO
  rc.outInit(NUMSIGNALS);

  //The hardware initialization needs to import the Simulation.txt file
  //the config.txt file and the errors.txt file
  //This will create the in_configuration_matrix and in_simulation_matrix
  
  //////////////////////////SIMULATION.TXT///////////////////////
  char simfile[256]={NULL};
  strcat(simfile,root_folder_name);
  strcat(simfile,"Input_Files/Simulation.txt");
  int ok = logger.ImportFile(simfile,&in_simulation_matrix,"simulation_matrix",-99);
  if (!ok) { exit(1); }// else {in_simulation_matrix.disp();}
  //////////////////////////CONFIG.TXT////////////////////////////
  char configfile[256]={NULL};
  strcat(configfile,root_folder_name);
  strcat(configfile,"Input_Files/Config.txt");
  ok = logger.ImportFile(configfile,&in_configuration_matrix,"configuration_matrix",-99);
  if (!ok) { exit(1); }// else {in_configuration_matrix.disp();}

  /////EXTRACT DATA FROM CONFIG FILE
  PRINTRATE = in_configuration_matrix.get(1,1);
  LOGRATE = in_configuration_matrix.get(2,1);
  RCRATE = in_configuration_matrix.get(3,1);

  //Pick the IMU you want to use
  //0 = MPU9250
  //1 = LSM9DS1
  int IMUTYPE = in_configuration_matrix.get(5,1);
  sense.initIMU(IMUTYPE);

  //Set the Filter Constant
  sense.orientation.FilterConstant = in_configuration_matrix.get(6,1);
  
  //Initialize Logger
  logger.init("data/",sense.getNumVars()+1);
  //Set and log headers
  logger.appendheader("Time (sec)");
  logger.appendheaders(sense.headernames,sense.getNumVars());
  logger.printheaders();
}

//This version of the loop runs assuming you are saving data from the simulation 
//environment
void hardware::send(MATLAB model_matrix) {
  //X,Y,Z
  sense.satellites.X = model_matrix.get(1,1);
  sense.satellites.Y = model_matrix.get(2,1);
  sense.satellites.Z = model_matrix.get(3,1);
  //We need to reset the GPS values if we haven't gotten a valid coorinate yet
  sense.satellites.reset();
  //printf("%lf %lf %lf \n",sense.satellites.X,sense.satellites.Y,sense.satellites.Z);
  //PAUSE();

  //Quaternions
  //model_matrix.disp();
  sense.orientation.ahrs.q0 = model_matrix.get(4,1);
  sense.orientation.ahrs.q1 = model_matrix.get(5,1);
  sense.orientation.ahrs.q2 = model_matrix.get(6,1);
  sense.orientation.ahrs.q3 = model_matrix.get(7,1);

  //Set gx,gy,gz from sense_matrix for filtering (notice the wierd ordering)
  sense.orientation.gy = model_matrix.get(11,1);
  sense.orientation.gx = model_matrix.get(12,1);
  sense.orientation.gz = -model_matrix.get(13,1);

  //Still need all the other sensor states but not right now

}

//This version of the loop runs assuming you are polling from real hardware
void hardware::loop(double currentTime,double elapsedTime,MATLAB control_matrix) {
  //Here we poll the receiver
  if (currentTime > nextRCtime) {
    rc.read();
    nextRCtime=currentTime+RCRATE;
  }

  //Poll all as quickly as possible sensors - This puts all raw data into the sense matrix
  sense.poll(currentTime,elapsedTime);

  //I then need to populate the pwm_array with the control signals as quickly as possible
  for (int i = 0;i<rc.out.NUMSIGNALS;i++) {
    rc.out.pwm_array[i] = control_matrix.get(i+1,1);
  }

  if (currentTime > nextLOGtime) {
    logger.printvar(currentTime);
    logger.println(sense.sense_matrix);
    nextLOGtime=currentTime+LOGRATE;
  }

  //And then send the pwm_array to the servos and ESCs as quickly as possible
  //The write function has a built in saturation block no need to worry there
  rc.out.write();
}
