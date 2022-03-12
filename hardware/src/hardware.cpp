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
  TELEMRATE = in_configuration_matrix.get(4,1);

  //Pick the IMU you want to use
  //0 = MPU9250
  //1 = LSM9DS1
  int IMUTYPE = in_configuration_matrix.get(5,1);
  sense.initIMU(IMUTYPE);

  //Set the Filter Constant
  sense.orientation.FilterConstant = in_configuration_matrix.get(6,1);

  //Initialize q0123 and ptp
  q0123.zeros(4,1,"Sense Quaternions");
  ptp.zeros(3,1,"Sense Roll Pitch Yaw");
  
  //Initialize Logger
  logger.init("data/",sense.getNumVars()+1);
  //Set and log headers
  logger.appendheader("Time (sec)");
  logger.appendheaders(sense.headernames,sense.getNumVars());
  logger.printheaders();

  //Initialize the Telemetry or HIL
  //(I think right now we can only do HIL or telemetry. We can't do both)
  NUMTELEMETRY = 6; //Set the actual values in the loop function
  telemetry_matrix.zeros(NUMTELEMETRY,1,"Telemetry Matrix HW");
  ser.init(sense.sense_matrix,NUMSIGNALS,NUMTELEMETRY);
}

//This version of the loop runs assuming you are saving data from the simulation 
//environment
void hardware::send(MATLAB model_matrix,double keyboardVars[]) {
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

  //Convert the quaternions to Euler Angles
  //There's no need to do this. 
  //The hardware loop routine will convert to Euler Angles
  //q0123.vecset(1,4,model_matrix,4);
  //q0123.disp();
  //ptp.quat2euler(q0123);
  //ptp.disp();
  //sense.orientation.roll = ptp.get(1,1)*180/PI;
  //sense.orientation.pitch = ptp.get(2,1)*180/PI;
  //sense.orientation.yaw = ptp.get(3,1)*180/PI;
  //printf("send() %lf %lf %lf \n",sense.orientation.roll,sense.orientation.pitch,sense.orientation.yaw);

  //Set gx,gy,gz from sense_matrix for filtering 
  sense.orientation.gy = model_matrix.get(11,1);
  sense.orientation.gx = model_matrix.get(12,1);
  sense.orientation.gz = -model_matrix.get(13,1);

  //Still need all the other sensor states but not right now

  //Is using the keyboard you need to copy over the keyboard values
  //to rcin
  #ifdef KEYBOARD
  for (int i = 0;i<4;i++) {
    rc.in.keyboard[i] = keyboardVars[i];
  }
  //printf("Key = ");
  //for (int i = 0;i<4;i++) {
  //printf("%lf ",keyboardVars[i]);
  //}
  //printf("\n");
  #endif
}

//This version of the loop runs assuming you are polling from real hardware
void hardware::loop(double currentTime,double elapsedTime,MATLAB control_matrix) {
  //Here we poll the receiver
  if (currentTime >= nextRCtime) {
    rc.read();
    nextRCtime=currentTime+RCRATE;
  }

  //Poll all as quickly as possible sensors - This puts all raw data into the sense matrix
  sense.poll(currentTime,elapsedTime);

  //I then need to populate the pwm_array with the control signals as quickly as possible
  for (int i = 0;i<rc.out.NUMSIGNALS;i++) {
    rc.out.pwm_array[i] = control_matrix.get(i+1,1);
  }

  //Check to see if it's time to log
  if (currentTime >= nextLOGtime) {
    logger.printvar(currentTime);
    logger.println(sense.sense_matrix);
    nextLOGtime=currentTime+LOGRATE;
  }

  //Send telemetry data. Always running to make sure
  //it works in SIL mode and SIMONLY.
  //Unfortunately I have to turn off telemetry in HIL mode
  //Because we are using the radios for HIL right now
  //If I find another way to send data from a computer
  //to an RPI I will change this.
  #ifndef HIL
  if (currentTime >= nextTELEMtime) {
    //For right now let's send RPY and GPS coordinates
    telemetry_matrix.set(1,1,currentTime);
    telemetry_matrix.set(2,1,sense.orientation.roll);
    telemetry_matrix.set(3,1,sense.orientation.pitch);
    telemetry_matrix.set(4,1,sense.orientation.yaw);
    telemetry_matrix.set(5,1,sense.satellites.latitude);
    telemetry_matrix.set(6,1,sense.satellites.longitude);
    ser.sendTelemetry(telemetry_matrix);
    nextTELEMtime=currentTime+TELEMRATE;
  }
  #endif

  //And then send the pwm_array to the servos and ESCs as quickly as possible
  //The write function has a built in saturation block no need to worry there
  //Right now we have to shut this off when using the satellite because
  //Satellites don't use servos
  #ifndef satellite
  rc.out.write();
  #endif
}
