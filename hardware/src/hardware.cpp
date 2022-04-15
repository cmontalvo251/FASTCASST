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
  //and the config.txt file
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

  sense.init(in_configuration_matrix,in_simulation_matrix);

  //Extract Number of SIGNALS and set headers
  pwmnames = (char**)malloc((NUMSIGNALS)*sizeof(char*));
  for (int i = 1;i<=NUMSIGNALS;i++) {
    pwmnames[i-1] = (char*)malloc((18)*sizeof(char));
    sprintf(pwmnames[i-1],"PWM Hardware Out %d",i);
  }

  //Initialize Logger
  logger.init("data/",sense.getNumVars()+1+NUMSIGNALS); //+1 for time
  //Set and log headers
  logger.appendheader("Time (sec)");
  logger.appendheaders(sense.headernames,sense.getNumVars());
  logger.appendheaders(pwmnames,NUMSIGNALS);
  logger.printheaders();

  //Initialize the Telemetry or HIL
  //(I think right now we can only do HIL or telemetry. We can't do both)
  NUMTELEMETRY = 6; //Set the actual values in the loop function
  telemetry_matrix.zeros(NUMTELEMETRY,1,"Telemetry Matrix HW");
  ser.init(sense.sense_matrix,NUMSIGNALS,NUMTELEMETRY);
}

//This version of the loop runs assuming you are saving data from the simulation 
//environment
void hardware::send(double currentTime,MATLAB model_matrix,double keyboardVars[]) {

  //Send Model Matrix to sensor class
  sense.send(currentTime,model_matrix);

  //We also need to set the temperature in the modeling matrix
  //I don't think we really need this in the model but whatever
  model_matrix.set(30,1,sense.getTemperature());
  
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
    //printf("Read RX %lf \n",currentTime);
    rc.read();
    //rc.in.printRCstate(-4);
    //printf("\n");
    nextRCtime=currentTime+RCRATE;
  }

  //Poll all as quickly as possible sensors - This puts all raw data into the sense matrix
  sense.poll(currentTime,elapsedTime);

  //I then need to populate the pwm_array with the control signals as quickly as possible
  for (int i = 0;i<rc.out.NUMSIGNALS;i++) {
    rc.out.pwm_array[i] = control_matrix.get(i+1,1);
    //However, the control routine is created by the user and doesn't necessarily have a
    //saturation filter built in so we need to do that here
    if (rc.out.pwm_array[i] > OUTMAX) {
      rc.out.pwm_array[i] = OUTMAX;
    } else if (rc.out.pwm_array[i] < OUTMIN) {
      rc.out.pwm_array[i] = OUTMIN;
    }
  }
  //printf("ELEVATOR ARRAY = %d \n",rc.out.pwm_array[2]);

  //Check to see if it's time to log
  if (currentTime >= nextLOGtime) {
    //printf("Hardware Logging %lf \n",currentTime);
    logger.printvar(currentTime);
    logger.print(sense.sense_matrix);
    logger.printarrayln(rc.out.pwm_array,rc.out.NUMSIGNALS);
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
    //printf("Sending Telemetry %lf \n",currentTime);
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
