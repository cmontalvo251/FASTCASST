#include "Comms.h"

//Constructor
Comms::Comms() {
  #ifdef ARDUINO
  baudRateWireless = 9600;
  baudRate = 9600; //Every example online shows 9600 but we may need to try 57600
  #else
  baudRateWireless = 57600;
  baudRate = 57600; //115200 on rpi3 did not work
  #endif
}

//The Comms class needs to know how big the send array is

//There are two functions for initializing HIL functions and one for telemetry
void Comms::TelemInit(int numtelem) {
  //First grab the telemetry number
  NUMTELEMETRY = numtelem;
  //Then initialize the Comms send array
  comms_telemetry_array = (float *) calloc(NUMTELEMETRY,sizeof(float));

  //We now set up communications depending on the type of comms we want
  printstdout("SETTING UP TELEMETRY \n");
  #if defined (RPI) || (ARDUINO)
  //Telemetry happens here
  #ifdef RPI
  char* portName = "/dev/ttyAMA0";
  #endif
  #ifdef ARDUINO
  char* portName = "Serial2";
  #endif
  uart.SerialInitWireless(portName,baudRateWireless);
  #endif

  #ifdef DESKTOP
  //This means we are on the desktop.
  //Telemetry on the desktop goes to a file but only when
  //running in SIMONLY or SIL mode
  //Run default init function which has the txt file bindings
  //There are #define in the Serial.cpp file that makes that happen
  uart.InitSerialPort(); 
  #endif
  
}

void Comms::HILInit(int numsens,int numc) {
  //Grab NUMSENSE AND NUMCTL
  NUMSENSE = numsens;
  NUMCTL = numc;
  //Then initialize the Comms send array
  comms_sense_array = (float *) calloc(NUMSENSE,sizeof(float));
  comms_ctl_array = (float *) calloc(NUMCTL,sizeof(float));

  //This routine only runs if HIL is on so we don't need a #define HIL here

  //We now set up communications depending on the type of comms we want
  printf("SETTING UP HIL COMMS \n");
  #ifdef RPI
  //HIL comms happens here
  hiluart.SerialInit("/dev/ttyUSB0",baudRate);
  //hiluart.SerialInit("/dev/ttyAMA0",baudRate);
  #endif

  #ifdef DESKTOP
  //This means we are on the desktop.
  //Hardware in the loop communication happens via USB0
  hiluart.SerialInit("/dev/ttyUSB0",baudRate);
  #endif
  
}

//This will send the sense matrix over Comms to the Hardware device
void Comms::sendSense(MATLAB comms_sense_matrix) {
  //First copy the sense matrix to the comms_sense_array
  for (int i = 1;i<=comms_sense_matrix.len();i++) {
    comms_sense_array[i-1] = comms_sense_matrix.get(i,1);
  }
  //Then send it over comms
  hiluart.SerialSendArray(comms_sense_array,comms_sense_matrix.len(),0);
  //PAUSE();
}


void Comms::readSense(MATLAB comms_sense_matrix) {
  //This function will read the sense array over UART
  hiluart.SerialGetArray(comms_sense_array,comms_sense_matrix.len(),0);
  //It will then overwrite the sense matrix for use elsewhere
  for (int i = 1;i<=comms_sense_matrix.len();i++) {
    comms_sense_matrix.set(i,1,comms_sense_array[i-1]);
  }
}

void Comms::sendControl(MATLAB comms_ctl_matrix) {
  //First copy the control matrix to the comms_ctl_array
  for (int i = 1;i<=comms_ctl_matrix.len();i++) {
    comms_ctl_array[i-1] = comms_ctl_matrix.get(i,1);
  }
  //Then send it over UART
  hiluart.SerialSendArray(comms_ctl_array,comms_ctl_matrix.len(),0);
}

int Comms::readControl(MATLAB comms_ctl_matrix) {
  //This function will read the ctl array over UART
  hiluart.SerialGetArray(comms_ctl_array,comms_ctl_matrix.len(),0);
  //comms_ctl_matrix.disp();
  //PAUSE();
  //It will then overwrite the ctl matrix for use elsewhere
  for (int i = 1;i<=comms_ctl_matrix.len();i++) {
    comms_ctl_matrix.set(i,1,comms_ctl_array[i-1]);
  }
  return 0; //error code doesn't work quite yet
}

void Comms::sendTelemetry(MATLAB comms_telemetry_matrix,int echo) {
  //It's assumed that whomever is calling this function
  //Also called the init function with the proper matrix
  //size thus we first copy the comms_send_matrix over to comms_send_array
  //telemetry_matrix.disp();
  for (int i = 1;i<=comms_telemetry_matrix.len();i++) {
    comms_telemetry_array[i-1] = comms_telemetry_matrix.get(i,1);
  }
  //Then send it over UART
  uart.SerialSendArray(comms_telemetry_array,comms_telemetry_matrix.len(),echo);
}
