#include "uart.h"

//Constructor
UART::UART() {
}

//The UART class needs to know how big the sense and ctl matrices
//are
//The telemetry class is completely set up by the USER
void UART::init(MATLAB sense_matrix,int NUMSIGNALS,int num) {
  //First grab the telemetry number
  NUMTELEMETRY = num;
  //Then initialize the telemetry array
  telemetry_array = (float *) calloc(NUMTELEMETRY,sizeof(float));
  //From here we initialize the uart arrays
  uart_sense_array = (float *) calloc(sense_matrix.len(),sizeof(float));
  uart_ctl_array = (float *) calloc(NUMSIGNALS,sizeof(float));

  //We now set up communications depending on the type of comms we want
  #ifdef RPI
  //This means we are on the raspberry pi. Either we are int auto
  //mode or in HIL mode. Either way we set up the port like this
  comms.SerialInit("/dev/ttyAMA0",baudRate);
  #endif

  #ifdef DESKTOP
  //This means we are on the desktop. Again either in AUTO or in
  //HIL but no matter what we open the port
  comms.SerialInit("/dev/ttyUSB0",baudRate);
  #endif
  
}

void UART::readSense(MATLAB sense_matrix) {
  //This function will read the sense array over UART
  // ----
  //It will then overwrite the sense matrix for use elsewhere
  for (int i = 1;i<=sense_matrix.len();i++) {
    sense_matrix.set(i,1,uart_sense_array[i-1]);
  }
}

void UART::readControl(MATLAB ctl_matrix) {
  //This function will read the ctl array over UART
  // ----
  //It will then overwrite the ctl matrix for use elsewhere
  for (int i = 1;i<=ctl_matrix.len();i++) {
    ctl_matrix.set(i,1,uart_ctl_array[i-1]);
  }
}

//This will send the sense matrix over UART to the Hardware device
void UART::sendSense(MATLAB sense_matrix) {
  //First copy the sense matrix to the uart_sense_array
  for (int i = 1;i<=sense_matrix.len();i++) {
    uart_sense_array[i-1] = sense_matrix.get(i,1);
  }
  //Then send it over UART
  comms.SerialSendArray(uart_sense_array,sense_matrix.len(),1);
}

void UART::sendControl(MATLAB ctl_matrix) {
  //First copy the control matrix to the uart_ctl_array
  for (int i = 1;i<=ctl_matrix.len();i++) {
    uart_ctl_array[i-1] = ctl_matrix.get(i,1);
  }
  //Then send it over UART
  comms.SerialSendArray(uart_ctl_array,ctl_matrix.len(),1);
}

void UART::sendTelemetry(MATLAB telemetry_matrix) {
  //It's assumed that whomever is calling this function
  //Also called the init function with the proper matrix
  //size thus we first copy the telemetry_matrix over to telemetry_array
  //telemetry_matrix.disp();
  for (int i = 1;i<=telemetry_matrix.len();i++) {
    telemetry_array[i-1] = telemetry_matrix.get(i,1);
  }
  //Then send it over UART
  comms.SerialSendArray(telemetry_array,NUMTELEMETRY,0);
}
