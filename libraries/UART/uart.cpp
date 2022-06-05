#include "uart.h"

//Constructor
UART::UART() {
}

//The UART class needs to know how big the send array is
void UART::init(int numtelem,int numsens,int numc) {
  //First grab the telemetry number
  NUMTELEMETRY = numtelem;
  NUMSENSE = numsens;
  NUMCTL = numc;
  //Then initialize the uary send array
  uart_telemetry_array = (float *) calloc(NUMTELEMETRY,sizeof(float));
  uart_sense_array = (float *) calloc(NUMSENSE,sizeof(float));
  uart_ctl_array = (float *) calloc(NUMCTL,sizeof(float));

  //We now set up communications depending on the type of comms we want
  #ifdef RPI
  //This means we are on the raspberry pi.
  //Telemetry happens here
  comms.SerialInit("/dev/ttyAMA0",baudRate);
  #ifdef HIL
  //HIL comms happens here
  hilcomms.SerialInit("/dev/ttyUSB0",baudRate);
  #endif
  #endif

  #ifdef DESKTOP
  //This means we are on the desktop.
  //Hardware in the loop communication happens via USB0
  #ifdef HIL
  hilcomms.SerialInit("/dev/ttyUSB0",baudRate);
  #else
  //Telemetry on the desktop goes to a file but only when
  //running in SIMONLY or SIL mode
  //Run default init function which has the txt file bindings
  comms.InitSerialPort(); 
  #endif
  #endif
  
}

//This will send the sense matrix over UART to the Hardware device
void UART::sendSense(MATLAB uart_sense_matrix) {
  //First copy the sense matrix to the uart_sense_array
  for (int i = 1;i<=uart_sense_matrix.len();i++) {
    uart_sense_array[i-1] = uart_sense_matrix.get(i,1);
  }
  //Then send it over UART
  hilcomms.SerialSendArray(uart_sense_array,uart_sense_matrix.len(),0);
}


void UART::readSense(MATLAB uart_sense_matrix) {
  //This function will read the sense array over UART
  //hilcomms.SerialReadyArray();
  //It will then overwrite the sense matrix for use elsewhere
  for (int i = 1;i<=uart_sense_matrix.len();i++) {
    uart_sense_matrix.set(i,1,uart_sense_array[i-1]);
  }
}

void UART::sendControl(MATLAB uart_ctl_matrix) {
  //First copy the control matrix to the uart_ctl_array
  for (int i = 1;i<=uart_ctl_matrix.len();i++) {
    uart_ctl_array[i-1] = uart_ctl_matrix.get(i,1);
  }
  //Then send it over UART
  hilcomms.SerialSendArray(uart_ctl_array,uart_ctl_matrix.len(),0);
}

void UART::readControl(MATLAB uart_ctl_matrix) {
  //This function will read the ctl array over UART
  hilcomms.SerialGetArray(uart_ctl_array,uart_ctl_matrix.len(),0);
  //uart_ctl_matrix.disp();
  //PAUSE();
  //It will then overwrite the ctl matrix for use elsewhere
  for (int i = 1;i<=uart_ctl_matrix.len();i++) {
    uart_ctl_matrix.set(i,1,uart_ctl_array[i-1]);
  }
}

void UART::sendTelemetry(MATLAB uart_telemetry_matrix,int echo) {
  //It's assumed that whomever is calling this function
  //Also called the init function with the proper matrix
  //size thus we first copy the uart_send_matrix over to uart_send_array
  //telemetry_matrix.disp();
  for (int i = 1;i<=uart_telemetry_matrix.len();i++) {
    uart_telemetry_array[i-1] = uart_telemetry_matrix.get(i,1);
  }
  //Then send it over UART
  comms.SerialSendArray(uart_telemetry_array,uart_telemetry_matrix.len(),echo);
}
