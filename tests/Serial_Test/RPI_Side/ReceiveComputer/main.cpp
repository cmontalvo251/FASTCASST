///The Main source files have no header files
//Everything is contained in here
#include <stdio.h>
using namespace std;

//Timer
#include <Timer/timer.h>
TIMER watch;

//Serial Module
#include <UART/Serial.h>

int main(int argc,char* argv[]) {
  //Print name of software
  printf("RPI Receive Test Script \n");
  printf("Main Loop Begin \n");
  
  double NUMTELEMETRY = 2; //Set the actual values in the loop function
  double x = 0;
  float *uart_telemetry_array;
  uart_telemetry_array = (float *) calloc(NUMTELEMETRY,sizeof(float));
  int baudRate = 57600; //Hardcode. I don't think we ever need to change
  Serial comms;
  printf("Serial Init \n");
  //comms.SerialInit("/dev/ttyUSB0",baudRate);
  comms.SerialInit("/dev/ttyAMA0",baudRate);
  printf("Serial Init done...\n");

  //Initialize the Timer if we're running in Software mode
  double initTime = 0;
  double nextTELEMtime = 0;
  double TELEMRATE = 0.0;
  watch.init(0);
  double currentTime = watch.currentTime;

  ///INFINITE WHILE LOOP
  while (1) {
    if (currentTime >= nextTELEMtime) {
      //Receive UART
      //printf("RUNNING Get Array \n");
      comms.SerialGetArray(uart_telemetry_array,NUMTELEMETRY,0);
      printf("VARS RECEIVED = %lf %lf \n",uart_telemetry_array[0],uart_telemetry_array[1]);
      nextTELEMtime=currentTime+TELEMRATE;
    }
      
    //Update Timer
    watch.updateTime();
    currentTime = watch.currentTime;
    
  }
}
