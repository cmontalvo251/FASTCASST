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
  printf("RPI Serial Send Test Script \n");
  printf("Main Loop Begin \n");
  
  double NUMTELEMETRY = 2; //Set the actual values in the loop function
  double x = 0;
  float *uart_telemetry_array;
  uart_telemetry_array = (float *) calloc(NUMTELEMETRY,sizeof(float));
  int baudRate = 57600; //Hardcode. I don't think we ever need to change
  Serial comms;
  //comms.SerialInit("/dev/ttyUSB0",baudRate);
  comms.SerialInit("/dev/ttyAMA0",baudRate);

  //Initialize the Timer if we're running in Software mode
  double initTime = 0;
  double nextTELEMtime = 0;
  double TELEMRATE = 1;
  watch.init(0);
  double currentTime = watch.currentTime;

  ///INFINITE WHILE LOOP
  while (1) {
    //Send Telemetry
    if (currentTime >= nextTELEMtime) {
      //Set up uart array
      uart_telemetry_array[0] = currentTime;
      uart_telemetry_array[1] = x;
      //Then send it over UART
      comms.SerialSendArray(uart_telemetry_array,NUMTELEMETRY,1);
      nextTELEMtime=currentTime+TELEMRATE;
      x+=10;
    }
      
    //Update Timer
    watch.updateTime();
    currentTime = watch.currentTime;
    
  }
}
