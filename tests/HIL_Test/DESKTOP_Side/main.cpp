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
  printf("Desktop Serial Send Test Script \n");
  printf("Main Loop Begin \n");
  
  double NUMTELEMETRY = 2; //Set the actual values in the loop function
  double x = 0;
  float *uart_telemetry_array;
  uart_telemetry_array = (float *) calloc(NUMTELEMETRY,sizeof(float));
  int baudRate = 57600; //Hardcode. I don't think we ever need to change
  Serial comms;
  //comms.SerialInit("/dev/ttyUSB0",baudRate);
  comms.SerialInit("/dev/ttyUSB0",baudRate);

  //Initialize the Timer if we're running in Software mode
  double initTime = 0;
  double nextTELEMtime = 0;
  double TELEMRATE = 1;
  watch.init(0);
  double currentTime = watch.currentTime;
  int READMODE = 0; //Initialize to be constantly in WRITEMODE

  ///INFINITE WHILE LOOP
  while (1) {
    if (READMODE) {
      ////READMODE
      printf("READ MODE CURRENT TIME = %lf \n",currentTime);
      //Receive UART
      printf("RUNNING Get Array \n");
      comms.SerialGetArray(uart_telemetry_array,NUMTELEMETRY,0);
      printf("VARS RECEIVED = %lf %lf \n",uart_telemetry_array[0],uart_telemetry_array[1]);
      READMODE = 0; //Comment this out if you just want to be in read mode forever
    } else {
      //WRITE MODE
      if (currentTime > nextTELEMtime) { 
      //Set up uart array
        uart_telemetry_array[0] = currentTime;
        uart_telemetry_array[1] = x;
        //Then send it over UART
        comms.SerialSendArray(uart_telemetry_array,NUMTELEMETRY,1);
        nextTELEMtime=currentTime+TELEMRATE;
        x+=10;
        //READMODE = 1; //Comment this out if you want to be in write mode forever
      }
    }
      
    //Update Timer
    watch.updateTime();
    currentTime = watch.currentTime;
    
  }
}
