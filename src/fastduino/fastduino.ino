//Note that you can't define #defines in this ino script to make everything Arduino specific
//You do have access to ARDUINO so just use #ifdef ARDUINO if you want to do something ARDUINO specific
//Furthermore, you can't import text files on an Arduino so everything
//in Simulation.txt and Config.txt must be placed in here

//Note when setting this up for the first time. 
//Install the Due board by going to the board manager
//Go to preferences and change the location of the libraries to ~/FASTCASST

//Timer.h for realtime clock - Tested on 2/28/2023 and it compiles and runs
#include "timer.h"
TIMER watch;

//Include the hardware environment
//#include "hardware.h"

///DEBUG HEADER FILES
//#include "mathp.h"

//Need MATLAB.h for Matrices
//#include "MATLAB.h"

//Datalogger is inside hardware.h
//#include "Datalogger.h"
//Datalogger logger;

//RCIO is inside hardware.h
//#include "RCIO.h"
//RCIO rc;

//RCInput is inside RCIO.h - Compiles on 2/28/2023 and Runs on 2/28/2023
#include "RCInput.h"
RCInput rin;

//RCOutput is inside RCIO.h
//#include "RCOutput.h"
//RCOutput rout;

//PWMSIGNALS.h is inside RCIO.h
//#include "PWMSIGNALS.h"

void setup() {
  //Setup Serial Std Out
  Serial.begin(115200);

  ///Print dummy version number
  Serial.print("FASTKit Software Version 42.0 \n");
  
  ///Initialize the timer
  Serial.print("Initiailizing Timer...\n");
  watch.init(0);
  Serial.print("Timer Initialized \n");
  
  //Hardware init

  //DEBUGGING
  //Initialize Datalogger
  //logger.init("data/",1+RECV_N_CHANNEL); //Time plus the receiver signals - Remember the SD card on the arduino needs to have a data folder
  //rc.outInit(RECV_N_CHANNEL);
  rin.initialize();
  //rout.initialize(RECV_N_CHANNEL);
  
}

void loop() {
  //Update Timer
  watch.updateTime();
  //Update RC Signals
  rin.readRCstate();
  //rc.read();  
  
  //Print Everything
  Serial.print("T = ");
  Serial.print(watch.currentTime);
  Serial.print(" RX = ");
  rin.printRCstate(-6);
  //rc.in.printRCstate(-5);
  Serial.print(" PWM = ");
  //rc.out.print();
  Serial.print("\n");
  cross_sleep(0.1);
}
