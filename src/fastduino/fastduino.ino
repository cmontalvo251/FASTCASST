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
#include "Datalogger.h"
Datalogger logger;

//RCIO is inside hardware.h
//#include "RCIO.h"
//RCIO rc;

//RCInput is inside RCIO.h - Compiles on 2/28/2023 and Runs on 2/28/2023
#include "RCInput.h"
RCInput rin;

//RCOutput is inside RCIO.h
#include "RCOutput.h"
RCOutput rout;

//PWMSIGNALS.h is inside RCIO.h
//#include "PWMSIGNALS.h"

//Hardware has sensors.h
//#include "sensors.h"

//Sensors has a lot of sensors. We're going to need to add them in one at a time
//Let's start with GPS
#include "GPS.h"
GPS satellites;

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
  logger.init("data/",1+RECV_N_CHANNEL*2+3); //Time plus the receiver signals and the PWM out signal and 3 LLH signals
  //Remember the SD card on the arduino needs to have a data/ folder
  
  //rc.outInit(RECV_N_CHANNEL);
  //Initialize RCInput
  rin.initialize();
  //Initialize RCOutputr
  rout.initialize(RECV_N_CHANNEL);
  
}

void loop() {
  //Update Timer
  watch.updateTime();
  //Update RC Signals
  //rc.read();

  //DEBUGGING
  rin.readRCstate();
  //Copy rin.rx_array to rout.pwm_array
  for (int idx = 0;idx<RECV_N_CHANNEL;idx++) {
    rout.pwm_array[idx] = rin.rx_array[idx];
  }
  //Send signals to PWM channels
  rout.write();
  
  //Poll GPS
  satellites.poll(watch.currentTime);
  
  //Print Everything
  Serial.print("T = ");
  Serial.print(watch.currentTime);
  Serial.print(" RX = ");
  rin.printRCstate(-6);
  //rc.in.printRCstate(-5);
  Serial.print(" PWM = ");
  rout.print();
  //rc.out.print();
  Serial.print(" LLH = ");
  satellites.printLLH();
  Serial.print("\n");

  //Log Everything
  logger.printvar(watch.currentTime);
  logger.printvar(satellites.latitude);
  logger.printvar(satellites.longitude);
  logger.printvar(satellites.altitude);
  logger.printarray(rin.rx_array,RECV_N_CHANNEL);
  logger.printarrayln(rout.pwm_array,RECV_N_CHANNEL);
  
  cross_sleep(0.1);
}
