//These #defines would normally go in the makefile
//but I don't want to learn Makefiles so they will go in here 
//instead
#define ARDUINO
#define AUTO

//Furthermore, you can't import text files on an Arduino so everything
//in Simulation.txt and Config.txt must be placed in here

//Note when setting this up for the first time. 
//Install the Due board by going to the board manager
//Go to preferences and change the location of the libraries to ~/FASTCASST

//Timer.h for realtime clock
#include "timer.h"
TIMER watch;

//Include the hardware environment
//#include "hardware.h"

///DEBUG HEADER FILES
#include "mathp.h"
//Need MATLAB.h for Matrices
#include "MATLAB.h"
//RCIO is inside hardware.h
//#include "RCIO.h"
//RCInput is inside RCIO.h
#include "RCInput.h"
RCInput rin;

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
  rin.initialize();
  
}

void loop() {
  //Update Timer
  watch.updateTime();
  //Update RC Signals
  rin.readRCstate();

  //Print Everything
  Serial.print("T = ");
  Serial.print(watch.currentTime);
  Serial.print("RX = ");
  rin.printRCstate(-5);
  Serial.print("\n");
  cross_sleep(0.1);
}
