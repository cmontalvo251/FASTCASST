//These #defines would normally go in the makefile
//but I don't want to learn Makefiles so they will go in here 
//instead
#define ARDUINO
#define AUTO

//Furthermore, you can't import text files on an Arduino so everything
//in Simulation.txt and Config.txt must be placed in here


//Timer.h for realtime clock
#include "timer.h"
TIMER watch;

//Include the hardware environment
//#include "hardware.h"

///DEBUG HEADER FILES
#include "mathp.h"
//Need MATLAB.h for Matrices
#include "MATLAB.h"


void setup() {
  //Setup Serial Std Out
  Serial.begin(115200);

  ///Print dummy version number
  Serial.print("FASTKit Software Version 42.0 \n");

  //Hardware init
  
  
  ///Initialize the timer
  Serial.print("Initiailizing Timer...\n");
  watch.init(0);
  Serial.print("Timer Initialized \n");
  
}

void loop() {
  //Update Timer
  watch.updateTime();
  Serial.print("T = ");
  Serial.print(watch.currentTime);
  cross_sleep(1.0);
}
