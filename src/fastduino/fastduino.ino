//These #defines would normally go in the makefile
//but I don't want to learn Makefiles so they will go in here 
//instead
#define ARDUINO
#define AUTO

#include "timer.h"
TIMER watch;

void setup() {
  //Setup Serial Std Out
  Serial.begin(115200);
  Serial.print("FASTKit Software Version 42.0 \n");
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
