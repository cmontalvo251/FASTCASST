

//These #defines would normally go in the makefile
//but I don't want to learn Makefiles so they will go in here 
//instead
#define ARDUINO
#define AUTO

#include <Rotation3.h>
#include <mathp.h>
#include <Datalogger.h>
#include <timer.h>

void setup() {
  //Setup Serial Std Out
  Serial.begin(115200);
  Serial.print("FASTKit Software Version 42.0 \n");

  String root_folder_name = "vehicles/airplane/";

}

void loop() {
  // put your main code here, to run repeatedly:

}
