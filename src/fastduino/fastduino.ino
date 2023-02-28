//These #defines would normally go in the makefile
//but I don't want to learn Makefiles so they will go in here 
//instead

//Version 1.0 of the Arduino Version just has the ARDUINO and AUTO flags
//To see if it compiles
#define ARDUINO
#define AUTO

//Version 2.0 - Adding the timer
//#include <timer.h>

void setup() {
  //Setup Serial Std Out
  Serial.begin(115200);
  Serial.print("FASTCASST Software Version 42.0 \n");

  String root_folder_name = "vehicles/airplane/";

}

void loop() {
  // put your main code here, to run repeatedly:

}
