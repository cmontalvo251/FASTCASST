///The Main source files have no header files
//Everything is contained in here

#include <stdio.h>
using namespace std;

//Timer 
#include <Timer/timer.h>
TIMER watch;
#define PRINTRATE 0.1 //Rate of printing to stdout
double lastPRINTtime = 0;

//Need Sensors class to read all sensors
#include <hardware/sensors/sensors.h>
sensors sense; 

int main(int argc,char* argv[]) {
  printf("FASTKit Logger \n");

  //Enter into infinite while loop
  while (1) {

    //Get Current Time
    double currentTime = watch.getTimeSinceStart();

    //Logger Simply uses the sensor class to poll everything
    sense.poll(currentTime);

    //PRINT TO STDOUT
    if (lastPRINTtime < currentTime) {
      lastPRINTtime+=PRINTRATE;
      //Time
      printf("%lf ",currentTime);
      //Poll all sensors
      sense.print();
      //Newline
      printf("\n");
    }
  }
}
