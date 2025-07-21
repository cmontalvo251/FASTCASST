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
#include <sensors/sensors.h>
sensors sense;

//Need datalogger to log data to disk
#include <Datalogger/Datalogger.h>
Datalogger logger;
#define LOGRATE 1.0
double lastLOGtime = 0;

int main(int argc,char* argv[]) {
  printf("FASTCASST Logger \n");

  //Initialize Logger
  logger.init("data/",sense.getNumVars());
  //Set and log headers
  logger.appendheaders(sense.headernames,sense.getNumVars());
  logger.printheaders();

  //Pick the IMU you want to use
  //0 = MPU9250
  //1 = LSM9DS1
  sense.initIMU(0);

  //Enter into infinite while loop
  while (1) {

    //Get Current Time and elapsed Time
    watch.updateTime();

    //Logger Simply uses the sensor class to poll everything
    sense.poll(watch.currentTime,watch.elapsedTime);

    //PRINT TO FILE
    if (lastLOGtime < watch.currentTime) {
      lastLOGtime+=LOGRATE;
      logger.append(sense.sense_matrix);
      logger.println();
    }

    //PRINT TO STDOUT
    if (lastPRINTtime < watch.currentTime) {
      lastPRINTtime+=PRINTRATE;
      //Time
      printf("%lf ",watch.currentTime);
      //Poll all sensors
      sense.print();
      //Newline
      printf("\n");
    }
  }
}
