///The Main source files have no header files
//Everything is contained in here
#include <stdio.h>
using namespace std;

//Main loop
void loop();

//Timer 
#include <Timer/timer.h>
TIMER watch;
#define PRINTRATE 0.1 //Rate of printing to stdout
double lastPRINTtime = 0;

//The Hardware environment is always running
#include <hardware/src/hardware.h>
hardware hw;

//If you're simulating the vehicle you have to turn on the 
//modeling environment
#if defined (SIL) || (SIMONLY) || (HIL)
#if defined (DESKTOP)
#define MODELING
#include <modeling/modeling.h>
modeling model;
#endif
#endif


int main(int argc,char* argv[]) {
  printf("FASTKit Software version 42.0 \n");

  ////The main routine needs to grab the root file name//////
  char root_folder_name[256];
  if (argc > 1) {
    sprintf(root_folder_name,"%s","vehicles/");
    strcat(root_folder_name,argv[1]);
  } else {
    printf("Using Default Root Folder Name\n");
    sprintf(root_folder_name,"%s","vehicles/portalcube/");
  }
  //strcat(root_folder_name,"/");
  //printf("Root Folder Name = %s \n",root_folder_name);
  //////////////////////////////////////////////////////////

  //The Hardware block needs the root filename to run it's
  //initialization routine
  hw.init(root_folder_name);

  //Begin main loop but run as a separate function in anticipation of threading
  loop();
}

//Main Loop
void loop() {
  //Enter into infinite while loop
  while (1) {

    //Get Current Time and elapsed Time
    double currentTime = watch.getTimeSinceStart();
    double elapsedTime = watch.getTimeElapsed();

    //PRINT TO STDOUT
    if (lastPRINTtime < currentTime) {
      lastPRINTtime+=PRINTRATE;
      //Time
      printf("%lf ",currentTime);
      //Newline
      printf("\n");
    }
  }

}
