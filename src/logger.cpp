///The Main source files have no header files
//Everything is contained in here

#include <stdio.h>
using namespace std;

//Timer 
#include <Timer/timer.h>
TIMER watch;
#define PRINTRATE 0.1 //Rate of printing to stdout
double PRINT = 0;

int main(int argc,char* argv[]) {
  printf("FASTKit Logger \n");

  //Enter into infinite while loop
  while (1) {

    //Get Current Time
    double t = watch.getTimeSinceStart();

    //


    //

    //PRINT TO STDOUT
    if (PRINT < t) {
      PRINT+=PRINTRATE;
      printf("%lf ",t);
      printf("\n");
    }
  }
}
