//Import standard IO headers
#include <stdio.h>
#include <time.h>
using namespace std;

//Import telemetry class
#include <Serial/Telemetry.h>
Telemetry serial;

int main(int argc, char** argv) {

  printf("Testing Telemetry \n");

  //Import Input Arguments
  float var1,var2;
  if (argc > 2) {
    var1 = atof(argv[1]);
    var2 = atof(argv[2]);
  } else {
    printf("Not enough input arguments. Using random numbers \n");
    srand(time(NULL));
    var1 = rand() % 100 + 1;
    var2 = rand() % 100 + 1;
  }

  printf("Opening Serial Port ttyAMA0 \n");
  serial.SerialInit("/dev/ttyAMA0",57600);
  printf("If no errors present, serial port is open \n");
  
  //serial.SerialPutHello(); //This will send w\r

  //This routine below will send 7 floats in the H:XXXXXXXX\r format
  //with a \n sent at the end of all the numbers
  float number_array[2];
  number_array[0] = var1;
  number_array[1] = var2;
  serial.SerialSendArray(number_array,2);
  return 0;
}
