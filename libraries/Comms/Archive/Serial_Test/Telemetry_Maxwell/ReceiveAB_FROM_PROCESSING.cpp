#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

//sudo apt-get install wiringpi
#include <wiringPi.h>
#include <wiringSerial.h>

//to compile run
//g++ -o run.exe Telemetry.cpp -lwiringPi

int main()
{
  int serial_port;
  char dat;

  if(wiringPiSetup() == -1)
    {
      fprintf(stdout, "Unable to start wiringPi: %s\n", strerror (errno));
      return 1;
    }
  
  if ((serial_port = serialOpen("/dev/ttyAMA0",57600)) < 0) /*open serial port*/
    {
      fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
      return 1;
    }
  
  while(1)
  {
    if(serialDataAvail(serial_port))
      {
	dat = serialGetchar(serial_port);
	printf("char = %c int(char) = %d \n", dat,int(dat));
	fflush(stdout);
      }
      //}
  }
}
