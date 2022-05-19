/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Control servos connected to PWM driver onboard of Navio2 shield for Raspberry Pi.

Connect servo to Navio2's rc output and watch it work.
PWM_OUTPUT = 0 complies to channel number 1, 1 to channel number 2 and so on.
To use full range of your servo correct SERVO_MIN and SERVO_MAX according to it's specification.

To run this example navigate to the directory containing it and run following commands:
make
sudo ./Servo
*/

#include <unistd.h>
#include "Navio2/PWM.h"
#include "Navio+/RCOutput_Navio.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Common/Util.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <memory>

#define SERVO_START 1100 /*mS*/ //initialize motor pwm
#define SERVO_MAX 1750 /*mS*/
#define SERVO_MIN 1100 /*ms*/
#define PWM_OUTPUT1 8
#define PWM_OUTPUT2 0 
#define PWM_OUTPUT3 1
#define PWM_OUTPUT4 2

using namespace std;
using namespace Navio;

RCOutput_Navio2 rcout;

int main(int argc, char *argv[])
{

  rcout.num_motors = 1; //num signals
  rcout.PWMOUTPUTS[0] = 0; //channel number

  int out_res = rcout.rcoutSetup();
  if (out_res) {
    return EXIT_FAILURE;
  }

  if (check_apm()) {
    return 1;
  }

  if (getuid()) {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
  }

  rcout.set_duty_cycle(0,SERVO_START);

  sleep(1);

  while (1) {
    printf("Setting SEROV_MIN \n");
    rcout.set_duty_cycle(0,SERVO_MIN);
    sleep(1);
    printf("Setting SEROV_MAX \n");
    rcout.set_duty_cycle(0,SERVO_MAX);
    sleep(1);
  }
  
  
  
  return 0;
}
