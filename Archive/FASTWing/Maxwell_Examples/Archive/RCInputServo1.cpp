//Combination of RCInput and Servo code for meta
#include <unistd.h> //Servo and RCInput
#include "Navio2/PWM.h" //Servo
#include "Navio2/RCOutput_Navio2.h" //Servo
#include <Common/Util.h> //Servo and RCInput
#include <stdio.h> //Servo
#include <stdlib.h> //Servo
#include <iostream> //Servo
#include <memory> //Servo
#include <stdint.h>
#include <sys/time.h>
#include <cstdio> //RCInput
#include <Navio2/RCInput_Navio2.h> //RCInput
#include <Navio2/Led_Navio2.h>

#define SERVO_START 1100 /*mS*/ //Initialize motor PWM
#define SERVO_MAX 1750 /*mS*/
#define SERVO_MIN 1100 /*mS*/
#define PWM_OUTPUT1 8
#define PWM_OUTPUT2 0
#define PWM_OUTPUT3 1
#define PWM_OUTPUT4 2

using namespace std;
using namespace Navio;

RCOutput_Navio2 rcout;

#define READ_FAILED -1

std::unique_ptr <RCInput> get_rcin()
{
  auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2()};
  printf("Hey you are using a NAVIO2 TWO!!!!!!!!!\n");
  return ptr;
}

int main(int argc, char *argv[])
{
  if (check_apm()){
    return 1;
      }
  auto rcin = get_rcin(); //See this code is using something super fucking fancy where instead of saying the variable is a double or of class RCInput they are telling the compiler that this is an auto variable. So the compiler with "aut"matically figure out what type of variable this is.

  //This initiliaze routine will loop through NUM_CHANNELS so make sure to change NUM_CHANNELS in RCInput_Navio2.h
  rcin->initialize();

  //How many channels do you want?
  //This #define NUM_CHANNELS is created in RCInput_Navio2.h
  rcin ->channel_count = NUM_CHANNELS;
  printf("There are %d channels!!!! \n",rcin->channel_count);

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
  rcout.set_duty_cycle(0, SERVO_START);

  sleep(1);
  int period = SERVO_MIN;
  while (1) {
    //for (int i = 0; i < rcin->channel_count ; i++)
    //    {
    period = rcin->read(0);
    printf("%d ", period);
    //}
    //printf("\n");
	rcout.set_duty_cycle(0,period);
  }
  return 0;
}
