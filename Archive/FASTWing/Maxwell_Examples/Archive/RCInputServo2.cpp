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

//#define SERVO_START 990 /*mS*/ //Initialize motor PWM //This happens in RCOutput_Navio2.h
#define SERVO_MAX 2100 /*mS*/ //Maximum signal read by the receiver is 2017 mS
#define SERVO_MIN 990 /*mS*/ //Minimum signal read by the receiver is 992 mS

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
  //rcin->initialize();
  rcin->NUMRX = 6;
  rcin->RXCHANNELS[0] = 0; //This is the roll.
  rcin->RXCHANNELS[1] = 1; //This is the pitch.
  rcin->RXCHANNELS[2] = 2; //This is the throttle.
  rcin->RXCHANNELS[3] = 3; //This is the yaw.
  rcin->RXCHANNELS[4] = 4; //This is the 3-way switch on the transmitter. I needed to set this up on the transmitter under 5/10.
  rcin->RXCHANNELS[5] = 5; //This is the gear switch (Top switch on the right back side). This is for the arm-disarm switch. I needed to set this up on the transmitter under 5/10.
  
  //How many channels do you want?
  //This #define NUM_CHANNELS is created in RCInput_Navio2.h
  //rcin ->channel_count = NUM_CHANNELS;
  //printf("There are %d channels!!!!! \n",rcin->channel_count);

  rcout.num_motors = 4; //num signals
  rcout.PWMOUTPUTS[0] = 0; //channel number
  rcout.PWMOUTPUTS[1] = 1;
  rcout.PWMOUTPUTS[2] = 2;
  rcout.PWMOUTPUTS[3] = 3;

  int out_res = rcout.rcoutSetup();
  if (out_res) {
    return EXIT_FAILURE;
  } else {
    printf("Servo Setup Complete \n");
  }
  if (check_apm()) {
    return 1;
  } else {
    printf("APM Not Running \n");
  }
  if (getuid()) {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
  }

  sleep(1);

  int period[4];
  while (1) {
    //printf("Reading Channels \n");
    rcin->pollReceiver();

    for (int i = 0;i < 6 ; i++)
      {
        printf("%lf ",rcin->RXVALS[i]);
      }

    //printf("Writing Servos \n");
    for (int i = 0;i < 4 ; i++)
      {
	rcout.set_duty_cycle(i, rcin->RXVALS[i]);
	//printf("Hey \n");
      }
    printf("\n");
  }
  return 0;
}
