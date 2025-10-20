#include <unistd.h>
#include <cstdio>

#include<Navio2/RCInput_Navio2.h>
#include<Common/Util.h>
#include<memory>
#include "Navio2/PWM.h"
#include "Navio+/RCOutput_Navio.h"
#include "Navio2/RCOutput_Navio2.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <memory>

#define SERVO_START 1100 /*mS*/ //initialize motor pwm
#define SERVO_MAX 1750 /*mS*/
#define SERVO_MIN 1100 /*ms*/
#define PWM_OUTPUT1 0
#define PWM_OUTPUT2 1 
#define PWM_OUTPUT3 2
#define PWM_OUTPUT4 3
#define PWM_OUTPUT5 4
#define PWM_OUTPUT6 5  
#define PWM_OUTPUT7 6
#define PWM_OUTPUT8 7
#define IDLE 1200 /*mS*/
#define READ_FAILED -1
using namespace std;
using namespace Navio;

std::unique_ptr <RCInput> get_rcin()
{

  auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
  printf("Hey you are using a NAVIO2 TWO!!!!!!!!!\n");
  return ptr;


}

std::unique_ptr <RCOutput> get_rcout()
{
    if (get_navio_version() == NAVIO2)
    {
        auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
        return ptr;
    } else
    {
        auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio() };
        return ptr;
    }

}


int main(int argc, char *argv[])
{
  //RCInput rcin{}; //what? Why are you declaring this variable twice??? Seems like we need to chat....
    
    if (check_apm()) {
        return 1;
    }
        auto pwm = get_rcout();

        if (check_apm()) {
            return 1;
        }

        if (getuid()) {
            fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
        }


        if( !(pwm->initialize(PWM_OUTPUT1)) ) {
            return 1;
        }

        if( !(pwm->initialize(PWM_OUTPUT2)) ) {
            return 1;
        }

	if( !(pwm->initialize(PWM_OUTPUT3)) ) {
            return 1;
        }

        if( !(pwm->initialize(PWM_OUTPUT4)) ) {
            return 1;
        }

	pwm->set_frequency(PWM_OUTPUT1, 50);

	if ( !(pwm->enable(PWM_OUTPUT1)) ) {
	    return 1;
	}

	pwm->set_frequency(PWM_OUTPUT2, 50);

	if ( !(pwm->enable(PWM_OUTPUT2)) ) {
	    return 1;
	}

	pwm->set_frequency(PWM_OUTPUT3, 50);

	if ( !(pwm->enable(PWM_OUTPUT3)) ) {
	    return 1;
	}

	pwm->set_frequency(PWM_OUTPUT4, 50);

	if ( !(pwm->enable(PWM_OUTPUT4)) ) {
	    return 1;
	}
        if( !(pwm->initialize(PWM_OUTPUT5)) ) {
            return 1;
        }

        if( !(pwm->initialize(PWM_OUTPUT6)) ) {
            return 1;
        }

	if( !(pwm->initialize(PWM_OUTPUT7)) ) {
            return 1;
        }

        if( !(pwm->initialize(PWM_OUTPUT8)) ) {
            return 1;
        }

	pwm->set_frequency(PWM_OUTPUT5, 50);

	if ( !(pwm->enable(PWM_OUTPUT5)) ) {
	    return 1;
	}

	pwm->set_frequency(PWM_OUTPUT6, 50);

	if ( !(pwm->enable(PWM_OUTPUT6)) ) {
	    return 1;
	}

	pwm->set_frequency(PWM_OUTPUT7, 50);

	if ( !(pwm->enable(PWM_OUTPUT7)) ) {
	    return 1;
	}

	pwm->set_frequency(PWM_OUTPUT8, 50);

	if ( !(pwm->enable(PWM_OUTPUT8)) ) {
	    return 1;
	}

    auto rcin = get_rcin(); //See this code is using something super fucking fancy where instead of saying the variable is a double or of class RCInput they are telling the compiler that this is an auto variable. So
    //the compiler with "auto"matically figure out what type of variable this is.

    //This initialize routine will loop through NUM_CHANNELS so make sure to change NUM_CHANNELS
    //in RCINput_Navio2.h
    rcin->initialize();

    //How many channels do you want?
    //This #define NUM_CHANNELS is created in RCInput_Navio2.h
    rcin->channel_count = NUM_CHANNELS;
    printf("There are %d channels \n",rcin->channel_count);

    	//start the real shit 
    pwm->set_duty_cycle(PWM_OUTPUT1, SERVO_START); //this allows the motor to get a second to initialize before reading signals
    pwm->set_duty_cycle(PWM_OUTPUT2, SERVO_START);
    pwm->set_duty_cycle(PWM_OUTPUT3, SERVO_START);
    pwm->set_duty_cycle(PWM_OUTPUT4, SERVO_START);
    pwm->set_duty_cycle(PWM_OUTPUT5, SERVO_START); //this allows the motor to get a second to initialize before reading signals
    pwm->set_duty_cycle(PWM_OUTPUT6, SERVO_START);
    pwm->set_duty_cycle(PWM_OUTPUT7, SERVO_START);
    pwm->set_duty_cycle(PWM_OUTPUT8, SERVO_START);
    sleep(1); //hold that pwm signal for 5 seconds
    int arm_switch;
    int throttle;
    printf("Time For take off");
    sleep(1);
    while (true)
    {
      for(int i = 0; i < rcin->channel_count ; i++)
      {
      int period = rcin->read(i);
      //printf("%d ", period);
      throttle = rcin->read(2);
      arm_switch = rcin->read(6);
	//	printf("%d \n", arm_switch);
	
       }
      //      printf("\n"); 
      //sleep(0.1);
      //      printf("hey \n");
      if(arm_switch < 1300){
      	    pwm->set_duty_cycle(PWM_OUTPUT1, SERVO_MIN);
      	    pwm->set_duty_cycle(PWM_OUTPUT2, SERVO_MIN);
      	    pwm->set_duty_cycle(PWM_OUTPUT3, SERVO_MIN);
      	    pwm->set_duty_cycle(PWM_OUTPUT4, SERVO_MIN);
       	    pwm->set_duty_cycle(PWM_OUTPUT5, SERVO_MIN);
      	    pwm->set_duty_cycle(PWM_OUTPUT6, SERVO_MIN);
      	    pwm->set_duty_cycle(PWM_OUTPUT7, SERVO_MIN);
      	    pwm->set_duty_cycle(PWM_OUTPUT8, SERVO_MIN);

	    //	    printf("unarmed");
	    //	    sleep(2);
      }
      if(arm_switch > 1300){
	//	printf("Armed");
      	if(throttle < IDLE){
      	    pwm->set_duty_cycle(PWM_OUTPUT1, IDLE);
      	    pwm->set_duty_cycle(PWM_OUTPUT2, IDLE);
      	    pwm->set_duty_cycle(PWM_OUTPUT3, IDLE);
      	    pwm->set_duty_cycle(PWM_OUTPUT4, IDLE);
      	    pwm->set_duty_cycle(PWM_OUTPUT5, IDLE);
      	    pwm->set_duty_cycle(PWM_OUTPUT6, IDLE);
      	    pwm->set_duty_cycle(PWM_OUTPUT7, IDLE);
      	    pwm->set_duty_cycle(PWM_OUTPUT8, IDLE);

	    //    printf("IDILING BY");
	    //	    sleep(2);
	    
      	}
      	if(throttle > IDLE){
      	    pwm->set_duty_cycle(PWM_OUTPUT1, throttle);
      	    pwm->set_duty_cycle(PWM_OUTPUT2, throttle);
      	    pwm->set_duty_cycle(PWM_OUTPUT3, throttle);
      	    pwm->set_duty_cycle(PWM_OUTPUT4, throttle);
       	    pwm->set_duty_cycle(PWM_OUTPUT5, throttle);
      	    pwm->set_duty_cycle(PWM_OUTPUT6, throttle);
      	    pwm->set_duty_cycle(PWM_OUTPUT7, throttle);
      	    pwm->set_duty_cycle(PWM_OUTPUT8, throttle);

	    printf("we're rolling");
	    //	    sleep(2);
	    
	}
      }
    }

    return 0;
}
