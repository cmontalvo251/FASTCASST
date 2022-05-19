#include "RCOutput_Navio2.h"

RCOutput_Navio2::RCOutput_Navio2()
{
}

int RCOutput_Navio2::rcoutSetup() {
  //rcout = new RCOutput_Navio2();
  for (int idx = 0;idx<num_motors;idx++) {
    //////////////////INTIALIZE ALL THE MOTORS///////////////////
    printf("Initializing %d motor \n",idx);
    if( !(initialize(PWMOUTPUTS[idx])) ) {
      return 1;
    }
  }
  for (int idx = 0;idx<num_motors;idx++) {
    //Set the frequency and enable each pin
    printf("Setting Frequency of %d motor \n",idx);
    set_frequency(PWMOUTPUTS[idx], 50);
    if ( !(enable(PWMOUTPUTS[idx])) ) {
      return 1;
    }
  }
   for (int idx = 0;idx<num_motors;idx++) {
    //start the real shit -- this actually starts writing to servos (motors)
    //this allows the motor to get a second to initialize before reading signals
     printf("Running Servo Start of %d motor \n",idx);
     set_duty_cycle(PWMOUTPUTS[idx], SERVO_START);
  }
  //sleep(1); //hold that pwm signal for 5 seconds - 5 seconds? Or just 1? <-this had to be moved to bumblebee.cpp
  return 0;
}


bool RCOutput_Navio2::initialize(int channel)
{
    return pwm.init(channel);
}

bool RCOutput_Navio2::enable(int channel)
{
    return pwm.enable(channel);
}

bool RCOutput_Navio2::set_frequency(int channel, float frequency)
{
    return pwm.set_period(channel, frequency);
}

bool RCOutput_Navio2::set_duty_cycle(int idx, float period)
{
  int channel = PWMOUTPUTS[idx];
  return pwm.set_duty_cycle(channel, period / 1000);
}
