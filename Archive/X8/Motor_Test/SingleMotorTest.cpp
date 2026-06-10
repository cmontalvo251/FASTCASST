//This script will send a PWM signal to the motor to test and make sure it works
#include <unistd.h>
#include <cstdio>
#include <Common/Util.h>
#include "Navio2/PWM.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <memory>
#include <stdint.h>
#include <sys/time.h>
#include <ctime>

///////////////All Classes////////////////////////////
//Led
#include <Navio2/Led_Navio2.h>
Led_Navio2 *led;

//RC Output
#include "Navio2/RCOutput_Navio2.h"
RCOutput_Navio2 rcout;

//RC Input
#include <Navio2/RCInput_Navio2.h>
RCInput_Navio2 rcin;
//Receiver Signals
int arm_switch;
int throttle;
int rollrc;
int pitchrc;
int yawrc;
int pusherpwm;
int configrc;

double MOTORSOUT_1 = 992;
double MOTORSOUT_2 = 1504;
double MOTORSOUT_3 = 2016;
float Counter = 0.0;

//Namespaces
using namespace std;
using namespace Navio;

///////////////////////////////////////////////////////

///////////////Main Loop///////////////////////////////
int main(int argc, char* argv[]) {
  ///////////ESC Setup///////////////////////////////////////
  rcout.num_motors = 9;
  rcout.PWMOUTPUTS[0] = 0;
  rcout.PWMOUTPUTS[1] = 1;
  rcout.PWMOUTPUTS[2] = 2;
  rcout.PWMOUTPUTS[3] = 3;
  rcout.PWMOUTPUTS[4] = 4;
  rcout.PWMOUTPUTS[5] = 5;
  rcout.PWMOUTPUTS[6] = 6;
  rcout.PWMOUTPUTS[7] = 7;
  rcout.PWMOUTPUTS[8] = 8; 
  
  int out_res = rcout.rcoutSetup();
  if (out_res) {
    printf("RCOUT Error \n");
    return 0;
  }
  ///////////////////////////////////////////////////////////

  ///////////INFINITE WHILE LOOP/////////////////////////////
  while (1) {

    Counter = Counter + 1;
    
    ///////////Send signal to ESC////////////////////////////
    //Send idle to all motors:
    if(Counter/1000 < 5){
      for (int i = 0;i<=8;i++) {
	rcout.set_duty_cycle(i,MOTORSOUT_1);
      }
    }
    //Motor 1:
    if(Counter/1000 > 5){
      rcout.set_duty_cycle(0,MOTORSOUT_2);
      rcout.set_duty_cycle(1,MOTORSOUT_1);
      rcout.set_duty_cycle(2,MOTORSOUT_1);
      rcout.set_duty_cycle(3,MOTORSOUT_1);
      rcout.set_duty_cycle(4,MOTORSOUT_1);
      rcout.set_duty_cycle(5,MOTORSOUT_1);
      rcout.set_duty_cycle(6,MOTORSOUT_1);
      rcout.set_duty_cycle(7,MOTORSOUT_1);
      rcout.set_duty_cycle(8,MOTORSOUT_1);
    } else{
      for (int i = 0;i<=8;i++) {
	rcout.set_duty_cycle(i,MOTORSOUT_1);
      }
    }
    //Motor 2:
    if(Counter/1000 > 10){
      rcout.set_duty_cycle(0,MOTORSOUT_1);
      rcout.set_duty_cycle(1,MOTORSOUT_2);
      rcout.set_duty_cycle(2,MOTORSOUT_1);
      rcout.set_duty_cycle(3,MOTORSOUT_1);
      rcout.set_duty_cycle(4,MOTORSOUT_1);
      rcout.set_duty_cycle(5,MOTORSOUT_1);
      rcout.set_duty_cycle(6,MOTORSOUT_1);
      rcout.set_duty_cycle(7,MOTORSOUT_1);
      rcout.set_duty_cycle(8,MOTORSOUT_1);
    } else{
      for (int i = 0;i<=8;i++) {
	rcout.set_duty_cycle(i,MOTORSOUT_1);
      }
    }
    //Motor 3:
    if(Counter/1000 > 15){
      rcout.set_duty_cycle(0,MOTORSOUT_1);
      rcout.set_duty_cycle(1,MOTORSOUT_1);
      rcout.set_duty_cycle(2,MOTORSOUT_2);
      rcout.set_duty_cycle(3,MOTORSOUT_1);
      rcout.set_duty_cycle(4,MOTORSOUT_1);
      rcout.set_duty_cycle(5,MOTORSOUT_1);
      rcout.set_duty_cycle(6,MOTORSOUT_1);
      rcout.set_duty_cycle(7,MOTORSOUT_1);
      rcout.set_duty_cycle(8,MOTORSOUT_1);
    } else{
      for (int i = 0;i<=8;i++) {
	rcout.set_duty_cycle(i,MOTORSOUT_1);
      }
    }
    //Motor 4:
    if(Counter/1000 > 20){
      rcout.set_duty_cycle(0,MOTORSOUT_1);
      rcout.set_duty_cycle(1,MOTORSOUT_1);
      rcout.set_duty_cycle(2,MOTORSOUT_1);
      rcout.set_duty_cycle(3,MOTORSOUT_2);
      rcout.set_duty_cycle(4,MOTORSOUT_1);
      rcout.set_duty_cycle(5,MOTORSOUT_1);
      rcout.set_duty_cycle(6,MOTORSOUT_1);
      rcout.set_duty_cycle(7,MOTORSOUT_1);
      rcout.set_duty_cycle(8,MOTORSOUT_1);
    } else{
      for (int i = 0;i<=8;i++) {
	rcout.set_duty_cycle(i,MOTORSOUT_1);
      }
    }
    //Motor 5:
    if(Counter/1000 > 25){
      rcout.set_duty_cycle(0,MOTORSOUT_1);
      rcout.set_duty_cycle(1,MOTORSOUT_1);
      rcout.set_duty_cycle(2,MOTORSOUT_1);
      rcout.set_duty_cycle(3,MOTORSOUT_1);
      rcout.set_duty_cycle(4,MOTORSOUT_2);
      rcout.set_duty_cycle(5,MOTORSOUT_1);
      rcout.set_duty_cycle(6,MOTORSOUT_1);
      rcout.set_duty_cycle(7,MOTORSOUT_1);
      rcout.set_duty_cycle(8,MOTORSOUT_1);
    } else{
      for (int i = 0;i<=8;i++) {
	rcout.set_duty_cycle(i,MOTORSOUT_1);
      }
    }
    //Motor 6:
    if(Counter/1000 > 30){
      rcout.set_duty_cycle(0,MOTORSOUT_1);
      rcout.set_duty_cycle(1,MOTORSOUT_1);
      rcout.set_duty_cycle(2,MOTORSOUT_1);
      rcout.set_duty_cycle(3,MOTORSOUT_1);
      rcout.set_duty_cycle(4,MOTORSOUT_1);
      rcout.set_duty_cycle(5,MOTORSOUT_2);
      rcout.set_duty_cycle(6,MOTORSOUT_1);
      rcout.set_duty_cycle(7,MOTORSOUT_1);
      rcout.set_duty_cycle(8,MOTORSOUT_1);
    } else{
      for (int i = 0;i<=8;i++) {
	rcout.set_duty_cycle(i,MOTORSOUT_1);
      }
    }
    //Motor 7:
    if(Counter/1000 > 35){
      rcout.set_duty_cycle(0,MOTORSOUT_1);
      rcout.set_duty_cycle(1,MOTORSOUT_1);
      rcout.set_duty_cycle(2,MOTORSOUT_1);
      rcout.set_duty_cycle(3,MOTORSOUT_1);
      rcout.set_duty_cycle(4,MOTORSOUT_1);
      rcout.set_duty_cycle(5,MOTORSOUT_1);
      rcout.set_duty_cycle(6,MOTORSOUT_2);
      rcout.set_duty_cycle(7,MOTORSOUT_1);
      rcout.set_duty_cycle(8,MOTORSOUT_1);
    } else{
      for (int i = 0;i<=8;i++) {
	rcout.set_duty_cycle(i,MOTORSOUT_1);
      }
    }
    //Motor 8:
    if(Counter/1000 > 40){
      rcout.set_duty_cycle(0,MOTORSOUT_1);
      rcout.set_duty_cycle(1,MOTORSOUT_1);
      rcout.set_duty_cycle(2,MOTORSOUT_1);
      rcout.set_duty_cycle(3,MOTORSOUT_1);
      rcout.set_duty_cycle(4,MOTORSOUT_1);
      rcout.set_duty_cycle(5,MOTORSOUT_1);
      rcout.set_duty_cycle(6,MOTORSOUT_1);
      rcout.set_duty_cycle(7,MOTORSOUT_2);
      rcout.set_duty_cycle(8,MOTORSOUT_1);
    } else{
      for (int i = 0;i<=8;i++) {
	rcout.set_duty_cycle(i,MOTORSOUT_1);
      }
    }
    //Motor 9:
    if(Counter/1000 > 45){
      rcout.set_duty_cycle(0,MOTORSOUT_1);
      rcout.set_duty_cycle(1,MOTORSOUT_1);
      rcout.set_duty_cycle(2,MOTORSOUT_1);
      rcout.set_duty_cycle(3,MOTORSOUT_1);
      rcout.set_duty_cycle(4,MOTORSOUT_1);
      rcout.set_duty_cycle(5,MOTORSOUT_1);
      rcout.set_duty_cycle(6,MOTORSOUT_1);
      rcout.set_duty_cycle(7,MOTORSOUT_1);
      rcout.set_duty_cycle(8,MOTORSOUT_2);
    } else{
      for (int i = 0;i<=8;i++) {
	rcout.set_duty_cycle(i,MOTORSOUT_1);
      }
    }
    /////////////////////////////////////////////////////////

    printf("%lf \n",Counter/1000);
    
  } //end while
  return 0;
}
