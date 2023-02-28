#include "RCOutput.h"

//constructor class
RCOutput::RCOutput() {
}

//Init routine
void RCOutput::initialize(int num) {
	NUMSIGNALS = num;
	pwm_array = (int *) calloc(NUMSIGNALS,sizeof(int));
	pwm_array_prev = (int *) calloc(NUMSIGNALS,sizeof(int));
	saturation_block(); //Set the pwm_array to minimum vals (no need to do this on pwm_array_prev)

	////////I REALIZE THERE ARE 3 LOOPS AND YOU COULD DO THIS WITH
	// 1 loop. BUT IT WILL NOT WORK. JUST TRUST ME. Dr. C 9/27/2021
	
	//Initialize channels
	for (int i = 0;i<NUMSIGNALS;i++) { 
	  printf("Initializing PWM Output = %d \n",i);
	  #if defined AUTO || HIL || ARDUINO
	  #if defined RPI || ARDUINO
	  if(!(pwmsignals.init(i))) {
	    exit(1);
	  }
	  #endif
	  #endif
	}

	#if defined AUTO || HIL
	#ifdef RPI
	//INITIALIZE FREQUENCY
	for (int i = 0;i<NUMSIGNALS;i++) { 
	  printf("Setting Frequency of %d Servo \n",i);
	  pwmsignals.set_period(i, 50);
	  if (!(pwm.enable(i))) {
	    exit(1);
	  }
	}
	//SET INITIAL DUTY CYCLE
	for (int i = 0;i<NUMSIGNALS;i++) { 
	  printf("Running Servo Start of %d servo \n",i);
	  pwmsignals.set_duty_cycle(i,OUTMIN/1000);
	}
	#endif
	#endif
}

//Write function
void RCOutput::write() {
  //Make sure we don't send ridiculous commands
  saturation_block();
  //Write the pwm signals in comms but only loop until comms.getRow()
  for (int i = 0;i<NUMSIGNALS;i++) {
    float us = pwm_array[i];
    //printf("Sending PWM signal to channel = %d \n",i);
    #if defined AUTO || HIL || ARDUINO
    #if defined RPI || ARDUINO
    pwmsignals.set_duty_cycle(i,us/1000.0);
    #endif
    #endif
  } 
}

void RCOutput::backup() {
  for (int i = 0;i<NUMSIGNALS;i++) {
    pwm_array_prev[i] = pwm_array[i];
  }
}

void RCOutput::revert() {
  //printf("Reverting \n");
  for (int i = 0;i<NUMSIGNALS;i++) {
    pwm_array[i] = pwm_array_prev[i];
  }	
}

int RCOutput::RangeCheck() {
	int badsignals = 0; //This is used in the event the controller is spitting out
	//such horribly bogus data that you probably just want to set everything to neutral 
	//I'm a little concerned because if this happens in flight it could be terrible
	//but right now this only runs in HIL mode
	for (int idx = 0;idx<NUMSIGNALS;idx++){
		float val = pwm_array[idx];
		if ((val <= 0) || (val >= 32168)) { 
			badsignals = 1;
		}
	}
	//printf("Running alg \n");
	if (badsignals == 1) {
		//printf("Setting to Neutral \n");
		setOutputNeutral();
	}
	//Then we run a saturation check
	saturation_block();

	return badsignals;
}

void RCOutput::setOutputNeutral() {
	for (int idx = 0;idx<NUMSIGNALS;idx++) {
		pwm_array[idx] = OUTMIN;
	}
	#ifdef airplane
	pwm_array[1] = OUTMID;
	pwm_array[2] = OUTMID;
	pwm_array[3] = OUTMID;
	#endif
}

void RCOutput::saturation_block() {
  for (int idx=0;idx<NUMSIGNALS;idx++) {
    float val = pwm_array[idx];
    if (val > OUTMAX) {
    	pwm_array[idx] = OUTMAX;
    }
    if (val < OUTMIN) {
    	pwm_array[idx] = OUTMIN;
    }
  }
}

void RCOutput::print() {
  for (int idx = 0;idx<NUMSIGNALS;idx++) {
    printf("%d ",pwm_array[idx]);
  }
}
