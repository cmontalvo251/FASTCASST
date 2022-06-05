#include "RCOutput.h"

//constructor class
RCOutput::RCOutput() {
}

//Init routine
void RCOutput::initialize(int num) {
	NUMSIGNALS = num;
	pwm_array = (int *) calloc(NUMSIGNALS,sizeof(int));
	saturation_block(); //Set the pwm_array to minimum vals

	////////I REALIZE THERE ARE 3 LOOPS AND YOU COULD DO THIS WITH
	// 1 loop. BUT IT WILL NOT WORK. JUST TRUST ME. Dr. C 9/27/2021
	
	//Initialize channels
	for (int i = 0;i<NUMSIGNALS;i++) { 
	  printf("Initializing PWM Output = %d \n",i);
	  #ifdef AUTO
	  if(!(pwm.init(i))) {
	    exit(1);
	  }
	  #endif
	}

	#ifdef AUTO
	//INITIALIZE FREQUENCY
	for (int i = 0;i<NUMSIGNALS;i++) { 
	  printf("Setting Frequency of %d Servo \n",i);
	  pwm.set_period(i, 50);
	  if (!(pwm.enable(i))) {
	    exit(1);
	  }
	}
	//SET INITIAL DUTY CYCLE
	for (int i = 0;i<NUMSIGNALS;i++) { 
	  printf("Running Servo Start of %d servo \n",i);
	  pwm.set_duty_cycle(i,OUTMIN/1000);
	}
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
    #ifdef AUTO
    pwm.set_duty_cycle(i,us/1000.0);
    #endif
  } 
}

void RCOutput::RangeCheck() {
	int badsignals = 0; //This is used in the event the controller is spitting out
	//such horribly bogus data that you probably just want to set everything to neutral 
	//I'm a little concerned because if this happens in flight it could be terrible
	//but right now this only runs in HIL mode
	for (int idx = 0;idx<NUMSIGNALS;idx++){
		float val = pwm_array[idx];
		if ((val < 0) || (val > 32168)) { 
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
