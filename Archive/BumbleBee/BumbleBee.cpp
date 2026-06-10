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

///MAIN LOOP RATES
#define RXPERIOD 0.1
#define LOGPERIOD 0.02
#define DEBUGPERIOD 0.05

//DEBUG FLAGS
#define DEBUGPRINTS
//#define USEUSB
#define LOGDATA

///////////////All Classes////////////////////////////

//Datalogger
#include "links/Datalogger.h"
Datalogger logger;
MATLAB OUTPUT;
int FILEOPEN = 0;

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

//Autopilot
#include "Autopilot.h"
Autopilot control;

//Namespaces
using namespace std;
using namespace Navio;

///////////////////////////////////////////////////////

//Timing Functions 
static unsigned long currentTime = 0;
static unsigned long lastReceiverTime = 0; //REVISIT - I think Dr. C wanted to move this to the receiver class rcin? Should ask

static unsigned long getCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  static unsigned long currentTime;
  currentTime = 1000000 * tv.tv_sec + tv.tv_usec;
  return currentTime;
}

//=====================================Main Loop Start====================================
int main(int argc, char* argv[]) {

  printf("Alright so this code will output the number of input arguments \n");
  printf("and the input argument itself \n");
  printf("The number of input arguments = %d \n",argc); //Argc is an integer so it will tell us how many input args ther are
  printf("The input arguments are as follows: \n");

  //This loop will then tell us what the input arguments are
  for (int idx = 0;idx<argc;idx++){
    printf("Input Arguments %d = %s \n",idx,argv[idx]);
  }

  ///////////Check and see if you are a root user////////////
  if (getuid()) {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
  }
  ///////////////////////////////////////////////////////////
  
  ///////////Check for ardupilot running/////////////////////
  if (check_apm()) {
    return 1;
  }
  ///////////////////////////////////////////////////////////

  ///////////Data Matrix/////////////////////////////////////
  #ifdef LOGDATA
  //Create OUTPUT Matrix
  //time, throttleRC, rollRC, pitchRC, yawRC, roll,pitch,yaw,gx,gy,gz,9 motor signals
  OUTPUT.zeros(20,1,"OUTPUT MATRIX");
  #endif
  ///////////////////////////////////////////////////////////

  //Get Current Time
  time_t now = time(0);
  char* time_msg = ctime(&now);
  tm *gmtm = gmtime(&now);
  time_msg = asctime(gmtm);
  
  ///////////LED Setup///////////////////////////////////////
  printf("Creating LED Class \n");
  led = new Led_Navio2();
  led->initialize(); //Initialize the LED
  printf("LED Class Created. Setting Color to Yellow \n");
  led->setColor(Colors::Yellow);
  printf("LED is now yellow \n");
  ///////////////////////////////////////////////////////////
    
  ///////////Setup RX////////////////////////////////////////
  rcin.NUMRX = 8;
  rcin.RXCHANNELS[0] = 0; //roll is channel 0
  rcin.RXCHANNELS[1] = 1; //pitch is channel 1
  rcin.RXCHANNELS[2] = 2; //throttle in channel 2
  rcin.RXCHANNELS[3] = 3; //yaw is channel 3
  rcin.RXCHANNELS[4] = 6; //arm_switch is channel 6
  rcin.RXCHANNELS[5] = 5; //pusher pwm
  rcin.RXCHANNELS[6] = 4; //configuration switch
  //Get the pointer for writing to servos
  //Once again we're using the Navio2 so we can get rid of the get_rcout function
  ///////////////////////////////////////////////////////////

  ///////////ESC Setup///////////////////////////////////////
  //I moved everything to a pwmSetup routine -- WHERE??
  //#define PWM_OUTPUT5 4 -- These have been moved to the rcout initialize routine
  //#define PWM_OUTPUT6 5 
  //#define PWM_OUTPUT7 6
  //#define PWM_OUTPUT8 7
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
  printf("Servo Setup Complete \n");
  sleep(1);
  ///////////////////////////////////////////////////////////

  ///////////Get Time////////////////////////////////////////
  printf("Getting Time \n");
  currentTime = getCurrentTime();
  static unsigned long previousTime = 0;
  float elapsedTime = 0;
  ///This is the equivalent of a void loop() routine on Arduino
  static unsigned long beginTime = getCurrentTime();
  lastReceiverTime = beginTime;
  float printTime = beginTime;
  float dataTime = beginTime;
  float timeSinceStart = 0;
  printf("Got Time \n");
  ///////////////////////////////////////////////////////////

  ///////////Notify User/////////////////////////////////////
  printf("Time For take off \n"); //Oh shit.
  printf("This shits about to get #nasty #nohomo #shemakemedoit #Ihopethisfuckingworks \n");
  printf("But first....wait 10 seconds for the red light \n");
  printf("JK we got rid of this one. Hit the arm switch and kiss that ass goodbye \n");
  ///////////////////////////////////////////////////////////

  /////////////INFINITE WHILE LOOP//////////////

  while (1) {

    /////////////Get Current Time////////////////////////////
    //printf("Time...%lf \n",timeSinceStart);
    previousTime = currentTime;
    currentTime = getCurrentTime();
    elapsedTime = (currentTime - previousTime) / 1000000.0; 
    timeSinceStart = (currentTime - beginTime) / 1000000.0;
    /////////////////////////////////////////////////////////

    /////////////Poll Receiver///////////////////////////////
    if ((currentTime - lastReceiverTime)/1000000.0 > RXPERIOD) {
      lastReceiverTime = currentTime;
      //printf("Polling Receiver \n");
      rcin.pollReceiver();
      //printf("Receiver Polled \n");
      rollrc = rcin.RXVALS[0];
      pitchrc = rcin.RXVALS[1];
      throttle = rcin.RXVALS[2];
      yawrc = rcin.RXVALS[3];
      arm_switch = rcin.RXVALS[4];
      pusherpwm = rcin.RXVALS[5];
      configrc = rcin.RXVALS[6];
      //printf("Values extracted \n");
    }
    ////////////////////////////////////////////////////////////

    ////////////////////CONTROL LOOP HANDLES IMU//////
    #ifdef PRINTSEVERYWHERE
    printf("Running Controller Update \n");
    #endif
    control.update(currentTime,elapsedTime,FILEOPEN);
    #ifdef PRINTSEVERYWHERE
    printf("Controller Updated \n");
    #endif

    ////////////////////////////////////////////////////////////
    //================write outputs===============
    if (arm_switch > 1300) {
      led->setColor(Colors::Green);
      //================Controller==============
      //This runs every timestep since imuLoop is running everytimestep
      control.loop(rollrc,pitchrc,yawrc,throttle,configrc,pusherpwm,elapsedTime); 

      /////////////////////////////////////////
      #ifdef LOGDATA
      if (FILEOPEN == 0) {
	//Mount USB stick so we can plot easier in the field
	#ifdef USEUSB
	printf("Mounting USB!");
	system("sudo mount /dev/sda1 /media/mntpt"); //Make sure to create this on RPi
	#endif
	//The file is not open so open it
	logger.findfile(argv[1]);
	logger.open();
	logger.printchar(time_msg);
	FILEOPEN = 1; //Set this flag to 1 to signify that the file is open
      }
      #endif LOGDATA
    } else {
      //The arm switch is down
      led->setColor(Colors::Red);

      //Reset the controller
      control.reset();

      #ifdef LOGDATA
      //Check and see if we need to close the file
      if (FILEOPEN == 1) {
	FILEOPEN = 0;
	logger.close();
	
        #ifdef USEUSB
	printf("Unmounting USB! \n");
	system("sudo umount /dev/sda1");
        #endif
      }
      #endif //LOGDATA
    }

    ///////////Send signal to ESC//////////////////////////////////
    for (int i = 0;i<=8;i++) {
      rcout.set_duty_cycle(i,control.MOTORSOUT[i]);
    }
    //printf("pusher out = %lf \n ",control.pusherout);
    rcout.set_duty_cycle(8,control.pusherout);
    ///////////////////////////////////////////////////////////////

    ///////////LOGDATA/////////////////////////////////////////////
    #ifdef LOGDATA
    if ((((currentTime - dataTime)/1000000.0) > LOGPERIOD) && (FILEOPEN == 1)) {
      dataTime = currentTime;
      OUTPUT.set(1,1,currentTime);
      OUTPUT.set(2,1,throttle);
      OUTPUT.set(3,1,rollrc);
      OUTPUT.set(4,1,pitchrc);
      OUTPUT.set(5,1,yawrc);
      OUTPUT.set(6,1,control.orientation.roll);
      OUTPUT.set(7,1,control.orientation.pitch);
      OUTPUT.set(8,1,control.orientation.yaw);
      OUTPUT.set(9,1,control.orientation.roll_rate);
      OUTPUT.set(10,1,control.orientation.pitch_rate);
      OUTPUT.set(11,1,control.orientation.yaw_rate);
      for (int i = 12;i<20;i++) {
	OUTPUT.set(i,1,control.MOTORSOUT[i-12]);
      }
      OUTPUT.set(20,1,control.pusherout);
      logger.println(OUTPUT);
      //printf("Data logged \n");
    }
    #endif
    ///////////////////////////////////////////////////////////////

    ///////////DEGBUG//////////////////////////////////////////////
    #ifdef DEBUGPRINTS
    if ((currentTime - printTime) / 1000000.0 > DEBUGPERIOD) { //print at 20 Hz
      printTime = currentTime;
      //Time
      printf("%lf %lf ",printTime/1000000.0,elapsedTime);
      //PTP
      printf("%+05.2f %+05.2f %+05.2f ",control.orientation.roll,control.orientation.pitch,control.orientation.yaw);
      //PQR from IMU
      printf("%+05.2f %+05.2f %+05.2f ",control.orientation.roll_rate,control.orientation.pitch_rate,control.orientation.yaw_rate);
      //Rec Signals
      printf("%d %d %d %d %d %d %d ",throttle,rollrc,pitchrc,yawrc,configrc,arm_switch,pusherpwm);
      //Rec Commands
      //printf("%lf ",control.thrust_desired);
      //PWM outputs
      for (int i = 0;i<=7;i++) {
	printf("%lf ",control.MOTORSOUT[i]);
      }
      printf(" %lf ",control.pusherout);
      //Newline
      printf("\n");
    }
    #endif
    ///////////////////////////////////////////////////////////////
  } //end while

  return 0;

}
