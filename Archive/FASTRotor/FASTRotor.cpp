#include <unistd.h>
#include "Navio2/PWM.h"
#include <Common/FASTPWM.h>
#include <Common/Util.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <memory>
#include <stdint.h>
#include <sys/time.h>
#include <cstdio>
#include <ctime>

#define RXPERIOD 0.5
#define LOGPERIOD 0.1
#define DEBUGPERIOD 0.1

//#define DEBUGPRINTS
//#define PRINTSEVERYWHERE
//#define ARMALLTHETIME
//#define USEUSB
//#define NOPANIC
//#define TELEMETRY
//#define LOGDATA

//ALL THE CLASSES
#include "Navio2/RCOutput_Navio2.h"
RCOutput_Navio2 rcout;

#include <Navio2/RCInput_Navio2.h>

RCInput_Navio2 rcin;

int throttle;
int rollrc;
int pitchrc;
int yawrc;
int arm_switch;
int extra_servo;
int arm_ok=0;

#include <Navio2/Led_Navio2.h>
Led_Navio2 *led;

#include "links/Datalogger.h"
Datalogger logger;
MATLAB OUTPUT;
int FILEOPEN = 0; //FILEOPEN is always zero if we aren't logging data

#include "Autopilot.h"
Autopilot control;
///NUM_MOTORS set in the autopilot routine header file
double MOTORSOUT[NUM_MOTORS]; //FIVE MOTORS (4 motors and 1 servo)

#ifdef TELEMETRY
#include "links/Serial.h"
float number_array[MAXFLOATS]; //MAXFLOATS is set to 10 in Serial.h right now
int number_of_numbers = 7;
#endif

static unsigned long currentTime = 0;
static unsigned long lastReceiverTime = 0; //REVISIT - Move this to the receiver class rcin

using namespace std;
using namespace Navio;

static unsigned long getCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  static unsigned long currentTime;
  currentTime = 1000000 * tv.tv_sec + tv.tv_usec;
  return currentTime;
}

////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////

int main(int argc, char* argv[]){

  printf("Alright so this code will output the number of input arguments \n");
  printf("and the input argument itself \n");
  //Argc is an integer so it will tell us how many input args ther are
  printf("The number of input arguments = %d \n",argc); 
  printf("The input arguments are as follows: \n");

  //This loop will then tell us what the input arguments are
  for (int idx = 0;idx<argc;idx++){
    printf("Input Arguments %d = %s \n",idx,argv[idx]);
  }

  ////////////////////SUDO CHECK/////////////////////////
  if (getuid()) {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
  }
  ////////////////////////////////////////////////////////

  ////////////////CHECK IF ARDUPILOT IS RUNNING///////////
  if (check_apm()){
    return 1;
  }
  //////////////////////////////////////////////////////

  ///////////LOG SETUP/////////////
  #ifdef LOGDATA
  //Create OUTPUT Matrix
  OUTPUT.zeros(22,1,"OUTPUT MATRIX");
  #endif
  /////////////////////////////////////////////////
  
  ///////////TIME SETUP///////////////////
  time_t now = time(0);
  char* time_msg = ctime(&now);
  tm *gmtm = gmtime(&now);
  time_msg = asctime(gmtm);
  ////////////////////////////////////////

  ////////////SERIAL SETUP//////////////
  #ifdef TELEMETRY
  //Initialize Serial Port
  //using this baudrate of 57600
  printf("Initializing the dev Port \n");
  my = SerialInit("/dev/ttyAMA0",57600);  //This is the UART port on the RPI
  printf("Dev Port Initialized. If no errors present we are currently listening \n");
  #endif

  ////////////LED SETUP///////////////
  printf("Creating LED Class \n");
  led = new Led_Navio2();
  led->initialize(); //Initialize the LED
  printf("LED Class Created. Setting Color to Yellow \n");
  led->setColor(Colors::Yellow);
  printf("LED is now yellow \n");
  ////////////////////////////////////////////

  ///////////////////RX SETUP///////////////////////
  rcin.NUMRX = 7;
  rcin.RXCHANNELS[0] = 0;
  rcin.RXCHANNELS[1] = 1;
  rcin.RXCHANNELS[2] = 2;
  rcin.RXCHANNELS[3] = 3;
  rcin.RXCHANNELS[4] = 4; 
  rcin.RXCHANNELS[5] = 5; 
  rcin.RXCHANNELS[6] = 6; 
  //How many channels do you want?
  //This #define NUM_CHANNELS is created in RCInput_Navio2.h
  //rcin ->channel_count = NUM_CHANNELS;
  //printf("There are %d channels!!!!! \n",rcin->channel_count);
  /////////////////////////////////////////////////////////////

  /////////////////ESC SETUP///////////////////////////
  rcout.num_motors = 5; //num signals
  rcout.PWMOUTPUTS[0] = 0; //Motor 1
  rcout.PWMOUTPUTS[1] = 1; //Motor 2
  rcout.PWMOUTPUTS[2] = 2; //Motor 3
  rcout.PWMOUTPUTS[3] = 3; //Motor 4
  rcout.PWMOUTPUTS[4] = 4; //Payload Release
  int out_res = rcout.rcoutSetup();
  if (out_res) {
    printf("RCOUT Error \n");
    return 0;
  }
  printf("Servo Setup Complete \n");
  sleep(1);
  ///////////////////////////////////////////////////////

  //////////////////Setup Controller/////////
  printf("Control Setup starting \n");
  control.setup(0); //Sending a 0 makes the barometer single thread. Otherwise send a 1 for multithreading
  printf("Pressure Temperature (after control setup) = %lf %lf \n",control.barotemp.Pressure,control.barotemp.Temperature);
  printf("Control Setup Complete \n");
  ///////////////////////////////////////////////////

  ///////////////GET TIMES//////////////////////////////
  printf("Getting Time \n");
  currentTime = getCurrentTime();
  static unsigned long previousTime = 0;
  double elapsedTime = 0;
  ///This is the equivalent of a void loop() routine on Arduino
  static unsigned long beginTime = getCurrentTime();
  lastReceiverTime = beginTime;
  double printTime = beginTime;
  double dataTime = beginTime;
  double timeSinceStart = 0;
  printf("Got Time \n");
  ////////////////////////////////////////////////////////

  ////////////////NOTIFY USER///////////////////////////
  printf("Time For take off \n");
  printf("This shits about to get #nasty #nohomo #shemademedoit \n");
  printf("But first....wait 10 seconds for the red light \n");
  printf("JK we got rid of this one. Go ahead and hit the arm switch and say goodbye \n");
  ///////////////////////////////////////////////////////

  while (1) {
    //////////////Get Current time/////////////////////////
    #ifdef PRINTSEVERYWHERE
    printf("Time...%lf \n",timeSinceStart);
    #endif
    previousTime = currentTime;
    currentTime = getCurrentTime();
    elapsedTime = (currentTime - previousTime) / 1000000.0;
    timeSinceStart = (currentTime - beginTime) / 1000000.0;
    /////////////////////////////////////////////////////////

    /////////////////POLL TELEMETRY//////////////////////////
    #ifdef TELEMETRY
    //From here we basically need to constantly read the serial port at least once
    //in the while loop and check for w\r from the computer so I'll need to write
    //set to 0 to turn echo off, 1 = all echos on, 2 = only echo if you receive anything
    int ok = SerialListen(&my,0);
    // w\r was received it means we need to respond
    if (ok == (119+13)) { //119 is ASCII for w and 13 is ASCII for \r
      //Once we get 119 and 13 we need to tell the groundstation that we heard them
      //so we send 119,13,10 - w\r\n
      SerialRespond(&my,0);
      //Once we've responded we must send whomever is talking to us some data
      number_array[0] = control.orientation.roll;
      number_array[1] = control.orientation.pitch;
      number_array[2] = control.orientation.yaw;
      number_array[3] = control.satellites.longitude;
      number_array[4] = control.satellites.latitude;
      number_array[5] = control.satellites.altitude;
      number_array[6] = currentTime/1000000.0;
      //Send over Serial (make sure to use the 1 hex \r format)
      SerialSendArray(&my,number_array,number_of_numbers,0); //The trailing 1 is for echo
    }
    #endif

    ///////////////POLL RECEIVER/////////////////
    int SIGNALSOFF;
    if ((currentTime - lastReceiverTime)/1000000.0 > RXPERIOD) {
      #ifdef PRINTSEVERYWHERE
      printf("Polling Receiver \n");
      #endif
      lastReceiverTime = currentTime;
      SIGNALSOFF = rcin.pollReceiver(currentTime/1000000.0);
      rollrc = rcin.RXVALS[3];
      pitchrc = rcin.RXVALS[1];
      throttle = rcin.RXVALS[2];
      yawrc = 1500-(rcin.RXVALS[0]-1500); //Whoops. This is why the rudder servo has different limits
      arm_switch = rcin.RXVALS[4];
      // = rcin.RXVALS[5];
      extra_servo = rcin.RXVALS[6];
    }
    ///////////////////////////////////////////////////////////////////

    ////////////////////CONTROL LOOP HANDLES PITOT, IMU and GPS//////
    #ifdef PRINTSEVERYWHERE
    printf("Running Controller Update \n");
    #endif
    control.update(currentTime,elapsedTime,FILEOPEN);
    #ifdef PRINTSEVERYWHERE
    printf("Controller Updated \n");
    #endif

    //////////////////MANUAL VS AUTOPILOT AND ARM SWITCH///////////////
    
    #ifdef PRINTSEVERYWHERE
    printf("Checking Config Switch \n");
    #endif

    if ((arm_switch > 1500) && (throttle < 1100) && (arm_ok == 0)) {
      arm_ok = 1;
    }
    
    if((arm_switch > 1500) && (arm_ok)){
      //Arm switch is up
      led->setColor(Colors::Green);
      #ifdef LOGDATA
      if (FILEOPEN == 0) {
      	//Mount USB stick to we can plot easier in the field
	#ifdef USEUSB
	printf("Mounting USB!");
      	system("sudo mount /dev/sda1 /media/mntpt/");
	#endif

        //The file is not open so open it
        logger.findfile(argv[1]);
        logger.open();
        logger.printchar(time_msg);
        FILEOPEN = 1; //Set this flag to 1 to signify that the file is open
      }
      #endif

      #ifdef NOPANIC
      oh_shit = SERVO_MIN;
      #endif
      
      /////AIRCRAFT IS ARMED
      ////AUTOPILOT IS ALWAYS ON
      control.loop(rollrc, pitchrc, throttle, yawrc,extra_servo,elapsedTime,0);
      for (int i = 0;i<NUM_MOTORS;i++) {
	MOTORSOUT[i] = control.signals[i];
      }
    } else {
      arm_ok = 0;
      //Arm switch is down or we've lost receiver signal
      led->setColor(Colors::Yellow);
      //Check and see if we need to close the file
      if (arm_switch != 0) { //this is a check to make sure we haven't lost receiver signal

	//Reset anti windup
	control.reset();
	
	#ifdef LOGDATA
	if (FILEOPEN == 1) {
	  FILEOPEN = 0;
	  logger.close();	  
	  //Unmount USB stick so we make sure everything is flushed out
          #ifdef USEUSB
	  printf("Unmounting USB! \n");
	  system("sudo umount /dev/sda1");
          #endif
	}
	#endif
      }
      //Check GPS Health
      /*int nogps = control.satellites.status();
      #ifdef PRINTSEVERYWHERE
      printf("Setting LED \n");
      #endif
      if (nogps) {
        led->setColor(Colors::Red);
      } else {
        led->setColor(Colors::Blue);
	}*/
      if (arm_switch == 0) {
	//This means that we've lost receiver signal so
	//default to failsafe
	for (int i = 0;i<NUM_MOTORS;i++) {
	  MOTORSOUT[i] = SERVO_MIN;
	}
      } else {
	//Down here though if we've actually thrown the switch properly
	//we can send default control signals
	for (int i = 0;i<NUM_MOTORS;i++) {
	  MOTORSOUT[i] = THROTTLE_MIN;
	}
	MOTORSOUT[4] = SERVO_MAX;
      }
    }  //end ARM SWITCH
    ///////////////////////////////////////////////

    if (SIGNALSOFF) {
      led->setColor(Colors::Yellow);
    }

    /////ADD SATURATION BLOCK
    for (int i = 0;i<NUM_MOTORS;i++) {
      if (MOTORSOUT[i] > SERVO_MAX) {
	MOTORSOUT[i] = SERVO_MAX;
      }
      if ((arm_switch > 1500) && (arm_ok)) {
	if (MOTORSOUT[i] < IDLE) {
	  MOTORSOUT[i] = IDLE;
	}
      } else {
	if (MOTORSOUT[i] < SERVO_MIN) {
	  MOTORSOUT[i] = SERVO_MIN;
	}
      }
    }

    double out_signal = 1500-(extra_servo-1500);
    MOTORSOUT[4] = out_signal;

    /////////////////////
    
    ////////////////Send signal to ESCs//////////////
    #ifdef PRINTSEVERYWHERE
    printf("Sending Signal to motors \n");
    #endif
    for (int i = 0;i<NUM_MOTORS;i++) {
      rcout.set_duty_cycle(i,MOTORSOUT[i]);
    }
    ////////////////////////////////////////////////

    ///////////////LOGDATA////////////////////////////////
    #ifdef LOGDATA
    if ((((currentTime - dataTime)/1000000.0) > LOGPERIOD) && (FILEOPEN == 1)) {
      #ifdef PRINTSEVERYWHERE
      printf("Logging Data \n");
      #endif
      dataTime = currentTime;
      OUTPUT.set(1,1,currentTime);
      OUTPUT.set(2,1,control.orientation.roll);
      OUTPUT.set(3,1,control.orientation.pitch);
      OUTPUT.set(4,1,control.orientation.yaw);
      OUTPUT.set(5,1,control.orientation.roll_rate);
      OUTPUT.set(6,1,control.orientation.pitch_rate);
      OUTPUT.set(7,1,control.orientation.yaw_rate);
      OUTPUT.set(8,1,rollrc);
      OUTPUT.set(9,1,pitchrc);
      OUTPUT.set(10,1,yawrc);
      OUTPUT.set(11,1,throttle);
      OUTPUT.set(12,1,arm_switch);
      OUTPUT.set(13,1,extra_servo);
      OUTPUT.set(14,1,MOTORSOUT[0]); //Motor 1
      OUTPUT.set(15,1,MOTORSOUT[1]); //2 
      OUTPUT.set(16,1,MOTORSOUT[2]); //3
      OUTPUT.set(17,1,MOTORSOUT[3]); //4
      OUTPUT.set(18,1,MOTORSOUT[4]); //extra servo
      OUTPUT.set(19,1,control.satellites.longitude); 
      OUTPUT.set(20,1,control.satellites.latitude); 
      OUTPUT.set(21,1,control.satellites.altitude); 
      OUTPUT.set(22,1,control.satellites.speed);
      OUTPUT.set(23,1,control.barotemp.Altitude);
      logger.println(OUTPUT);
    }
    #endif
    ///////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////
    #ifdef DEBUGPRINTS
    if ((currentTime - printTime) / 1000000.0 > DEBUGPERIOD) { //print at 20 Hz
      printTime = currentTime;
      //Time
      printf("%lf %lf ",printTime/1000000.0,elapsedTime);
      //PTP
      printf("%+05.2f %+05.2f %+05.2f ",control.orientation.roll,control.orientation.pitch,control.orientation.yaw);
      //PQR
      //printf("%+05.2f %+05.2f %+05.2f ",gx,gy,gz);
      //PQR_filtered
      //printf("%+05.2f %+05.2f %+05.2f ",control.orientation.roll_rate,control.orientation.pitch_rate,control.orientation.yaw_rate);
      //Rec Signals
      printf("%d %d %d %d %d %d ",throttle,rollrc,pitchrc,yawrc,arm_switch,extra_servo);
      //PWM outputs
      for (int i = 0;i <NUM_MOTORS;i++) {
	printf("%lf ", MOTORSOUT[i]);
      }
      //BarometerTemperature Health
      //printf("%lf %lf %lf ",control.barotemp.Pressure,control.barotemp.Temperature,control.barotemp.Altitude);
      //Pitot Tube Speed
      //printf("%lf %f %f ",control.pitots.V_avg[0],control.pitots.windspeed[0],control.pitots.windspeed_filtered[0]);
      //printf("%lf %lf %lf %lf ",control.pitots.windspeed_filtered[0],control.pitots.windspeed_filtered[1],control.satellites.speed,control.satellites.latitude);
      //printf("%lf %lf %lf ",control.satellites.longitude,control.satellites.latitude,control.satellites.altitude);
      //Newline
      printf("\n");
    }
    #endif
    ///////////////////////////////////////////////////////////////////

   } //end while

  return 0;
  
}
