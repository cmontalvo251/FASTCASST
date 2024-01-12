//This code should fly the airplane with no problems. Starting to thread the code...
#include <unistd.h>
#include "Navio2/PWM.h"
#include "Navio2/RCOutput_Navio2.h"
#include <Navio2/ADC_Navio2.h>
#include <Common/MS5611.h>
#include <Common/Util.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <memory>
#include <stdint.h>
#include <sys/time.h>
#include <cstdio>
#include <Navio2/RCInput_Navio2.h>
#include <Navio2/Led_Navio2.h>
#include <Common/Ublox.h>
#include "Common/MPU9250.h"
#include <ctime>
#include <pthread.h>

//#define SERVO_START 1100 /*mS*/ //Initialize motor PWM - Setup in RCOutput_Navio.h
#define SERVO_MAX 1900 /*mS*/
#define SERVO_MIN 1100 /*ms*/
#define SERVO_MID 1500 /*ms*/
#define THROTTLE_MIN 994
#define THROTTLE_MAX 2100
#define IDLE 1200 /*mS*/
#define G_SI 9.80665
#define PI   3.14159
#define RAD2DEG 180.0/PI
#define DEG2RAD PI/180.0
#define READ_FAILED -1

#define PITOTPERIOD 0.1
//#define BAROPERIOD 1.0
#define GPSPERIOD 1.0
#define RXPERIOD 0.05
#define LOGPERIOD 0.1
#define DEBUGPERIOD 0.05

//#define DEBUGPRINTS
//#define PRINTSEVERYWHERE
#define USEAHRS
#define LOGDATA
#define READIMU
#define USEGPS
#define USEBARO
#define USEPITOT

using namespace std;
using namespace Navio;

RCOutput_Navio2 rcout;
RCInput_Navio2 rcin;

Led_Navio2 *led;

#ifdef READIMU
InertialSensor *imu;
#endif

#ifdef LOGDATA
#include "links/Datalogger.h"
#include "links/MATLAB.h"
MATLAB OUTPUT;
int FILEOPEN = 0;
Datalogger logger;
#endif

#ifdef USEAHRS
#ifdef READIMU
#include "Navio2/AHRS.hpp"
AHRS ahrs;
#endif
#endif

float ax=0, ay=0, az=0;
float gx=0, gy=0, gz=0;
float mx=0, my=0, mz=0;
float qw, qx, qy, qz;
float negativeone = -1.0;
float roll = 0 , pitch = 0 , yaw = 0;
float gx_filtered=0, gy_filtered=0, gz_filtered=0;

//int arm_switch; //Used configrc (3-way) switch instead!
int throttle;
int rollrc;
int pitchrc;
int yawrc;
int configrc;

float baroPressure;

float offset[3];
struct timeval tv;
float dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;
static unsigned long lastReceiverTime = 0;
static unsigned long lastGPSTime = 0;
static unsigned long lastADCTime = 0;
static unsigned long lastBaroTime = 0;

double MOTORSOUT[4];
//double MOTORSOUT_[4];
//double inSIG_[4];
//double inSIG[4];

//#include "links/Aerodynamics.h"
//Aerodynamics aero;

#ifdef READIMU
int imuSetup(){
  printf("Selected: MPU9250\n");
  imu = new MPU9250();

  if (!imu->probe()) {
    printf("Sensor not enable\n");
    return 0;
  }

  imu->initialize();
  printf("Beginning Gyro calibration...\n");
  offset[0] = 0;
  offset[1] = 0;
  offset[2] = 0;
  for(int i = 0; i<100; i++)
    {
      imu->update();
      imu->read_gyroscope(&gx, &gy, &gz);
      gx *= 180 / PI;
      gy *= 180 / PI;
      gz *= 180 / PI;
      offset[0] += gx*0.0175;
      offset[1] += gy*0.0175;
      offset[2] += gz*0.0175;
      usleep(10000);
    }
  offset[0]/=100.0;
  offset[1]/=100.0;
  offset[2]/=100.0;

  printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);

  #ifdef USEAHRS
  ahrs.setGyroOffset(offset[0],offset[1],offset[2]);
  #endif

  return 1;
}
#endif

static unsigned long getCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  static unsigned long currentTime;
  currentTime = 1000000 * tv.tv_sec + tv.tv_usec;
  return currentTime;
}

#ifdef READIMU
void imuLoop(float elapsedTime){
  imu->update();
  imu->read_accelerometer(&ax, &ay, &az);
  imu->read_gyroscope(&gx, &gy, &gz);

  #ifdef USEAHRS
  //ahrs.update(ax,ay,az,gx,gy,gz,mx,my,mz,elapsedTime);
  ahrs.updateNOMAG(ax,ay,az,gx,gy,gz,elapsedTime);
  ahrs.getEuler(&pitch,&roll,&yaw);
  #endif

  float s = 0.35;
  gx_filtered = gx_filtered*s + (1-s)*gy*RAD2DEG;
  gy_filtered = gy_filtered*s + (1-s)*gx*RAD2DEG;
  gz_filtered = gz_filtered*s + (1-s)*(-gz)*RAD2DEG;
}
#endif

std::unique_ptr <ADC> get_converter(){
  if (get_navio_version() == NAVIO2){
    auto ptr = std::unique_ptr <ADC>{ new ADC_Navio2() };
    return ptr;
  }
}

#ifdef READIMU
void ControlLoop(float rollrc, float pitchrc, float roll, float pitch){
  //float W = 12.1; //Newtons
  //float m = 1.234; //kg
  //float Kpt = 75;
  //printf("rollrc: %f, ", rollrc);
  float roll_command = (45.0/515.0)*(rollrc-1500.0); //Convert from microsecond pulse to degrees
  //printf("roll_command: %f, ", roll_command);
  //printf("pitchrc: %f, ", pitchrc);
  float pitch_command = (45.0/515.0)*(pitchrc-1500.0);
  //printf("pitch_command: %f, ", pitch_command);
  float ThetaC = pitch;
  //printf("ThetaC: %f, ", ThetaC);
  float PhiC = roll;
  //printf("PhiC: %f, ", PhiC);

  float Kpe = -1;
  float Kde = -0.5;
  float dele;
  //printf("q: %f, ", gy_filtered);
  
  float Kpa = -1;
  float Kdp = -0.5;
  float dela;
  //printf("p: %f, ", gx_filtered);

  dele = Kpe*(ThetaC - pitch_command) - Kde*gy_filtered; //Find degree
  //printf("dele: %f, ", dele);
  dela = Kpa*(PhiC - roll_command) - Kdp*gx_filtered;
  //printf("dela: %f, ", dela);

  if(dele > SERVO_MAX){
    dele = SERVO_MAX;
  }
  if(dele < SERVO_MIN){
    dele = SERVO_MIN;
  }

  if(dela > SERVO_MAX){
    dela = SERVO_MAX;
  }
  if(dela < SERVO_MIN){
    dela = SERVO_MIN;
  }

  float roll_auto = (dela*(515.0/45.0))+1500.0; //Convert from degree to microsecond pulse
  //printf("roll_auto: %f\n", roll_auto);
  float pitch_auto = (dele*(515.0/45.0))+1500.0;
  //printf("pitch_auto: %f\n", pitch_auto);

  MOTORSOUT[2] = pitch_auto; //Send to motors
  MOTORSOUT[1] = roll_auto;
}
#endif

///////////////Barometer Threading Setup////////////////
//This works, and the barometer data is being recorded!!
#ifdef USEBARO
void * acquireBarometerData(void * barom){
  MS5611* barometer = (MS5611*)barom;

  while(1){
      barometer->refreshPressure();
      usleep(10000); //Waiting for pressure data to be ready
      barometer->readPressure();

      barometer->refreshTemperature();
      usleep(10000); //Waiting for pressure data to be ready
      barometer->readTemperature();

      barometer->calculatePressureAndTemperature();

      baroPressure = barometer->getPressure();

      //sleep(0.5);
  }

  pthread_exit(NULL);
  
}
#endif
////////////////////////////////////////////////////////

int main(int argc, char *argv[]){

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
  OUTPUT.zeros(20,1,"OUTPUT MATRIX");
  //Get Current Time
  time_t now = time(0);
  char* time_msg = ctime(&now);
  tm *gmtm = gmtime(&now);
  time_msg = asctime(gmtm);
  #endif

  ////////////LED SETUP///////////////
  led = new Led_Navio2();
  if (!led->initialize()) {
    return EXIT_FAILURE;
  }
  led->setColor(Colors::Yellow);
  ////////////////////////////////////////////

  ///////////BAROMETER SETUP///////////////////
  #ifdef USEBARO
  MS5611 baro;
  baro.initialize();
 
  #ifdef PRINTSEVERYWHERE
  printf("Baro \n");
  #endif

  pthread_t baro_thread;

  if(pthread_create(&baro_thread, NULL, acquireBarometerData, (void*)&baro)){
    printf("Error: Failed to create barometer thread\n");
    return 0;
  }
  #endif
  ///////////////////////////////////////////////

  //////////////////GPS SETUP//////////////////
  #ifdef USEGPS
  std::vector<double> pos_data,nav_data;
  Ublox gps;
  if(gps.testConnection()){
    printf("Ublox test OK\n");
    if(!gps.configureSolutionRate(1000)){
      printf("Setting new rate: FAILED\n");
    }
  } else {
    printf("GPS Test failed \n");
    return EXIT_FAILURE;
  }
  #endif
  //////////////////////////////////////////////

  ////////////////////PITOT SETUP///////////////////
  #ifdef USEPITOT
  auto adc = get_converter();
  adc->initialize();
  float adc_result = 0;
  //float results[adc->get_channel_count()] = {0.0f};
  printf("Starting ADC...\n");
  #endif
  ///////////////////////////////////////////////////

  ///////////////////RX SETUP///////////////////////
  rcin.NUMRX = 6;
  rcin.RXCHANNELS[0] = 0; //Roll
  rcin.RXCHANNELS[1] = 1; //Pitch
  rcin.RXCHANNELS[2] = 2; //Throttle
  rcin.RXCHANNELS[3] = 3; //Yaw
  rcin.RXCHANNELS[4] = 4; //3-way switch - setup on transmitter (5/10)
  //rcin.RXCHANNELS[5] = 5; //Arm-Disarm switch - setup on transmitter (5/10)
  //How many channels do you want?
  //This #define NUM_CHANNELS is created in RCInput_Navio2.h
  //rcin ->channel_count = NUM_CHANNELS;
  //printf("There are %d channels!!!!! \n",rcin->channel_count);
  /////////////////////////////////////////////////////////////

  /////////////////ESC SETUP///////////////////////////
  rcout.num_motors = 4; //num signals
  rcout.PWMOUTPUTS[0] = 0; //channel number - Roll
  rcout.PWMOUTPUTS[1] = 1; //Pitch
  rcout.PWMOUTPUTS[2] = 2; //Throttle
  rcout.PWMOUTPUTS[3] = 3; //Yaw
  int out_res = rcout.rcoutSetup();
  if (out_res) {
    return EXIT_FAILURE;
  } else {
    printf("Servo Setup Complete \n");
  }
  sleep(1);
  ///////////////////////////////////////////////////////

  //////////////////IMU SETUP//////////////////////////
  #ifdef READIMU
  int imu_res = imuSetup();
  if (!imu_res) {
    return EXIT_FAILURE;
  }
  #endif
  ///////////////////////////////////////////////////

  ///////////////GET TIMES//////////////////////////////
  static unsigned long currentTime = getCurrentTime();
  static unsigned long previousTime = 0;
  float elapsedTime = 0;
  ///This is the equivalent of a void loop() routine on Arduino
  static unsigned long beginTime = getCurrentTime();
  lastReceiverTime = beginTime;
  float printTime = beginTime;
  float dataTime = beginTime;
  float timeSinceStart = 0;
  ////////////////////////////////////////////////////////

  ////////////////NOTIFY USER///////////////////////////
  printf("Time For take off \n");
  sleep(1);
  printf("This shits about to get #nasty #nohomo #shemademedoit \n");
  printf("But first....wait 10 seconds for the red light \n");
  ///////////////////////////////////////////////////////

  while (1) {
    //////////////Get Current time/////////////////////////
    previousTime = currentTime;
    currentTime = getCurrentTime();
    elapsedTime = (currentTime - previousTime) / 1000000.0;
    timeSinceStart = (currentTime - beginTime) / 1000000.0;
    #ifdef PRINTSEVERYWHERE
    printf("Time... \n");
    #endif
    /////////////////////////////////////////////////////////

    ///////////////POLL RECEIVER/////////////////
    if ((currentTime - lastReceiverTime)/1000000.0 > RXPERIOD) {
      lastReceiverTime = currentTime;
      rcin.pollReceiver();
      rollrc = rcin.RXVALS[3];
      pitchrc = rcin.RXVALS[1];
      throttle = rcin.RXVALS[2];
      yawrc = SERVO_MID-(rcin.RXVALS[0]-SERVO_MID);
      configrc = rcin.RXVALS[4];
      //arm_switch = rcin.RXVALS[5];
      #ifdef PRINTSEVERYWHERE
      printf("Polling Receiver \n");
      #endif
    }
    ///////////////////////////////////////////////////

    //////////////////POLLING IMU AS QUICKLY AS POSSIBLE? WHY?////////////
    #ifdef READIMU
    imuLoop(elapsedTime);
    #ifdef PRINTSEVERYWHERE
    printf("IMU... \n");
    #endif
    #endif
    ///////////////////////////////////////////////////////////////////

    ///////////////////POLL GPS/////////////////////////////////
    #ifdef USEGPS
    if ((currentTime - lastGPSTime)/1000000.0 > GPSPERIOD) {
      lastGPSTime = currentTime;
      gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data);
      #ifdef PRINTSEVERYWHERE
      printf("GPS \n");
      #endif 
    }
    #endif
    //////////////////////////////////////////////////////////////

    ////////////////////POLL PITOT/////////////////
    #ifdef USEPITOT
    if ((currentTime - lastADCTime)/1000000.0 > PITOTPERIOD) {
      lastADCTime = currentTime;
      adc_result = adc->read(4);
      if (adc_result == READ_FAILED) {
	adc_result = -99;
      }
      #ifdef PRINTSEVERYWHERE
      printf("A%d: %.4fV ", 0, adc_result / 1000);
      printf("\n");
      #endif
    }
    #endif
    ////////////////////////////////////////////////////

    //////////////////MANUAL VS AUTOPILOT AND ARM SWITCH///////////////
    if(configrc > 1300){
      //Arm switch is up
      led->setColor(Colors::Green);
      #ifdef LOGDATA
      if (FILEOPEN == 0) {
	//The file is not open so open it
	logger.findfile("/home/pi/Data/output");
	logger.open();
	logger.printchar(time_msg);
	FILEOPEN = 1; //Set this flag to 1 to signify that the file is open
      }
      #endif
      //Probably need to add autopilot logic
      //Manual control signals
      if (configrc > 1800) {
	//Autotpilot
	MOTORSOUT[0] = throttle;
	MOTORSOUT[3] = yawrc;
	//Convert roll and pitch inputs to roll and pitch commands
	#ifdef READIMU
	ControlLoop(rollrc, pitchrc, roll, pitch);
	#else	
	MOTORSOUT[1] = rollrc;
	MOTORSOUT[2] = pitchrc;
	#endif
	//PD control on elevator and aileron
      } else {
	MOTORSOUT[0] = throttle;
	MOTORSOUT[1] = rollrc;
	MOTORSOUT[2] = pitchrc;
	MOTORSOUT[3] = yawrc;
      }
    }
    else if(1300 > configrc){ //Arm switch is down
        #ifdef LOGDATA
	//Check and see if we need to close the file
	if (FILEOPEN == 1) {
	  FILEOPEN = 0;
	  logger.close();
	}
	#endif
	if (timeSinceStart > 10) {
	  gps.decodeSingleMessage(Ublox::NAV_STATUS, nav_data);
	  if (int(nav_data[0]) == 0x00) {
	    led->setColor(Colors::Red);
	  } else {
	    led->setColor(Colors::Blue);
	  }
	}
	//for (int i = 0;i < 4;i++) {
	  //MOTORSOUT[i] = SERVO_MIN;
	  MOTORSOUT[0] = SERVO_MIN;
	  MOTORSOUT[1] = SERVO_MID;
	  MOTORSOUT[2] = SERVO_MID;
	  MOTORSOUT[3] = SERVO_MID;
	  //MOTORSOUT_[i] = SERVO_MIN;
	  //inSIG_[i] = SERVO_MIN;
	  //inSIG[i] = SERVO_MIN;
	  //}
    } //end ARM SWITCH
    ///////////////////////////////////////////////

    ////////////////Send signal to ESCs//////////////
    for (int i = 0;i < 4;i++) {
      rcout.set_duty_cycle(i,MOTORSOUT[i]);
    }
    ////////////////////////////////////////////////

    ///////////////LOGDATA////////////////////////////////
    #ifdef LOGDATA
    if ((((currentTime - dataTime)/1000000.0) > LOGPERIOD) && (FILEOPEN == 1)) {
      //printf("Logging Data \n");
      dataTime = currentTime;
      OUTPUT.set(1,1,currentTime);
      #ifdef READIMU
      OUTPUT.set(2,1,roll);
      OUTPUT.set(3,1,pitch);
      OUTPUT.set(4,1,yaw);
      OUTPUT.set(5,1,gx_filtered);
      OUTPUT.set(6,1,gy_filtered);
      OUTPUT.set(7,1,gz_filtered);
      #endif
      OUTPUT.set(8,1,rollrc);
      OUTPUT.set(9,1,pitchrc);
      OUTPUT.set(10,1,yawrc);
      OUTPUT.set(11,1,throttle);
      //OUTPUT.set(12,1,arm_switch);
      OUTPUT.set(12,1,configrc);
      OUTPUT.set(13,1,MOTORSOUT[0]); //Throttle
      OUTPUT.set(14,1,MOTORSOUT[1]); //Rollrc
      OUTPUT.set(15,1,MOTORSOUT[2]); //Pitchrc
      OUTPUT.set(16,1,MOTORSOUT[3]); //Yawrc
      #ifdef USEGPS
      //OUTPUT.set(16,1,pos_data[0]/1000.0); ///Time of the week Use Dropbox/Blackbox/Time/convertjulian2ymd.py
      OUTPUT.set(17,1,pos_data[1]/10000000.0); //lon - Maxwell says it may be lon lat
      OUTPUT.set(18,1,pos_data[2]/10000000.0); //lat - It really is lon lat
      OUTPUT.set(19,1,pos_data[3]/1000.0); ///height above ellipsoid 1984?
      #endif
      #ifdef USEPITOT
      OUTPUT.set(20,1,adc_result/1000.0);
      #endif
      #ifdef USEBARO
      OUTPUT.set(21,1,baroPressure);
      #endif
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
      //printf("%+05.2f %+05.2f %+05.2f ",roll,pitch,yaw);
      //PQR
      //printf("%+05.2f %+05.2f %+05.2f ",gx,gy,gz);
      //PQR_filtered
      //printf("%+05.2f %+05.2f %+05.2f ",gx_filtered,gy_filtered,gz_filtered);
      //Rec Signals
      printf("%d %d %d %d %d ",throttle,rollrc,pitchrc,yawrc,configrc);
      //PWM outputs
      for (int i = 0;i < 4;i++) {
	printf("%lf ", MOTORSOUT[i]);
      }
      #ifdef USEBARO
      printf("Temperature (C): %f, Pressure(mbar): %f ", baro.getTemperature(), baroPressure);
      #endif
      printf("\n");
    }
    #endif
    ///////////////////////////////////////////////////////////////////
    
  } //end while

  pthread_exit(NULL);
  
  return 0;
  
}
