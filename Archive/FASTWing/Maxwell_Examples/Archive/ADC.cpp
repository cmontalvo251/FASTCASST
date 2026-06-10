//ADC for pitot tube code for meta
#include <unistd.h>
#include "Navio2/PWM.h"
#include "Navio2/RCOutput_Navio2.h"
#include <Navio2/ADC_Navio2.h>
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

//#define DEBUGPRINTS
#define USEAHRS
#define LOGDATA

using namespace std;
using namespace Navio;

RCOutput_Navio2 rcout;

RCInput_Navio2 rcin;

Led_Navio2 *led;
InertialSensor *imu;

#ifdef LOGDATA
#include "links/Datalogger.h"
#include "links/MATLAB.h"
MATLAB OUTPUT;
int FILEOPEN = 0;
Datalogger logger;
#endif

#ifdef USEAHRS
#include "Navio2/AHRS.hpp"
AHRS ahrs;
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

double MOTORSOUT[4];
//double MOTORSOUT_[4];
//double inSIG_[4];
//double inSIG[4];

//#include "links/Aerodynamics.h"
//Aerodynamics aero;

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

static unsigned long getCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  static unsigned long currentTime;
  currentTime = 1000000 * tv.tv_sec + tv.tv_usec;
  return currentTime;
}

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

std::unique_ptr <ADC> get_converter(){
  if (get_navio_version() == NAVIO2){
    auto ptr = std::unique_ptr <ADC>{ new ADC_Navio2() };
    return ptr;
  } /*else{
    auto ptr = std::unique_ptr <ADC>{ new ADC_Navio() };
    return ptr;
    }*/
}

int main(int argc, char *argv[]){


  #ifdef LOGDATA
  //Create OUTPUT Matrix
  OUTPUT.zeros(20,1,"OUTPUT MATRIX");

  //Get Current Time
  time_t now = time(0);
  char* time_msg = ctime(&now);
  tm *gmtm = gmtime(&now);
  time_msg = asctime(gmtm);
  #endif


  led = new Led_Navio2();
  if (!led->initialize()) {
    return EXIT_FAILURE;
  }

  led->setColor(Colors::Yellow);

  if (getuid()) {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
  }

  if (check_apm()){
    return 1;
  }

  std::vector<double> pos_data;
  Ublox gps;

  auto adc = get_converter();
  adc->initialize();
  float results[adc->get_channel_count()] = {0.0f};
  printf("Starting ADC...\n");

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

  int imu_res = imuSetup();
  if (!imu_res) {
    return EXIT_FAILURE;
  }

  static unsigned long currentTime = getCurrentTime();
  static unsigned long previousTime = 0;
  float elapsedTime = 0;
  ///This is the equivalent of a void loop() routine on Arduino
  static unsigned long beginTime = getCurrentTime();
  lastReceiverTime = beginTime;
  float printTime = beginTime;
  float dataTime = beginTime;
  float timeSinceStart = 0;

  printf("Time For take off \n");
  sleep(1);
  printf("This shits about to get #nasty #nohomo #shemakemedoit \n");

  if(gps.testConnection()){
    printf("Ublox test OK\n");
    if(!gps.configureSolutionRate(1000)){
      printf("Setting new rate: FAILED\n");
    }
  }

  printf("But first....wait 10 seconds for the red light \n");
  //printf("Starting While Loop!\n");

  while (1) {
    //Get Current time
    previousTime = currentTime;
    currentTime = getCurrentTime();
    elapsedTime = (currentTime - previousTime) / 1000000.0;
    timeSinceStart = (currentTime - beginTime) / 1000000.0;
    //printf("Stuff... \n");

    if ((currentTime - lastReceiverTime)/1000000.0 > 0.1) {
      lastReceiverTime = currentTime;
      rcin.pollReceiver();
      rollrc = rcin.RXVALS[0];
      pitchrc = rcin.RXVALS[1];
      throttle = rcin.RXVALS[2];
      yawrc = rcin.RXVALS[3];
      configrc = rcin.RXVALS[4];
      //arm_switch = rcin.RXVALS[5];
      //printf("Polling Receiver \n");
    }

    imuLoop(elapsedTime);
    //printf("IMU... \n");

    if ((currentTime - lastGPSTime)/1000000.0 > 1.0) {
      lastGPSTime = currentTime;
      if(gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1){
	//printf("GPS Millisecond Time of Week: %.01f s\n", pos_data[0]/1000);
        //printf("Longitude: %1f\n", pos_data[1]/10000000);
	//printf("Latitude: %1f\n", pos_data[2]/10000000);
	//printf("Height above Ellipsoid: %.31f m\n", pos_data[3]/1000);
	//printf("Height above mean sea level: %.31f m\n", pos_data[4]/1000);
	//printf("Horizontal Accuracy Estimate: %.31f m\n", pos_data[5]/1000);
	//printf("Vertical Accuracy Estimate: %.31f m\n", pos_data[6]/1000);
      }
      if(gps.decodeSingleMessage(Ublox::NAV_STATUS, pos_data) == 1){
	//printf("Current gps status: \n");
	//printf("gpsfixOK: %d\n", ((int)pos_data[1] & 0x01));

	/*printf("gps fix status: ");
	switch((int)pos_data[0]){
	case 0x00:
	  printf("no fix\n");
	  break;
	case 0x01:
	  printf("dead reckoning only\n");
	  break;
	case 0x02:
	  printf("2D-fix\n");
	  break;
	case 0x03:
	  printf("3D-fix\n");
	  break;
	case 0x04:
	  printf("gps + dead reckoning combined\n");
	  break;
	case 0x05:
	  printf("time only fix\n");
	  break;
	default:
	  printf("Reserved value. Current state unknown\n");
	  break;
	}
	printf("\n");*/
	}
    }
    
    if ((currentTime - lastADCTime)/1000000.0 > 1.0) {
      lastADCTime = currentTime;
      for (int i = 0; i < adc->get_channel_count(); i++){
	results[i] = adc->read(i);
	if (results[i] == READ_FAILED)
	  return EXIT_FAILURE;
	printf("A%d: %.4fV ", i, results[i] / 1000);
      }
      printf("\n");
    }

    if(configrc > 1300) {
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
      MOTORSOUT[0] = throttle;
      MOTORSOUT[1] = rollrc;
      MOTORSOUT[2] = pitchrc;
      MOTORSOUT[3] = yawrc;
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
	  led->setColor(Colors::Red);
	}
	for (int i = 0;i < 4;i++) {
	  //MOTORSOUT[i] = SERVO_MIN;
	  MOTORSOUT[0] = SERVO_MIN;
	  MOTORSOUT[1] = SERVO_MID;
	  MOTORSOUT[2] = SERVO_MID;
	  MOTORSOUT[3] = SERVO_MID;
	  //MOTORSOUT_[i] = SERVO_MIN;
	  //inSIG_[i] = SERVO_MIN;
	  //inSIG[i] = SERVO_MIN;
	}
    } //end ARM SWITCH

    //Send signal to ESCs
    for (int i = 0;i < 4;i++) {
      rcout.set_duty_cycle(i,MOTORSOUT[i]);
    }

    #ifdef LOGDATA
    if ((((currentTime - dataTime)/1000000.0) > 0.02) && (FILEOPEN == 1)) {
      //printf("Logging Data \n");
      dataTime = currentTime;
      OUTPUT.set(1,1,currentTime);
      OUTPUT.set(2,1,roll);
      OUTPUT.set(3,1,pitch);
      OUTPUT.set(4,1,yaw);
      OUTPUT.set(5,1,gx_filtered);
      OUTPUT.set(6,1,gy_filtered);
      OUTPUT.set(7,1,gz_filtered);
      OUTPUT.set(8,1,rollrc);
      OUTPUT.set(9,1,pitchrc);
      OUTPUT.set(10,1,yawrc);
      OUTPUT.set(11,1,throttle);
      //OUTPUT.set(12,1,arm_switch);
      //OUTPUT.set(13,1,configrc);
      OUTPUT.set(12,1,MOTORSOUT[0]); //Throttle
      OUTPUT.set(13,1,MOTORSOUT[1]); //Rollrc
      OUTPUT.set(14,1,MOTORSOUT[2]); //Pitchrc
      OUTPUT.set(15,1,MOTORSOUT[3]); //Yawrc
      OUTPUT.set(16,1,pos_data[0]/1000.0);
      OUTPUT.set(17,1,pos_data[1]/10000000.0);
      OUTPUT.set(18,1,pos_data[2]/10000000.0);
      OUTPUT.set(19,1,pos_data[3]/1000.0);
      //OUTPUT.set(22,1,pos_data[4]/1000.0);
      //OUTPUT.set(23,1,pos_data[5]/1000.0);
      //OUTPUT.set(24,1,pos_data[6]/1000.0);
      OUTPUT.set(20,1,results[4]/1000.0);
      logger.println(OUTPUT);
    }
    #endif

    #ifdef DEBUGPRINTS
    if ((currentTime - printTime) / 1000000.0 > 0.05) { //print at 20 Hz
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
      printf("%d %d %d %d %d %d ",throttle,rollrc,pitchrc,yawrc,configrc,arm_switch);
      //PWM outputs
      for (int i = 0;i < 4;i++) {
	printf("%lf ", MOTORSOUT[i]);
      }
      printf("\n");
    }
    #endif
    //printf("End... \n");
  } //end while
  
  return 0;
}
