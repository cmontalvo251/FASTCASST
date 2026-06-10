//This is the AHRS filter for the raw IMU data for meta
#include <unistd.h>
#include "Navio2/PWM.h"
#include "Navio2/RCOutput_Navio2.h"
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
#include <Common/MPU9250.h>
//#include "Navio2/LSM9DS1.h"


//#define SERVO_START 1100 /*mS*/ //Initialize motor PWM - Setup in RCOutput_Navio.h
#define SERVO_MAX 2100 /*mS*/
#define SERVO_MIN 990 /*mS*/
#define G_SI 9.80665
#define PI   3.14159
#define RAD2DEG 180.0/PI
#define DEG2RAD PI/180.0
#define READ_FAILED -1

#define USEAHRS

using namespace std;
using namespace Navio;

RCOutput_Navio2 rcout;

std::unique_ptr <RCInput> get_rcin()
{
  auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2()};
  printf("Hey you are using a NAVIO2 TWO!!!!!!!!!\n");
  return ptr;
}

Led_Navio2 *led;
InertialSensor *imu;

#ifdef USEAHRS
#include "Navio2/AHRS.hpp"
AHRS ahrs;
#endif

float ax=0, ay=0, az=0;
float gx=0, gy=0, gz=0;
float mx=0, my=0, mz=0;
float qw, qx, qy, qz;
float negativeone = -1.0;
//Date for PID
float roll = 0 , pitch = 0 , yaw = 0;
float gx_filtered=0, gy_filtered=0, gz_filtered=0;

// Timing data
float offset[3];
struct timeval tv;
float dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;
static unsigned long lastReceiverTime = 0;

int imuSetup()
{
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

void imuLoop(float elapsedTime)
{
  imu->update();
  imu->read_accelerometer(&ax, &ay, &az);
  imu->read_gyroscope(&gx, &gy, &gz);

#ifdef USEAHRS
  ahrs.updateNOMAG(ax,ay,az,gx,gy,gz,elapsedTime);
  ahrs.getEuler(&pitch,&roll,&yaw);
  printf("Roll: %05.2f Pitch: %05.2f Yaw: %05.2f ", roll, pitch, yaw);
  //printf("P: %05.2f Q: %05.2f R: %05.2f ", gx, gy, gz);
#endif

  float s = 0.35;
  gx_filtered = gx_filtered*s + (1-s)*gy*RAD2DEG;
  gy_filtered = gy_filtered*s + (1-s)*gx*RAD2DEG;
  gz_filtered = gz_filtered*s + (1-s)*(-gz)*RAD2DEG;
  printf("P: %05.2f Q: %05.2f R: %05.2f \n", gx_filtered, gy_filtered, gz_filtered);
}

int main(int argc, char *argv[])
{
  //Set some LED status
  led = new Led_Navio2();
  if (!led->initialize()) {
    return EXIT_FAILURE;
  }
  //Set Color to Yellow
  led->setColor(Colors::Yellow);

  if (check_apm()){
    return 1;
      }

  auto rcin = get_rcin(); //See this code is using something super fucking fancy where instead of saying the variable is a double or of class RCInput they are telling the compiler that this is an auto variable. So the compiler with "aut"matically figure out what type of variable this is.
  //This initiliaze routine will loop through NUM_CHANNELS so make sure to change NUM_CHANNELS in RCInput_Navio2.h
  //rcin->initialize();
  rcin->NUMRX = 6;
  rcin->RXCHANNELS[0] = 0; //Roll
  rcin->RXCHANNELS[1] = 1; //Pitch
  rcin->RXCHANNELS[2] = 2; //Throttle
  rcin->RXCHANNELS[3] = 3; //Yaw
  rcin->RXCHANNELS[4] = 4; //3-way switch - setup on transmitter (5/10)
  rcin->RXCHANNELS[5] = 5; //Arm-Disarm switch - setup on transmitter (5/10)

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

  //Setup the IMU.
  int imu_res = imuSetup();
  if (!imu_res) {
    return EXIT_FAILURE;
  }

  static unsigned long currentTime = getCurrentTime();
  static unsigned long previousTime = 0;
  float elapsedTime = 0;
  static unsigned long beginTime = getCurrentTime();
  lastReceiverTime = beginTime;
  float printTime = beginTime;
  float dataTime = beginTime;
  float timeSinceStart = 0;

  printf("Time For take off \n"); //Oh shit.
  sleep(1);
 
  int period[4];
  while (1) {
    //Get Current time
    previousTime = currentTime;
    currentTime = getCurrentTime();
    elapsedTime = (currentTime - previousTime) / 1000000.0;
    timeSinceStart = (currentTime - beginTime) / 1000000.0;
    imuLoop(elapsedTime);

    //printf("Reading Channels \n");
    rcin->pollReceiver();

    for (int i = 0;i<5;i++) {
      //printf("%lf ",rcin->RXVALS[i]);
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
