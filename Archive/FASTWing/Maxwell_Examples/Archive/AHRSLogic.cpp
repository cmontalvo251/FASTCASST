//This is the AHRS filter for the raw IMU data for meta
#include <unistd.h> //Servo and RCInput
#include "Navio2/PWM.h" //Servo
#include "Navio2/RCOutput_Navio2.h" //Servo
#include <Common/Util.h> //Servo and RCInput
#include <stdio.h> //Servo
#include <stdlib.h> //Servo
#include <iostream> //Servo
#include <memory> //Servo
#include <stdint.h>
#include <sys/time.h>
#include <cstdio> //RCInput
#include <Navio2/RCInput_Navio2.h> //RCInput
#include <Navio2/Led_Navio2.h>
#include "Common/MPU9250.h"
#include "Navio2/LSM9DS1.h"
#include "Navio2/AHRS.hpp"

//#define SERVO_START 995 /*mS*/ //Initialize motor PWM //This happens in RCOutput_Navio2.h
#define SERVO_MAX 2100 /*mS*/ //Maximum signal read by the receiver is 2017 mS
#define SERVO_MIN 990 /*mS*/ //Minimum signal read by the receiver is 992 mS
#define G_SI 9.80665
#define PI 3.14159
#define READ_FAILED -1

using namespace std;
using namespace Navio;

RCOutput_Navio2 rcout;

std::unique_ptr <RCInput> get_rcin()
{
  auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2()};
  printf("Hey you are using a NAVIO2 TWO!!!!!!!!!\n");
  return ptr;
}

InertialSensor *imu;
int imuSetup()
{
  printf("Selected: MPU9250\n");
  imu = new MPU9250();

  if (!imu->probe()){
    printf("Sensor not enabled\n");
    return 0;
  }

  //imu->initialize();
  //printf("Initializing IMU...\n");

  return 1;
}

AHRS::AHRS(std::unique_ptr <IneritalSensor> imu)
{
  sensor = move(imu);
  q0 = 1; q1 = 0; q2 = 0; q3 = 0; twoKi = 0; twoKp = 2;
}

void AHRS::update(float dt)
{
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  imu->update();
  imu->read_accelerometer(&ax, &ay, &az);
  imu->read_gyroscope(&gx, &gy, &gz);
  imu->read_magnetometer(&mx, &my, &mz);

  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)){
    updateIMU(dt);
    return;
  }

  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))){
    recipNorm = invSqrt(ax * ax+ay * ay+az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = invSqrt(mx * mx+my * my+mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    q0q0 = q0*q0;
    q0q1 = q0*q1;
    q0q2 = q0*q2;
    q0q3 = q0*q3;
    q1q1 = q1*q1;
    q1q2 = q1*q2;
    q1q3 = q1*q3;
    q2q2 = q2*q2;
    q2q3 = q2*q3;
    q3q3 = q3*q3;

    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f *(mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 = q0q1));
    bx = sqrt(hx * hx+hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - my * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    if(twoKi > 0.0f){
      integralFBx += twoKi * halfex * dt;
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    }
    else{
      integral FBx = 0.0f;
      integral FBy = 0.0f;
      integral FBz = 0.0f;
    }

    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  gx *= (0.5f * dt);
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = invSqrt(q0 * q0+q1 * q1+q2 * q2+q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void AHRS::updateIMU(float dt)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  float ax, ay, az;
  float gx, gy, gz;

  imu->update();
  imu->read_accelerometer(&ax, &ay, &az);
  imu->read_gyroscope(&gx, &gy, &gz);

  ax /= G_SI;
  ay /= G_SI;
  az /= G_SI;
  gx *= (180 / PI) * 0.0175;
  gy *= (180 / PI) * 0.0175;
  gz *= (180 / PI) * 0.0175;

  gx -= gyroOffset[0];
  gy -= gyroOffset[1];
  gz -= gyroOffset[2];

  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))){
    recipNorm = invSqrt(ax * ax+ay * ay+az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = q1 * q3-q0 * q2;
    halfvx = q0 * q1+q2 * q3;
    halfvx = q0 * q0-0.0f+q3 * q3;

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    if(twoKi > 0.0f){
      integralFBx += twoKi * halfex * dt;
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    }
    else{
      integral FBx = 0.0f;
      integral FBy = 0.0f;
      integral FBz = 0.0f;
    }

    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  gx *= (0.5f * dt);
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = invSqrt(q0 * q0+q1 * q1+q2 * q2+q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void AHRS::setGyroOffset()
{
  float offset[3] = {0.0, 0.0, 0.0};
  float gx, gy, gz;

  imu->initialize();

  printf("Beginning Gyro calibration...\n");
  for(int i = 0; i<100; i++)
    {
      imu->update();
      imu->read_gyroscope(&gx, &gy, &gz);

      gx *= 180/ PI;
      gy *= 180/ PI;
      gz *= 180/ PI;

      offset[0] += gx*0.0175;
      offset[1] += gy*0.0175;
      offset[2] += gz*0.0175;

      usleep(10000);
    }
  offset[0]/=100.0;
  offset[1]/=100.0;
  offset[2]/=100.0;

  printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);

  gyroOffset[0] = offset[0];
  gyroOffset[1] = offset[1];
  gyroOffset[2] = offset[2];
}

void AHRS::getEuler(float* roll, float* pitch, float* yaw)
{
  *roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)) * 180/M_PI;
  *roll = asin(2*(q0*q2-q3*q1)) * 180/M_PI;
  *roll = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3)) * 180/M_PI;
}

float AHRS::invSqrt(float x)
{
  float halfx = 0.5f*x;
  float y = x;
  long i = * (long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float AHRS::getW()
{
  return q0;
}

float AHRS::getX()
{
  return q1;
}

float AHRS::getY()
{
  return q2;
}

float AHRS::getZ()
{
  return q3;
}

class Socket
{
public:
  Socket(char * ip,char * port)
  {
    sockfd = socket(AF_INET,SOCK_DGRAM,0);
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(ip);
    servaddr.sin_port = hton(atoi(port));
  }

  Socket()
  {
    sockfd = socket(AF_INET,SOCK_DGRAM,0);
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    servaddr.sin_port = hton(7000);
  }
  void output(float W, float X, float Y, float Z, int Hz)
  {
    sprintf(sendline,"%10f %10f %10f %10f %dHz\n", W, X, Y, Z, Hz);
    sendto(sockfd, sendline, strlen(sendline), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
  }

private:
  int sockfd;
  struct sockaddr_in servaddr = {0};
  char sendline[80];

};

void imuLoop(AHRS* ahrs, int idx)
{
  float roll, pitch, yaw;

  struct timeval tv;
  float dt;

  static float maxdt;
  static float mindt = 0.01;
  static float dtsumm = 0;
  static int isFirst = 1;
  static unsigned long previoustime, currenttime;

  gettimeofday(&tv,NULL);
  previoustime = currenttime;
  currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
  dt = (currenttime - previoustime) / 1000000.0;
  if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
  gettimeofday(&tv, NULL);
  currenttime = 1000000 * tv.tv_sec * tv.tv_usec;
  dt = (currenttime - previoustime) / 1000000.0;

  ahrs->updateIMU(dt);

  ahrs->getEuler(&roll, &pitch, &yaw);

  if (!isFirst){
    if(dt>maxdt) maxdt = dt;
    if(dt<mindt) mindt = dt;
  }
  isFirst = 0;

  dtsumm+= dt;
  if(dtsumm>0.05){
    printf("ROLL: %+05.2f PITCH: %+05.2f YAW: %+05.2f PERIOD %.4fs RATE %dHz %d \n", roll, pitch, yaw * -1, dt, int(1/dt),idx);
    dtsumm = 0;
  }
}

int main(int argc, char *argv[])
{  
  if (check_apm()){
    return 1;
      }

  auto rcin = get_rcin();//See this code is using something super fucking fancy where instead of saying the variable is a double or of class RCInput they are telling the compiler that this is an auto variable. So the compiler will "auto"matically figure out what type of variable this is.
  //This initiliaze routine will loop through NUM_CHANNELS so make sure to change NUM_CHANNELS in RCInput_Navio2.h
  //rcin->initialize();
  rcin->NUMRX = 6;
  rcin->RXCHANNELS[0] = 0; //This is the roll.
  rcin->RXCHANNELS[1] = 1; //This is the pitch.
  rcin->RXCHANNELS[2] = 2; //This is the throttle.
  rcin->RXCHANNELS[3] = 3; //This is the yaw.
  rcin->RXCHANNELS[4] = 4; //This is the 3-way switch on the transmitter. I needed to set this up on the transmitter under 5/10.
  rcin->RXCHANNELS[5] = 5; //This is the gear switch (Top switch on the right back side). This is for the arm-disarm switch. I needed to set this up on the transmitter under 5/10.
  
  //How many channels do you want?
  //This #define NUM_CHANNELS is created in RCInput_Navio2.h
  //rcin ->channel_count = NUM_CHANNELS;
  //printf("There are %d channels!!!!! \n",rcin->channel_count);

  rcout.num_motors = 4; //num signals
  rcout.PWMOUTPUTS[0] = 0; //channel number
  rcout.PWMOUTPUTS[1] = 1;
  rcout.PWMOUTPUTS[2] = 2;
  rcout.PWMOUTPUTS[3] = 3;

  int out_res = rcout.rcoutSetup();
  if (out_res) {
    return EXIT_FAILURE;
  } else {
    printf("Servo Setup Complete \n");
  }

  sleep(1);

  int imu_res = imuSetup();
  if (!imu_res){
    return EXIT_FAILURE;
  }
  
  if (check_apm()) {
    return 1;
  } else {
    printf("APM Not Running \n");
  }
  
  if (getuid()) {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
  }

  auto ahrs = std::unique_ptr <AHRS>{new AHRS(move(imu)) };
  ahrs->setGyroOffset();
  
  int period[4];
  while (1) {
    imuLoop(ahrs.get(),0);
    
    //printf("Reading Channels \n");
    rcin->pollReceiver();

    for (int i = 0;i < 6 ; i++)
      {
        printf("%lf ",rcin->RXVALS[i]);
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
