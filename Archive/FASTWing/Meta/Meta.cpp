#include <cstdio>

#include <Navio2/Led_Navio2.h>
#include <Navio+/Led_Navio.h>

#include<Navio2/RCInput_Navio2.h>
#include<Common/Util.h>
#include<memory>
#include "Navio2/PWM.h"
#include "Navio+/RCOutput_Navio.h"
#include "Navio2/RCOutput_Navio2.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <sys/time.h>
#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>
//#include "AHRS.hpp"
#include "AHRS2.hpp"

#define G_SI 9.80665
#define PI   3.14159
#define SERVO_START 1100 /*mS*/ //initialize motor pwm
#define SERVO_MAX 1750 /*mS*/
#define SERVO_MID 1500
#define SERVO_MIN 1100 /*ms*/
#define PWM_OUTPUT5 4
#define PWM_OUTPUT6 5 
#define PWM_OUTPUT7 6
#define PWM_OUTPUT8 7
#define IDLE 1250 /*mS*/
#define READ_FAILED -1

#define DEBUGPRINTS

using namespace std;
using namespace Navio;

// Objects
InertialSensor *imu;
//AHRS    ahrs_mah;   // Mahony AHRS
AHRS	ahrs_mad;   // Madgwick AHRS --Let's ese only the MADWICK FILTER

// Sensor data

float ax, ay, az;
float gx, gy, gz;
float gx_filtered=0, gy_filtered=0, gz_filtered=0;
float mx, my, mz;

float qw, qx, qy, qz;
float negativeone = -1.0;
// Orientation data
float mad_roll = 0 , mad_pitch = 0 , mad_yaw = 0;
float mah_roll = 0 , mah_pitch = 0 , mah_yaw = 0;

// Timing data

float offset[3];
struct timeval tv;
float dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;

// Network data

//InertialSensor* create_inertial_sensor()
//{
//    InertialSensor *imu;

    //Defaulting to MPU9250
    
    //if (!strcmp(sensor_name, "mpu")) {
	// }
	//    else if (!strcmp(sensor_name, "lsm")) {
	//        printf("Selected: LSM9DS1\n");
	//        imu = new LSM9DS1();
	//    }
	//    else {
        //return NULL;
	//    }

//    return imu;
//}

//============================= Initial setup =================================

void imuSetup()
{
    //----------------------- MPU initialization ------------------------------

    imu->initialize();

    //-------------------------------------------------------------------------

    printf("Beginning Gyro calibration...\n");
    for(int i = 0; i<100; i++)
      {
	imu->update();
	imu->read_gyroscope(&gy, &gx, &gz);
	
	gx *= 180 / PI;
	gy *= 180 / PI;
	gz *= 180 / PI;
	
	offset[0] += (-gx*0.0175);
	offset[1] += (-gy*0.0175);
	offset[2] += (-gz*0.0175);
	usleep(10000);
      }
    offset[0]/=100.0;
    offset[1]/=100.0;
    offset[2]/=100.0;
    
    printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
    ahrs_mad.setGyroOffset(offset[0], offset[1], offset[2]);
    //ahrs_mah.setGyroOffset(offset[0], offset[1], offset[2]);
    //ahrs_mah.set_Kp(100);
}


std::unique_ptr <Led> get_led()
{
    if (get_navio_version() == NAVIO2)
    {
        auto ptr = std::unique_ptr <Led>{ new Led_Navio2() };
        return ptr;
    } else
    {
        auto ptr = std::unique_ptr <Led>{ new Led_Navio() };
        return ptr;
    }
}

void print_help()
{
  printf("Possible parameters:\nSensor selection: -i [sensor name]\n");
  printf("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
  printf("If you want to visualize IMU data on another machine,\n");
  printf("add IP address and port number (by default 7000):\n");
  printf("-i [sensor name] ipaddress portnumber\n");

}

static unsigned long getCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  static unsigned long currentTime;
  currentTime = 1000000 * tv.tv_sec + tv.tv_usec;
  return currentTime;
}

// static unsigned long getTimeElapsed(static unsigned long previousTime) {
//   static unsigned long currentTime = getCurrentTime();
//   static unsigned long elapsedTime = currentTime - previousTime;
//   return elapsedTime;
// }

//============================== Main IMU loop ====================================

void imuLoop(float elapsedTime)
{
  //----------------------- Calculate delta time ----------------------------
  // gettimeofday(&tv,NULL);
  // previoustime = currenttime;
  // currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
  // dt = (currenttime - previoustime) / 1000000.0;
  // if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
  // gettimeofday(&tv,NULL);
  // currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
  // dt = (currenttime - previoustime) / 1000000.0;

    //-------- Read raw measurements from the MPU and update AHRS --------------

    // Accel + gyro.
    /*    imu->update();
    imu->read_accelerometer(&ax, &ay, &az);
    imu->read_gyroscope(&gx, &gy, &gz);
    imu->read_magnetometer(&mx, &my, &mz);

    ax /= G_SI;
    ay /= G_SI;
    az /= G_SI;
    gx *= 180 / PI;
    gy *= 180 / PI;
    gz *= 180 / PI;

    ahrs_mah.updateMahonyIMU(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, dt);
    ahrs_mad.updateMadgwickIMU(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, dt);
*/
//     Accel + gyro + mag.
//     Soft and hard iron calibration required for proper function.
    imu->update();
    imu->read_accelerometer(&ax, &ay, &az);
    imu->read_gyroscope(&gy, &gx, &gz);
    imu->read_magnetometer(&mx, &my, &mz);

    ax /= G_SI;
    ay /= G_SI;
    az /= G_SI;
    gx *= 180 / PI;
    gy *= 180 / PI;
    gz *= 180 / PI;

    ahrs_mad.updateMadgwick(ax, ay, az, gy*0.0175, gx*0.0175, gz*0.0175, my, mx, -mz,elapsedTime);
    //ahrs_mah.updateMahony(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, my, mx, -mz, dt);

    //------------------------ Read Euler angles ------------------------------

    ahrs_mad.getEuler(&mad_pitch, &mad_roll, &mad_yaw);

    //ahrs_mah.getEuler(&mah_roll, &mah_pitch, &mah_yaw);

    //qw = ahrs_mad.getW();
    //qx = ahrs_mad.getX();
    //qy = ahrs_mad.getY();
    //qz = ahrs_mad.getZ();

    //Run Complimentary Filter to get angular rate filtered
    //float time_constant = 0.5; // Number must be between 0 and 1. A value of 1 means you believe the measurement
    //a value of zero means you don't beleive the measurement. Since there is no model running though.
    //you end up just having nothing and getting zero for everthing.
    //gx_filtered += elapsedTime*(time_constant*gx + (1.0-time_constant)*gx_filtered);
    //gy_filtered += elapsedTime*(time_constant*gy + (1.0-time_constant)*gy_filtered);
    //gz_filtered += elapsedTime*(time_constant*gz + (1.0-time_constant)*gz_filtered);


    //Ok fuck complimentary filters. Running a First order Filter using Euler's method
    //float settling_time = 0.01;
    //float time_constant = 4.0/settling_time;
    //printf("Elapsed Time = %lf \n",elapsedTime);
    //printf("Time Constant = %lf \n",time_constant);
    //For Euler's method to work elapsedTime*time_constant < 1
    //When we were running this elapsedTime = 0.018 so let's let time_consant = 10
    float time_constant = 10;
    gx_filtered += elapsedTime*time_constant*(gx-gx_filtered);
    gy_filtered += elapsedTime*time_constant*(gy-gy_filtered);
    gz_filtered += elapsedTime*time_constant*(gz-gz_filtered);

    //------------------- Discard the time of the first cycle -----------------

//     if (!isFirst)
//     {
//     	if (dt > maxdt) maxdt = dt;
//     	if (dt < mindt) mindt = dt;
//     }
//     isFirst = 0;

//     //------------- Console and network output with a lowered rate ------------

//     dtsumm += dt;
//     if(dtsumm > 0.05)
//     {
//         // Console output
//         printf("Madgwick: ROLL: %+05.2f\t PITCH: %+05.2f\t YAW: %+05.2f\t PERIOD %.4fs\t RATE %dHz \n", mad_roll, mad_pitch, mad_yaw * -1, dt, int(1/dt));
//         printf("Mahony :  ROLL: %+05.2f\t PITCH: %+05.2f\t YAW: %+05.2f\t PERIOD %.4fs\t RATE %dHz \n", mah_roll, mah_pitch, mah_yaw * -1, dt, int(1/dt));
// //	printf("Quaternion: w:  %+05.2f\t x:  %+05.2f\t y:  %+05.2f\t z:  %+05.2f\n",qw,qx,qy,qz);

//         // Network output
// //        sprintf(sendline,"%10f %10f %10f %10f %dHz\n", ahrs.getW(), ahrs.getX(), ahrs.getY(), ahrs.getZ(), int(1/dt));
// //        sendto(sockfd, sendline, strlen(sendline), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));

//         dtsumm = 0;
//     }
}


std::unique_ptr <RCInput> get_rcin()
{

  auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
  #ifdef DEBUGPRINTS
  printf("Hey you are using a NAVIO2 TWO!!!!!!!!!\n");
  #endif
  return ptr;


}

std::unique_ptr <RCOutput> get_rcout()
{
  if (get_navio_version() == NAVIO2)
    {
      auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
      return ptr;
    } else
    {
      auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio() };
      return ptr;
    }

}

//=====================================Main Loop Start====================================
int main(int argc, char *argv[])
{
  auto led = get_led();
  if (!led->initialize())
    return EXIT_FAILURE;

  led->setColor(Colors::Yellow);

  //Check for ardupilot running
  if (check_apm()) {
    return 1;
  }

  //Get the pointer for writing to servos
  auto pwm = get_rcout();

  //Don't need to check twice
  //if (check_apm()) {
  //            return 1;
  //        }

  //Check and see if you are a root user
  if (getuid()) {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
  }


  //////////////////INTIALIZE ALL THE MOTORS///////////////////
  if( !(pwm->initialize(PWM_OUTPUT5)) ) {
    return 1;
  }

  if( !(pwm->initialize(PWM_OUTPUT6)) ) {
    return 1;
  }

  if( !(pwm->initialize(PWM_OUTPUT7)) ) {
    return 1;
  }

  if( !(pwm->initialize(PWM_OUTPUT8)) ) {
    return 1;
  }

  pwm->set_frequency(PWM_OUTPUT5, 50);

  if ( !(pwm->enable(PWM_OUTPUT5)) ) {
    return 1;
  }

  pwm->set_frequency(PWM_OUTPUT6, 50);

  if ( !(pwm->enable(PWM_OUTPUT6)) ) {
    return 1;
  }

  pwm->set_frequency(PWM_OUTPUT7, 50);

  if ( !(pwm->enable(PWM_OUTPUT7)) ) {
    return 1;
  }

  pwm->set_frequency(PWM_OUTPUT8, 50);

  if ( !(pwm->enable(PWM_OUTPUT8)) ) {
    return 1;
  }
  //////////////////////////////////////////////////////////

  ////Get the pointer for the Receiver
  auto rcin = get_rcin(); //See this code is using something super fucking fancy where instead of saying the variable is a double or of class RCInput they are telling the compiler that this is an auto variable. So
  //the compiler with "auto"matically figure out what type of variable this is.

  //This initialize routine will loop through NUM_CHANNELS so make sure to change NUM_CHANNELS
  //in RCINput_Navio2.h
  rcin->initialize();

  //How many channels do you want?
  //This #define NUM_CHANNELS is created in RCInput_Navio2.h
  rcin->channel_count = NUM_CHANNELS;
  printf("There are %d channels \n",rcin->channel_count);

  //start the real shit -- this actually starts writing to servos (motors)
  pwm->set_duty_cycle(PWM_OUTPUT5, SERVO_START); //this allows the motor to get a second to initialize before reading signals
  pwm->set_duty_cycle(PWM_OUTPUT6, SERVO_START);
  pwm->set_duty_cycle(PWM_OUTPUT7, SERVO_START);
  pwm->set_duty_cycle(PWM_OUTPUT8, SERVO_START);
  sleep(1); //hold that pwm signal for 5 seconds
  int arm_switch;
  int throttle;
  int rollrc;
  int pitchrc;
  int yawrc;
  printf("Time For take off \n"); //Oh shit.
  sleep(1);
  printf("This shits about to get #nasty #nohomo #shemakemedoit \n");

  ///Initialize the Kalman Filter shit. -- How much money do you think this guy makes???
  //imu = create_inertial_sensor(sensor_name);
  //Actually this is the IMU. ended up overhauling the code after we called Shawn from Kansas City.
  printf("Selected: MPU9250\n");
  imu = new MPU9250();

  //Is this really gonna throw an error. Seems pointless to put this in but just in case we'll leave it
  if (!imu) {
    printf("Wrong sensor name. Select: mpu or lsm\n");
    return EXIT_FAILURE;
  }

  ///This sends out a probe. Quick recon
  if (!imu->probe()) {
    printf("Sensor not enable\n");
    return EXIT_FAILURE;
  }

  //Finish setting up the IMU
  imuSetup();

  //===================================

  ////^^^^Everything above here is like a setup routine

  static unsigned long currentTime = getCurrentTime();
  static unsigned long previousTime = 0;
  float elapsedTime = 0;
  int top_front_left_pwm = 0;
  int top_front_right_pwm = 0;
  int top_back_right_pwm = 0;
  int top_back_left_pwm = 0;
  int throttle_pwm = 0;
  int aileron_pwm = 0;
  int rudder_pwm = 0;
  int elevator_pwm = 0;
  float phic,thetac,psic;
  float kproll,kdroll,d_roll,kppitch,kdpitch,d_pitch,kyaw,d_yaw;
  ///This is the equivalent of a void loop() routine on Arduino
  static unsigned long beginTime = getCurrentTime();
  static unsigned long lastReceiverTime = getCurrentTime();
  float timeSinceStart = 0;
  while (true)
    {
      //Get Current time
      //#ifdef DEBUGPRINTS
      currentTime = getCurrentTime();
      elapsedTime = (currentTime - previousTime) / 1000000.0; 
      previousTime = currentTime;
      timeSinceStart = (currentTime - beginTime) / 1000000.0;
      //#endif
      
      //Printout loop health      
      //============read inputs=============
      //for(int i = 0; i < rcin->channel_count ; i++)
      //{
      //int period = rcin->read(i);
	  //printf("%d ", period);
	  //let's set the channel inputs, shall we?
      if ((currentTime - lastReceiverTime)/1000000.0 > 0.1) {
	lastReceiverTime = currentTime;
	rollrc = rcin->read(0);//the variables have an rc because roll, pitch. and yaw are used by the IMU.
	pitchrc = rcin->read(1);
	throttle = rcin->read(2);
	yawrc = rcin->read(3);
	arm_switch = rcin->read(6);
      }
      //	  //	printf("%d \n", arm_switch);
      //}

      //I'm pretty sure this is what gives us the ptp values
      imuLoop(elapsedTime); //why does it take so long to get through this loop?
      
      //================write outputs===============
      if(arm_switch < 1300){
	if (timeSinceStart > 10) {
	  led->setColor(Colors::Red);
	}
	throttle_pwm = SERVO_MIN;
	aileron_pwm = SERVO_MID;
        elevator_pwm = SERVO_MID;
        rudder_pwm = SERVO_MID;

	//	    printf("unarmed");
	//	    sleep(2);
      } else if (arm_switch > 1300) {
        led->setColor(Colors::Green);
	//	printf("Armed");
      	// if(throttle < IDLE){
	//   top_front_left_pwm = IDLE;//+ d_roll + d_pitch - d_yaw;
	//   top_front_right_pwm = IDLE;// - d_roll + d_pitch + d_yaw;
	//   top_back_right_pwm = IDLE;// -d_roll -d_pitch - d_yaw;
	//   top_back_left_pwm = IDLE;// + d_roll - d_pitch +d_yaw;

	//   //    printf("IDILING BY");
	//   //	    sleep(2);
	    
      	// } else if (throttle > IDLE) {
	 	  
	  //}	
	throttle_pwm = throttle;
	aileron_pwm = rollrc;
        elevator_pwm = pitchrc;
        rudder_pwm = yawrc;
	
	
      } //end of arm switch
      //carlos came up with the signal naming convention, i think it is bullshit.
      pwm->set_duty_cycle(PWM_OUTPUT5,throttle_pwm);
      pwm->set_duty_cycle(PWM_OUTPUT6,aileron_pwm);
      pwm->set_duty_cycle(PWM_OUTPUT7,elevator_pwm);
      pwm->set_duty_cycle(PWM_OUTPUT8,rudder_pwm);
	  
      #ifdef DEBUGPRINTS
      float printTime = currentTime / 1000000.0;
      //Time
      printf("%lf %lf ",printTime,elapsedTime);
      //PTP
      printf("%+05.2f %+05.2f %+05.2f ",mad_roll,mad_pitch,mad_yaw);
      //PQR
      printf("%+05.2f %+05.2f %+05.2f ",gx,gy,gz);
      //PQR_filtered
      printf("%+05.2f %+05.2f %+05.2f ",gx_filtered,gy_filtered,gz_filtered);
      //Rec Signals
      printf("%d %d %d %d %d ",throttle,rollrc,pitchrc,yawrc,arm_switch);
      //PWM outputs
      printf("%d %d %d %d ",throttle_pwm,aileron_pwm,elevator_pwm,rudder_pwm);
      //Newline
      printf("\n");
      #endif

    } //end of void loop

  return 0;
}
