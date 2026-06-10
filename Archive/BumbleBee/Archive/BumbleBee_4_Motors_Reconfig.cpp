#include <cstdio>
#include <Navio2/Led_Navio2.h>
#include<Navio2/RCInput_Navio2.h>
#include<Common/Util.h>
#include<memory>
#include "Navio2/PWM.h"
#include "Navio2/RCOutput_Navio2.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <memory>
#include <stdint.h>
#include <sys/time.h>
#include <Common/MPU9250.h>

#define G_SI 9.80665
#define SERVO_START 1100 /*mS*/ //initialize motor pwm
#define SERVO_MAX 1900 /*mS*/
#define SERVO_MIN 1100 /*ms*/
#define PWM_OUTPUT5 4
#define PWM_OUTPUT6 5 
#define PWM_OUTPUT7 6
#define PWM_OUTPUT8 7
#define IDLE 1200 /*mS*/
#define READ_FAILED -1

//#define DEBUGPRINTS
//#define USEMAD //use this for the madgwick filter
#define USEAHRS //use this for the AHRS filter
//#define USEKYRELLPID
//#define USEMONTALVOPID
#define USERECONFIG

using namespace std;
using namespace Navio;

//If we're using the reconfigurable controller we need a few things
#ifdef USERECONFIG
#include "links/MATLAB.h"
#include "links/Motor.h"
#include "links/Propulsion.h"
#include "links/mathp.h"
//Propulsion class to handle all the motor control stuff
Propulsion propel;
#else
#define PI   3.14159
#define RAD2DEG 180.0/PI
#define DEG2RAD PI/180.0
//Operands
#define CONSTRAIN(in,min,max) (in > max ? max : (in < min ? min : in))
#define wrap_Pi(x) (x < -Pi ? x+Pi2 : (x > Pi ? x - Pi2: x))
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#endif

//Objects
//Always need a sensor
InertialSensor *imu;

//Need an Led
Led_Navio2 *led;

//Need a PWM object
RCOutput_Navio2 *rcout;

//and a rx object - I edited the constructor class to run the initialize routine
//check RCInput_Navio2.cpp
RCInput_Navio2 rcin;

//The type of filter is up for debate
#ifdef USEMAD
#include "AHRS2.hpp"
AHRS ahrs_mad;   // Madgwick AHRS --Let's use only the MADWICK FILTER
#endif
#ifdef USEAHRS
#include "AHRS.hpp"
AHRS ahrs;
#endif

// Sensor data
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

//Receiver Signals
int arm_switch;
int throttle;
int rollrc;
int pitchrc;
int yawrc;
//Commands created by receiver
float roll_command=0,pitch_command=0,yaw_rx=0;

//PID Loop gains and other stuff for control
float kproll,kdroll,d_roll=0,kppitch,kdpitch,d_pitch=0,d_yaw=0,kipitch,kiroll;
float yawLock = 0;
float perrorIntegral=0,qerrorIntegral=0;
float pitcherrorIntegral=0,rollerrorIntegral=0;
int top_front_left_pwm = 0;
int top_front_right_pwm = 0;
int top_back_right_pwm = 0;
int top_back_left_pwm = 0;

#ifdef USERECONFIG
int MotorsSetup() {
  //Add Motors(rx,ry,rz,nx,ny,nz,yaw_dir)
  float rx = (0.0762*METERS2FT); //Meters converted to Feet
  float ry = (0.10115*METERS2FT);
  float rz = (0.0508*METERS2FT);
  //Bottom Motors
  //propel.addMotor(rx,-ry,0.0,0.0,0.0,-1.0,1);  //Add an extra variable here 
  //propel.addMotor(rx,ry,0.0,0.0,0.0,-1.0,-1);  //that's a 0 or 1.
  //propel.addMotor(-rx,ry,0.0,0.0,0.0,-1.0,1);/// var = 0, motorOff, var = 1, motorOn
  //propel.addMotor(-rx,-ry,0.0,0.0,0.0,-1.0,-1);
  //Bottom Motors
  propel.addMotor(rx,-ry,-rz,0.0,0.0,-1.0,1); //top_front_left
  propel.addMotor(rx,ry,-rz,0.0,0.0,-1.0,-1); //top_front_right
  propel.addMotor(-rx,ry,-rz,0.0,0.0,-1.0,1); //top_back_right
  propel.addMotor(-rx,-ry,-rz,0.0,0.0,-1.0,-1); //top_back_left
  //Pusher
  //float rxpusher = 0.1524*METERS2FT;
  //propel.addMotor(-rxpusher,0.0,0.0,1.0,0.0,0.0,1);

  //Finalize Motor Calcs
  float mass = ((735./1000.)*KG2LBF)/GEARTHENG; //grams to kg to lbf to slugs
  MATLAB I;
  I.eye(3,"Inertia");
  float Ixx = (0.0973*KG2LBF)*METERS2FT*METERS2FT; //kg-m^2 to lbf-ft^2
  float Iyy = (0.07413*KG2LBF)*METERS2FT*METERS2FT;
  float a = (6.5/100.)*METERS2FT; //cm to meters to feet
  float Izz = Ixx + Iyy - mass*a*a/6.0;
  I.set(1,1,Ixx);
  I.set(2,2,Iyy);
  I.set(3,3,Izz);
  I.disp();
  propel.MotorBeepRPY(mass,I,SERVO_MAX,SERVO_MIN);
}
#endif

int rcoutSetup() {
  rcout = new RCOutput_Navio2();

  //////////////////INTIALIZE ALL THE MOTORS///////////////////
  if( !(rcout->initialize(PWM_OUTPUT5)) ) {
    return 1;
  }

  if( !(rcout->initialize(PWM_OUTPUT6)) ) {
    return 1;
  }

  if( !(rcout->initialize(PWM_OUTPUT7)) ) {
    return 1;
  }

  if( !(rcout->initialize(PWM_OUTPUT8)) ) {
    return 1;
  }

  //Set the frequency and enable each pin
  rcout->set_frequency(PWM_OUTPUT5, 50);

  if ( !(rcout->enable(PWM_OUTPUT5)) ) {
    return 1;
  }

  rcout->set_frequency(PWM_OUTPUT6, 50);

  if ( !(rcout->enable(PWM_OUTPUT6)) ) {
    return 1;
  }

  rcout->set_frequency(PWM_OUTPUT7, 50);

  if ( !(rcout->enable(PWM_OUTPUT7)) ) {
    return 1;
  }

  rcout->set_frequency(PWM_OUTPUT8, 50);

  if ( !(rcout->enable(PWM_OUTPUT8)) ) {
    return 1;
  }

  //start the real shit -- this actually starts writing to servos (motors)
  //this allows the motor to get a second to initialize before reading signals
  rcout->set_duty_cycle(PWM_OUTPUT5, SERVO_START); 
  rcout->set_duty_cycle(PWM_OUTPUT6, SERVO_START);
  rcout->set_duty_cycle(PWM_OUTPUT7, SERVO_START);
  rcout->set_duty_cycle(PWM_OUTPUT8, SERVO_START);
  sleep(1); //hold that pwm signal for 5 seconds - 5 seconds? Or just 1?
  return 0;
}

void computeControl(float elapsedTime) {

  #if defined(USEKYRELLPID) || defined(USERECONFIG)

  //Gains
  #ifdef USEKYRELLPID
  float kp = 3.0;
  float kd = 0.9;
  float ki = 0.5;
  float kyaw = 1.0;
  #endif
  #ifdef USERECONFIG
  float kp,kd,ki,kyaw;
  //2/28/2019 - Tried
  //kp = 0.05;
  //kd = 0.016;
  //ki = 0.008;
  //kyaw = 0.016; -- Very twitchy not enough control
  //Gains below are still twitchy. I think I need to look at units.
  //kp = 0.5;
  //kd = 0.032;
  //ki = 0.016;  //Seems like there isn't enough integral gain
  //ki = 0.16;
  //kyaw = 0.016; Yaw seems like it could use some more
  //kyaw = 0.1;
  //After looking at units here is what I came up with.
  //I still haven't derived kyaw so I may need to later
  //Ummm yea so I made a code called compute_gains and there's no way I would have
  //guessed what these units were. This is insane. If this works my jaw will be on the floor.
  kp = 3.0;
  //kd = 2.3e-7;
  //ki = 7.1e-8;
  //kyaw = 2.0e-9;
  //^^^No where near enough control authority. No twitching but I can't control it.
  //kd = 2.3e-2;
  //ki = 7.1e-3;
  //kyaw = 2.0e-4;
  ///^^^^Drone was twitchy in roll and pitch - It was controllable kind of. I had no yaw authority.
  kd = 2.3e-3;
  ki = 7.1e-4;
  kyaw = 2.0e-2;
  //kd = N*(kd^2)*ry*kt/Ix
  #endif
  
  //Using Control Architecture from KyrellGod
  //Compute outer loop commands
  //2/26/2019 - twichiness so lowered from 4.5
  float phidot_command = kp*(roll_command - roll);
  float thetadot_command = kp*(pitch_command - pitch);

  //Constrain by 250
  phidot_command = CONSTRAIN(phidot_command,-250.0,250.0);
  thetadot_command = CONSTRAIN(thetadot_command,-250.0,250.0);

  //Yaw is a little wierd
  float psidot_command = 0;
  //If the pilot is hitting the stick
  //Then send the yaw signal directly to the inner loop
  //on 2/26/2019 the quad spun level but like a corkscrew so I'm turning off the yawLock variable and just controlling psidot_command
  //if (fabs(yaw_rx) > 5.0) {
  psidot_command = yaw_rx;
  //yawLock = yaw;
  //}
  //Otherwise compute error and compute rate command based on that
  //else {
  //float yaw_error = wrap_180(yawLock - yaw);
  //psidot_command = 1.5*yaw_error;
  //psidot_command = CONSTRAIN(psidot_command,-360,360);
  //}

  //These ptpdot commands are Euler angle derivatives and need to be converted to body frame angular rates
  float cos_theta = cos(pitch*DEG2RAD);
  float sin_theta = sin(pitch*DEG2RAD);
  float cos_phi = cos(roll*DEG2RAD);
  float sin_phi = sin(roll*DEG2RAD);
  float p_command = phidot_command - sin_theta*psidot_command;
  float q_command = cos_phi*thetadot_command + sin_phi*cos_theta*psidot_command;
  float r_command = -sin_phi*thetadot_command + cos_phi*cos_theta*psidot_command;

  //We then compute the inner loop commands which is a PI controller except for yaw
  float perror = (p_command - gx_filtered);
  float qerror = (q_command - gy_filtered);
  float rerror = (r_command - gz_filtered);

  //Need to compute integral error + anti-Windup
  perrorIntegral += perror*elapsedTime;
  perrorIntegral = CONSTRAIN(perrorIntegral,-50.0,50.0);
  qerrorIntegral += qerror*elapsedTime;
  qerrorIntegral = CONSTRAIN(qerrorIntegral,-50.0,50.0);

  //Then compute your commands note that yaw_out is just proportional
  //2/26/2019 - Trying to set integral to zero to see what happens - previously ki = 1.0
  //Getting rid of integral kind of worked but there was a significant amount of drift so trying 0.5 now
  //Adding a complimentary filter though
  d_roll = kd*perror + ki*perrorIntegral;
  d_pitch = kd*qerror + ki*qerrorIntegral;
  //Originally 2.7 - too twitchy
  //1.0 still made it corkscrew. Even getting rid of the outerloop didn't work
  //so I'm switching the sign on it now.
  //I'm putting the sign back to positive but I'm switching the d_yaw below
  d_yaw = kyaw*rerror;
  #endif

  #ifdef USEMONTALVOPID
  ///^^^Everything above was adapted from KyrellGod.
  //I have derived that controller in the laplace domain and there are differences but
  //there is enough of a similarity to make a controller that is very similar to what I've implemented
  //in the past

  //If the code below does not work there are somethings that could be different fundamentally
  //1.) Kyrell constrains kproll*roll_error to +-250.0
  //2.) Kyrell also constrains the integral error
  //^^If those two things don't make it work then it's one of the two things below.

  //On Feb 28th, 2019 I flight tested both of these controllers. The KyrellPID controller definitely operated
  //better but the controller below still worked. I believe the rotation from body to inertial is what's causing it coupled
  //with the integral on the inner loop to filter out noise
  
  //3.) Kyrell Rotates the kp_roll_error to the body frame.
  //4.) Apparently the zero dynamics are different (I derived this on paper)
  
  //================roll========================
  //kproll = 3.0; //Tried 5.0
  //kdroll = 1.8; //Tried 115
  //The gains above are from guess and check however the gains below are from adapting to KyrellGod
  //kproll = kp*kd + ki;
  //kiroll = kp*ki;
  //kdroll = kd;
  kproll = 3.2;
  kdroll = 0.9;
  kiroll = 1.5;
  float roll_error = roll_command - roll;
  rollerrorIntegral+=roll_error*elapsedTime;
  rollerrorIntegral = CONSTRAIN(rollerrorIntegral,-50.0,50.0);
  float kp_roll_error = kproll*roll_error;
  kp_roll_error = CONSTRAIN(kp_roll_error,-250.0,250.0); //May need this
  d_roll = kp_roll_error - kdroll*gx_filtered + kiroll*rollerrorIntegral;
  
  //==============pitch=======================
  //kppitch = 6.0; ///From auto_tune.m if pitch is in degrees and pitch rate is in rad/s you want
  //kdpitch = 2.3; //to try 40 and 300
  //The gains above are from guess and check however the gains below are from adapting to KyrellGod
  //kppitch = kp*kd + ki;
  //kipitch = kp*ki;
  //kdpitch = kd;
  kppitch = 3.2;
  kdpitch = 0.9;
  kipitch = 1.5;
  float pitch_error = pitch_command - pitch;
  pitcherrorIntegral+=pitch_error*elapsedTime;
  pitcherrorIntegral = CONSTRAIN(pitcherrorIntegral,-50.0,50.0);
  float kp_pitch_error = kppitch*pitch_error;
  kp_pitch_error = CONSTRAIN(kp_pitch_error,-250.0,250.0); //May need this
  d_pitch = kp_pitch_error - kdpitch*gy_filtered + kipitch*pitcherrorIntegral;
  
  //==================yaw====================
  //kyaw = 0.8;
  //The gains above are from guess and check but the gain below is from adapting from KyrellGod's code
  float kyaw = 1.0;
  d_yaw = kyaw*(yaw_rx - gz_filtered);
  #endif

  //Constrain individual commands
  d_roll = CONSTRAIN(d_roll,-500,500);
  d_pitch = CONSTRAIN(d_pitch,-500,500);
  d_yaw = CONSTRAIN(d_yaw,-500,500);

  #ifdef USERECONFIG
  propel.computeReconfigurable(d_roll,-d_pitch,d_yaw);
  //For now we will just hard code the extraction of the motors to get something flying
  top_back_left_pwm = throttle + propel.MOTORS[3].pwm_signal + IDLE - 996;
  top_back_right_pwm = throttle + propel.MOTORS[2].pwm_signal + IDLE - 996;
  top_front_right_pwm = throttle + propel.MOTORS[1].pwm_signal + IDLE - 996;
  top_front_left_pwm = throttle + propel.MOTORS[0].pwm_signal + IDLE - 996;
  #else
  top_back_left_pwm =   throttle + d_roll - d_pitch - d_yaw + IDLE - 996;
  top_back_right_pwm =  throttle - d_roll - d_pitch + d_yaw + IDLE - 996;
  top_front_right_pwm = throttle - d_roll + d_pitch - d_yaw + IDLE - 996;
  top_front_left_pwm =  throttle + d_roll + d_pitch + d_yaw + IDLE - 996;
  #endif

  //Saturation Blocks
  top_front_left_pwm = CONSTRAIN(top_front_left_pwm,IDLE,SERVO_MAX);
  top_front_right_pwm = CONSTRAIN(top_front_right_pwm,IDLE,SERVO_MAX);
  top_back_right_pwm = CONSTRAIN(top_back_right_pwm,IDLE,SERVO_MAX);
  top_back_left_pwm = CONSTRAIN(top_back_left_pwm,IDLE,SERVO_MAX);
  
}

int imuSetup()
{
  printf("Selected: MPU9250\n");
  imu = new MPU9250();

  ///Initialize the Kalman Filter shit. -- How much money do you think this guy makes???
  //imu = create_inertial_sensor(sensor_name);
  //Actually this is the IMU. ended up overhauling the code after we called Shawn from Kansas City.
  //We do this regardless of the filter type that we use

  ///This sends out a probe. Quick recon
  if (!imu->probe()) {
    printf("Sensor not enable\n");
    return 0;
  }
  
  //Setup for MADGWICK FILTER
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
#ifdef USEMAD
  ahrs_mad.setGyroOffset(offset[0], offset[1], offset[2]);
#endif

  //Setup for AHRS Filter
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

//============================== Main IMU loop ====================================

void imuLoop(float elapsedTime)
{
  imu->update();
  imu->read_accelerometer(&ax, &ay, &az);
  imu->read_gyroscope(&gx, &gy, &gz);
  
#ifdef USEMAD
  imu->read_magnetometer(&mx, &my, &mz);
  ax /= G_SI;
  ay /= G_SI;
  az /= G_SI;
  gx *= 180 / PI;
  gy *= 180 / PI;
  gz *= 180 / PI;
  ahrs_mad.updateMadgwick(ax, ay, az, gy*0.0175, gx*0.0175, gz*0.0175, my, mx, -mz,elapsedTime);
  ahrs_mad.getEuler(&pitch, &roll, &yaw);
#endif

#ifdef USEAHRS
  //ahrs.update(ax,ay,az,gx,gy,gz,mx,my,mz,elapsedTime);
  ahrs.updateNOMAG(ax,ay,az,gx,gy,gz,elapsedTime);
  ahrs.getEuler(&pitch,&roll,&yaw);
#endif

  //Run Complimentary Filter to get angular rate filtered
  //float time_constant = 0.5; // Number must be between 0 and 1.
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
  // float time_constant = 10;
  // gx_filtered += elapsedTime*time_constant*(gx-gx_filtered);
  // gy_filtered += elapsedTime*time_constant*(gy-gy_filtered);
  // gz_filtered += elapsedTime*time_constant*(gz-gz_filtered);
  float s = 0.1; //Changed filter from 0.2 to 0.1 on 2/28/2019
  gx_filtered = gx_filtered*s + (1-s)*gy*RAD2DEG;
  gy_filtered = gy_filtered*s + (1-s)*gx*RAD2DEG;
  gz_filtered = gz_filtered*s + (1-s)*(-gz)*RAD2DEG;

}


//=====================================Main Loop Start====================================
int main(int argc, char *argv[])
{
  //Set some LED status
  led = new Led_Navio2();
  if (!led->initialize()) {
    return EXIT_FAILURE;
  }
  //Set Color to Yellow
  led->setColor(Colors::Yellow);
  
  //Check and see if you are a root user
  if (getuid()) {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
  }
  
  //Check for ardupilot running
  if (check_apm()) {
    return 1;
  }

  //Setup the receiver
  double RXCHANNELS[NUM_CHANNELS],RXVALS[NUM_CHANNELS];
  int NUMRX = 5;
  RXCHANNELS[0] = 0; //roll is channel 0
  RXCHANNELS[1] = 1; //pitch is channel 1
  RXCHANNELS[2] = 2; //throttle in channel 2
  RXCHANNELS[3] = 3; //yaw is channel 3
  RXCHANNELS[4] = 6; //arm_switch is channel 6

  //Get the pointer for writing to servos
  //Once again we're using the Navio2 so we can get rid of the get_rcout function
  //I moved everything to a pwmSetup routine
  int out_res = rcoutSetup();
  if (out_res) {
    return EXIT_FAILURE;
  }

  //Setup the IMU.
  int imu_res = imuSetup();
  if (!imu_res) {
    return EXIT_FAILURE;
  }

  //If we're using the reconfigurable controller set up the motors
  #ifdef USERECONFIG
  MotorsSetup();
  #endif

  //===================================
  ////^^^^Everything above here is like a setup routine

  static unsigned long currentTime = getCurrentTime();
  static unsigned long previousTime = 0;
  float elapsedTime = 0;
  ///This is the equivalent of a void loop() routine on Arduino
  static unsigned long beginTime = getCurrentTime();
  lastReceiverTime = beginTime;
  float printTime = beginTime;
  float timeSinceStart = 0;

  printf("Time For take off \n"); //Oh shit.
  sleep(1);
  printf("This shits about to get #nasty #nohomo #shemakemedoit \n");

  //MAIN LOOP
  while (true)
    {
      //Get Current time
      previousTime = currentTime;
      currentTime = getCurrentTime();
      elapsedTime = (currentTime - previousTime) / 1000000.0; 
      timeSinceStart = (currentTime - beginTime) / 1000000.0;

      //Poll Receiver at 10 Hz
      if ((currentTime - lastReceiverTime)/1000000.0 > 0.1) {
	//Save Current Time
	lastReceiverTime = currentTime;
	//Poll Receiver
	//printf("Polling Receiver \n");
      	rcin.pollReceiver(RXCHANNELS,RXVALS,NUMRX);
	//printf("Receiver Polled \n");
	rollrc = RXVALS[0];
	pitchrc = RXVALS[1];
	throttle = RXVALS[2];
	yawrc = RXVALS[3];
	arm_switch = RXVALS[4];
	//printf("Values extracted \n");
	//================make commands===============
	//i need to take the pwm signals and make them angle commands
	//These commands only need to be generated when you poll the receiver
	roll_command = (40.0/500.0)*(float(rollrc)-1500.0);
	pitch_command = (40.0/500.0)*(float(pitchrc)-1500.0);
	yaw_rx = (135.0/500.0)*(float(yawrc)-1500.0); //this is really a yawRate command
      }

      //I'm pretty sure this is what gives us the ptp values
      imuLoop(elapsedTime); //why does it take so long to get through this loop?
      

      //================write outputs===============
      if(arm_switch > 1300) {
	led->setColor(Colors::Green);
	//================Controller==============
	//This runs every timestep since imuLoop is running everytimestep
	computeControl(elapsedTime);
	/////////////////////////////////////////
       } else {
       	if (timeSinceStart > 10) {
       	  led->setColor(Colors::Red);
       	}
	//Reset integral error
	pitcherrorIntegral = 0;
	rollerrorIntegral = 0;
	perrorIntegral = 0;
	qerrorIntegral = 0;
       	top_front_left_pwm = SERVO_MIN;
       	top_front_right_pwm = SERVO_MIN;
       	top_back_right_pwm = SERVO_MIN;
       	top_back_left_pwm = SERVO_MIN;
      } //end of arm switch

      //Send signals to motors
      //carlos came up with the signal naming convention, i think it is bullshit.
      rcout->set_duty_cycle(PWM_OUTPUT5,top_front_left_pwm);
      rcout->set_duty_cycle(PWM_OUTPUT6,top_front_right_pwm);
      rcout->set_duty_cycle(PWM_OUTPUT7,top_back_right_pwm);
      rcout->set_duty_cycle(PWM_OUTPUT8,top_back_left_pwm);
	  
      #ifdef DEBUGPRINTS
      if ((currentTime - printTime) / 1000000.0 > 0.05) { //print at 20 Hz
	printTime = currentTime;
	//Time
	printf("%lf %lf ",printTime/1000000.0,elapsedTime);
	//PTP
	printf("%+05.2f %+05.2f %+05.2f ",roll,pitch,yaw);
	//PQR
	//printf("%+05.2f %+05.2f %+05.2f ",gx,gy,gz);
	//PQR_filtered
	//printf("%+05.2f %+05.2f %+05.2f ",gx_filtered,gy_filtered,gz_filtered);
	//Rec Signals
	printf("%d %d %d %d %d ",throttle,rollrc,pitchrc,yawrc,arm_switch);
	//PWM outputs
	printf("%d %d %d %d ",top_front_left_pwm,top_front_right_pwm,top_back_right_pwm,top_back_left_pwm);
	#ifdef USERECONFIG
	for (int i = 0;i<propel.NUMMOTORS;i++) {
	  printf("%d ",(int)propel.MOTORS[i].pwm_signal);
	}
        #endif
	//d_ values
	printf("%+05.2f %+05.2f %+05.2f ",d_roll,d_pitch,d_yaw);
	//command angles	
	//printf("%+05.2f %+05.2f ",roll_command,pitch_command);
	//Newline
	printf("\n");
      }
      #endif

    } //end of void loop

  return 0;
}
