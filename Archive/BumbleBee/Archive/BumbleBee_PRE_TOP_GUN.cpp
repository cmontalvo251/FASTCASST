#include <cstdio>
#include <Navio2/Led_Navio2.h>
#include <Navio2/RCInput_Navio2.h>
#include <Common/Util.h>
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
#define SERVO_MAX 1900 /*mS*/
#define SERVO_MIN 1100 /*ms*/
#define THROTTLE_MIN 994
#define THROTTLE_MAX 2100
#define IDLE 1200 /*mS*/
#define READ_FAILED -1

//#define DEBUGPRINTS
//#define USEMAD //use this for the madgwick filter
#define USEAHRS //use this for the AHRS filter
//#define USEKYRELLPID
//#define USEMONTALVOPID
#define USERECONFIG
#define LOGDATA

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

#ifdef LOGDATA
#include "links/Datalogger.h"
#include "links/MATLAB.h"
MATLAB OUTPUT;
int FILEOPEN = 0;
Datalogger logger;
#endif

//Objects
//Always need a sensor
InertialSensor *imu;

//Need an Led
Led_Navio2 *led;

//Need a PWM object
//#define PWM_OUTPUT5 4 -- These have been moved to the rcout initialize routine
//#define PWM_OUTPUT6 5 
//#define PWM_OUTPUT7 6
//#define PWM_OUTPUT8 7
RCOutput_Navio2 rcout;

//and a rx object - I edited the constructor class to run the initialize routine
//check RCInput_Navio2.cpp
RCInput_Navio2 rcin;

//The type of filter is up for debate
#ifdef USEMAD
#include "Navio2/AHRS2.hpp"
AHRS ahrs_mad;   // Madgwick AHRS --Let's use only the MADWICK FILTER
#endif
#ifdef USEAHRS
#include "Navio2/AHRS.hpp"
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
int pusherpwm;
int configrc;
//Commands created by receiver
float roll_command=0,pitch_command=0,yaw_rx=0,thrust_desired=0;
float mass=1.0;

//PID Loop gains and other stuff for control
float kproll,kdroll,d_roll=0,kppitch,kdpitch,d_pitch=0,d_yaw=0,kipitch,kiroll;
float yawLock = 0;
float perrorIntegral=0,qerrorIntegral=0;
float pitcherrorIntegral=0,rollerrorIntegral=0;
//int top_front_left_pwm = 0;
//int top_front_right_pwm = 0;
//int top_back_right_pwm = 0;
//int top_back_left_pwm = 0;
//int bottom_front_left_pwm = 0;
//int bottom_front_right_pwm = 0;
//int bottom_back_right_pwm = 0;
//int bottom_back_left_pwm = 0;
double MOTORSOUT[8];
double MOTORSOUT_[8];
double inSIG_[8];
double inSIG[8];

#ifdef USERECONFIG
int MotorsSetup() {
  //Add Motors(rx,ry,rz,nx,ny,nz,yaw_dir)
  float rx = (0.0762*METERS2FT); //Meters converted to Feet
  float ry = (0.10115*METERS2FT);
  float rz = (0.0508*METERS2FT);
  //Bottom Motors
  propel.addMotor(rx,-ry,rz,0.0,0.0,-1.0,-1);  //Add an extra variable here 
  propel.addMotor(rx,ry,rz,0.0,0.0,-1.0,1);  //that's a 0 or 1.
  propel.addMotor(-rx,ry,rz,0.0,0.0,-1.0,-1);/// var = 0, motorOff, var = 1, motorOn
  propel.addMotor(-rx,-ry,rz,0.0,0.0,-1.0,1);
  //Top Motors
  propel.addMotor(rx,-ry,-rz,0.0,0.0,-1.0,1); //top_front_left A ROTATION OF 1 = ccw and -1 = CW
  propel.addMotor(rx,ry,-rz,0.0,0.0,-1.0,-1); //top_front_right
  propel.addMotor(-rx,ry,-rz,0.0,0.0,-1.0,1); //top_back_right
  propel.addMotor(-rx,-ry,-rz,0.0,0.0,-1.0,-1); //top_back_left
  //Pusher
  //float rxpusher = 0.1524*METERS2FT; -- Don't do this because it will mess up the controller
  //propel.addMotor(-rxpusher,0.0,0.0,1.0,0.0,0.0,1);

 //Finalize Motor Calcs
  mass = ((735./1000.)*KG2LBF)/GEARTHENG; //grams to kg to lbf to slugs
  MATLAB I;
  I.eye(3,"Inertia");
  float Ixx = (0.045*KG2LBF)*METERS2FT*METERS2FT; //kg-m^2 to lbf-ft^2
  float Iyy = (0.07413*KG2LBF)*METERS2FT*METERS2FT;
  float a = (6.5/100.)*METERS2FT; //cm to meters to feet
  float Izz = Ixx + Iyy - mass*a*a/6.0;
  I.set(1,1,Ixx);
  I.set(2,2,Iyy);
  I.set(3,3,Izz);
  printdouble(mass,"mass");
  I.disp();
  //propel.MotorBeepRPY(mass,I,SERVO_MAX,SERVO_MIN);
  //Need to include throttle from user RollPitchYawThrottle
  propel.MotorBeep(mass,I,SERVO_MAX,IDLE);

  //Do you want to remove some motors? Sure.
  //propel.RemoveMotorsRPY(4); -- This was flight tested on 3/11/2019 and it worked.
  //^^^Code has since changed though see below.
  //propel.RemoveMotorsRPY(2); --- This did not work so Propulsion class was overhauled.

  //This is the new overhauled code here. This has been tested on 3/12/2019 with 8 motors, 4 motors and.....
  propel.RemoveMotors(3); //This will allocated matrices to remove motors

  //If you want to keep using 4 motors then uncomment this otherwise you can change this variable to turn motors on and off
  //propel.MOTORSRUNNING = propel.NUMMOTORS;
  //If you want to turn on motors midway through flight just do this
  //propel.MOTORSRUNNING = propel.NUMMOTORS - propel.MOTORSOFF;  
  
}
#endif

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
  //kp = 3.0;
  //kd = 2.3e-7;
  //ki = 7.1e-8;
  //kyaw = 2.0e-9;
  //^^^No where near enough control authority. No twitching but I can't control it.
  //kd = 2.3e-2;
  //ki = 7.1e-3;
  //kyaw = 2.0e-4;
  ///^^^^Drone was twitchy in roll and pitch - It was controllable kind of. I had no yaw authority.
  //  kd = 2.3e-3;
  //  ki = 7.1e-4;
  //  kyaw = 2.0e-2;
  //gains above are for 4 motors, gains below are for 8
  //kd =  1.15e-3;
  //ki = 3.55e-4;
  //kyaw = 1.0e-2;//kd = N*(kd^2)*ry*kt//
  //^^^Gains above were tested on 3/8/2019 -- still a bit twitchy. Yaw seemed good
  //kp = 2.8;
  //kd = 1.0e-3;
  //ki = 3.2e-4;
  //kyaw = 1.0e-2;
  //^^^Gains above were still twitchy on 3/11/2019
  //kp = 2.8;
  //kd = 1.3e-3;
  //ki = 3.2e-4;
  //Still twitchy. After plotting data it looks like the roll rate filter isn't good enough and the derivative gain was way too high.
  //kp = 2.8;
  //kd = 1.3e-4;
  //ki = 3.2e-4;
  //kyaw = 1.0e-2;
  //System above was uncontrollable. Gains are the same as last time but I changed the filter to 0.35
  //kp = 2.8;
  //kd = 1.3e-3;  0.0013
  //ki = 3.2e-4;  0.00032
  //kyaw = 1.0e-2; 0.01
  ///
  //kp = 2.8;
  //kd = 0.0008;
  //ki = 0.0006;
  //kyaw = 0.01;
  //^^^^Drone flew way better but the data is pretty awful. Let's try lowering kp
  //kp = 2.0;
  //kd = 0.0008;
  //ki = 0.0006;
  //kyaw = 0.01;
  //^^^Kp did nothing. Let's just leave it at like 2.5 but keep lowering kd and increasing ki. Also changed filter to 0.4 from 0.35
  //kp = 2.5;
  //kd = 0.0005;
  //ki = 0.0008;
  //kyaw = 0.01;
  //^^^I didn't have enough control so I'm bringing kp back up and then reducing ki since I think it's too big. It's possible it's the filter
  //Kept the same gains and put the filter back to 0.35
  kp = 3.0;
  kd = 0.0005;
  ki = 0.0007;
  kyaw = 0.01;
  //^^^Alright well the quad still twitches a bit but it's way better than it was before. If I had to guess I would say that we could maybe lower kd a bit more but I'm not sure. That would be my next guess
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
  //propel.computeReconfigurable(d_roll,-d_pitch,d_yaw); -- Code changed. Throttle command sent as thrust input
  propel.computeReconfigurable(thrust_desired/mass,d_roll,-d_pitch,d_yaw);
  //For now we will just hard code the extraction of the motors to get something flying

  //Added this filter here. This is a standard first order filter using a tustin transformation
  //the a here is a pole for the first order transfer function such that G = a/(s+a) thus if you
  //make a bigger the system will filter less but repond faster.
  //In plot_flight_data.py I have a = 3.0 which is a ton of filtering and there's no way that would
  //work in flight. Any filter is better than none though. 
  //So far I have tried 16 and the system was unstable.
  //Trying 25 yielded undesiredable results in the lab but outside on 4/1/2019 and it took off with horrible oscillations. I would say it was marginally stable.
  //Suggest increasing to 35.0
  //Setting the value to 35.0 made the system respond very well. This is still untested with shutting off motors but other than a few oscillations the system
  //did seem a heck of a lot smoother than before. Still not 100% positive how the data looks thought but I suggest plotting output28.txt
  double a = 35.0;
  double tau = 1.0/a;
  double alfa = 2*tau/elapsedTime;
  for (int i = 0;i<=8;i++) {
    //MOTORSOUT[i] = throttle + propel.MOTORS[i].pwm_signal + IDLE - 996; -- This has changed as well
    //inSIG_[i] = inSIG[i]; //Reset old signal
    //inSIG[i] = propel.MOTORS[i].pwm_signal; //Routine now compute actual signal
    //MOTORSOUT_[i] = MOTORSOUT[i];
    //MOTORSOUT[i] = (inSIG[i]+inSIG_[i]-MOTORSOUT_[i]*(1-alfa))/(alfa+1);

    //If you want to get rid of the filter simply comment out the lines above and uncomment the line below
    MOTORSOUT[i] = propel.MOTORS[i].pwm_signal; //Routine now compute actual signal
    
    //bottom_front_left_pwm = throttle + propel.MOTORS[0].pwm_signal + IDLE - 996;
    //bottom_front_right_pwm = throttle + propel.MOTORS[1].pwm_signal + IDLE - 996;
    //bottom_back_right_pwm = throttle + propel.MOTORS[2].pwm_signal + IDLE - 996;
    //bottom_back_left_pwm  = throttle + propel.MOTORS[3].pwm_signal + IDLE - 996;
    //top_front_left_pwm = throttle + propel.MOTORS[4].pwm_signal + IDLE - 996;
    //top_front_right_pwm = throttle + propel.MOTORS[5].pwm_signal + IDLE - 996;
    //top_back_right_pwm = throttle + propel.MOTORS[6].pwm_signal + IDLE - 996;
    //top_back_left_pwm = throttle + propel.MOTORS[7].pwm_signal + IDLE - 996;
  }
  #else
  //top_back_left_pwm =   throttle + d_roll - d_pitch - d_yaw + IDLE - 996;
  //top_back_right_pwm =  throttle - d_roll - d_pitch + d_yaw + IDLE - 996;
  //top_front_right_pwm = throttle - d_roll + d_pitch - d_yaw + IDLE - 996;
  //top_front_left_pwm =  throttle + d_roll + d_pitch + d_yaw + IDLE - 996;
  MOTORSOUT[0] =   throttle + d_roll - d_pitch - d_yaw + IDLE - 996;
  MOTORSOUT[1] =  throttle - d_roll - d_pitch + d_yaw + IDLE - 996;
  MOTORSOUT[2] = throttle - d_roll + d_pitch - d_yaw + IDLE - 996;
  MOTORSOUT[3] =  throttle + d_roll + d_pitch + d_yaw + IDLE - 996;
  #endif

  //Saturation Blocks
  for (int i = 0;i<=8;i++) {
    if (i < propel.MOTORSRUNNING) {
      MOTORSOUT[i] = CONSTRAIN(MOTORSOUT[i],IDLE,SERVO_MAX);
    } else {
      MOTORSOUT[i] = SERVO_MIN;
    }
  }
  //top_front_left_pwm = CONSTRAIN(top_front_left_pwm,IDLE,SERVO_MAX);
  //top_front_right_pwm = CONSTRAIN(top_front_right_pwm,IDLE,SERVO_MAX);
  //top_back_right_pwm = CONSTRAIN(top_back_right_pwm,IDLE,SERVO_MAX);
  //top_back_left_pwm = CONSTRAIN(top_back_left_pwm,IDLE,SERVO_MAX);
  //bottom_front_left_pwm = CONSTRAIN(bottom_front_left_pwm,IDLE,SERVO_MAX);
  //bottom_front_right_pwm = CONSTRAIN(bottom_front_right_pwm,IDLE,SERVO_MAX);
  //bottom_back_right_pwm = CONSTRAIN(bottom_back_right_pwm,IDLE,SERVO_MAX);
  //bottom_back_left_pwm = CONSTRAIN(bottom_back_left_pwm,IDLE,SERVO_MAX);
  
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
  float s = 0.35; //Changed filter from 0.2 to 0.1 on 2/28/2019. Changed to 0.2 again on 3/11/2019 to try and remove some noise. Filtered changed from 0.35 to 0.4 on 3/11/19. Did not like it so changed it back.
  gx_filtered = gx_filtered*s + (1-s)*gy*RAD2DEG;
  gy_filtered = gy_filtered*s + (1-s)*gx*RAD2DEG;
  gz_filtered = gz_filtered*s + (1-s)*(-gz)*RAD2DEG;
}


//=====================================Main Loop Start====================================
int main(int argc, char *argv[])
{

  #ifdef LOGDATA
  //Create OUTPUT Matrix
  //time, throttleRC, rollRC, pitchRC, yawRC, roll,pitch,yaw,gx,gy,gz,9 motor signals
  OUTPUT.zeros(20,1,"OUTPUT MATRIX");
  #endif
  
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
  //double RXCHANNELS[NUM_CHANNELS],RXVALS[NUM_CHANNELS];
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
    return EXIT_FAILURE;
  }
  sleep(1);
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
  float dataTime = beginTime;
  float timeSinceStart = 0;

  printf("Time For take off \n"); //Oh shit.
  sleep(1);
  printf("This shits about to get #nasty #nohomo #shemakemedoit \n");

  printf("But first....wait 10 seconds for the red light \n");

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
	//================make commands===============
	//i need to take the pwm signals and make them angle commands
	//These commands only need to be generated when you poll the receiver
	roll_command = (50.0/500.0)*(float(rollrc)-1500.0);
	pitch_command = (50.0/500.0)*(float(pitchrc)-1500.0);
	yaw_rx = (135.0/500.0)*(float(yawrc)-1500.0); //this is really a yawRate command
	thrust_desired = ((8.0*propel.MOTORS[0].Max_Thrust)/((double)THROTTLE_MAX-(double)THROTTLE_MIN))*(throttle-THROTTLE_MIN);
      }

      //I'm pretty sure this is what gives us the ptp values. Yes it does.
      imuLoop(elapsedTime);

      //In here we can add a switch to turn motors on and off
      if(configrc < 1150) {
	propel.MOTORSRUNNING = propel.NUMMOTORS;
      }else{
	propel.MOTORSRUNNING = propel.NUMMOTORS - propel.MOTORSOFF;
      }
      //================write outputs===============
      if(arm_switch > 1300) {
	led->setColor(Colors::Green);
	//================Controller==============
	//This runs every timestep since imuLoop is running everytimestep
	computeControl(elapsedTime);
	/////////////////////////////////////////
	#ifdef LOGDATA
	if (FILEOPEN == 0) {
	  //The file is not open so open it
	  logger.findfile("/home/pi/FASTPilot/FASTRotor/BumbleBee/Data_Files/output");
	  logger.open();
	  FILEOPEN = 1; //Set this flag to 1 to signify that the file is open
	} //else {
	  //The file is already open so don't do anything?

       } else {
	//Check and see if we need to close the file
	if (FILEOPEN == 1) {
	  FILEOPEN = 0;
	  logger.close();
	}
	#endif
	
	//Otherwise just send servo_min to all motors
       	if (timeSinceStart > 10) {
       	  led->setColor(Colors::Red);
       	}
	//Reset integral error
	pitcherrorIntegral = 0;
	rollerrorIntegral = 0;
	perrorIntegral = 0;
	qerrorIntegral = 0;
	for (int i = 0;i<=8;i++) {
	  MOTORSOUT[i] = SERVO_MIN;
	  MOTORSOUT_[i] = SERVO_MIN;
	  inSIG_[i] = SERVO_MIN;
	  inSIG[i] = SERVO_MIN;
	}
       	//top_front_left_pwm = SERVO_MIN;
       	//top_front_right_pwm = SERVO_MIN;
       	//top_back_right_pwm = SERVO_MIN;
	//top_back_left_pwm = SERVO_MIN;
	//bottom_front_left_pwm = SERVO_MIN;
       	//bottom_front_right_pwm = SERVO_MIN;
       	//bottom_back_right_pwm = SERVO_MIN;
       	//bottom_back_left_pwm = SERVO_MIN;
      } //end of arm switch
      //Send signals to motors
      //carlos came up with the signal naming convention, i think it is bullshit.
      //Collin is right. I'm sorry. It's been changed
      for (int i = 0;i<=8;i++) {
	rcout.set_duty_cycle(i,MOTORSOUT[i]);
      }
      //rcout.set_duty_cycle(0,bottom_front_left_pwm);
      //rcout.set_duty_cycle(1,bottom_front_right_pwm);
      //rcout.set_duty_cycle(2,bottom_back_right_pwm);
      //rcout.set_duty_cycle(3,bottom_back_left_pwm);
      //rcout.set_duty_cycle(4,top_front_left_pwm);
      //rcout.set_duty_cycle(5,top_front_right_pwm);
      //rcout.set_duty_cycle(6,top_back_right_pwm);
      //rcout.set_duty_cycle(7,top_back_left_pwm);
      rcout.set_duty_cycle(8,pusherpwm);

      #ifdef LOGDATA
      if ((((currentTime - dataTime)/1000000.0) > 0.02) && (FILEOPEN == 1)) {
	dataTime = currentTime;
	OUTPUT.set(1,1,currentTime);
	OUTPUT.set(2,1,throttle);
	OUTPUT.set(3,1,rollrc);
	OUTPUT.set(4,1,pitchrc);
	OUTPUT.set(5,1,yawrc);
	OUTPUT.set(6,1,roll);
	OUTPUT.set(7,1,pitch);
	OUTPUT.set(8,1,yaw);
	OUTPUT.set(9,1,gx_filtered);
	OUTPUT.set(10,1,gy_filtered);
	OUTPUT.set(11,1,gz_filtered);
	//OUTPUT.set(12,1,bottom_front_left_pwm);
	//OUTPUT.set(13,1,bottom_front_right_pwm);
	//OUTPUT.set(14,1,bottom_back_right_pwm);
	//OUTPUT.set(15,1,bottom_back_left_pwm);
	//OUTPUT.set(16,1,top_front_left_pwm);
	//OUTPUT.set(17,1,top_front_right_pwm);
	//OUTPUT.set(18,1,top_back_right_pwm);
	//OUTPUT.set(19,1,top_back_left_pwm);
	for (int i = 12;i<20;i++) {
	  OUTPUT.set(i,1,MOTORSOUT[i-12]);
	}
	OUTPUT.set(20,1,pusherpwm);
	logger.println(OUTPUT);
	//printf("Data logged \n");
	//log.flush(); No longer need to flush - arm_switch accomplishes this
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
	printf("%d %d %d %d %d %d",throttle,rollrc,pitchrc,yawrc,configrc,arm_switch);
	//Rec Commands
	printf("%lf ",thrust_desired);
	//PWM outputs
	for (int i = 0;i<=8;i++) {
	  printf("%lf ",MOTORSOUT[i]);
	}
	//printf("%d %d %d %d %d %d %d %d %d",bottom_front_left_pwm,bottom_front_right_pwm,bottom_back_right_pwm,bottom_back_left_pwm,top_front_left_pwm,top_front_right_pwm,top_back_right_pwm,top_back_left_pwm,pusherpwm);
        //#ifdef USERECONFIG
	//for (int i = 0;i<propel.NUMMOTORS;i++) {
	//printf("%d ",(int)propel.MOTORS[i].pwm_signal);
	//}
        //#endif
	//d_ values
	//printf("%+05.2f %+05.2f %+05.2f ",d_roll,d_pitch,d_yaw);
	//command angles	
	//printf("%+05.2f %+05.2f ",roll_command,pitch_command);
	//Newline
	printf("\n");
      }
      #endif

    } //end of void loop

  return 0;
}
