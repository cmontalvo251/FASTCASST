///The Main source files have no header files
//Everything is contained in here

#include <stdio.h>
using namespace std;

//Timer 
#include <Timer/timer.h>
TIMER watch;
#define PRINTRATE 0.1 //Rate of printing to stdout
double lastPRINTtime = 0;

//Demo routine is designed to be as simple as possible so we will just import the IMU
#include <IMU/IMU.h>
IMU orientation;

//RCIO will take care of the receiver and pwm signals
#include <RCIO/RCIO.h>
RCIO rc;

int main(int argc,char* argv[]) {
  printf("FASTKit Demo \n");

  //Pick the IMU you want to use
  //0 = MPU9250
  //1 = LSM9DS1
  orientation.init(0);

  //Set the number of PWM channels
  rc.outInit(4); //4 motors

  //Enter into infinite while loop
  while (1) {

    //Get Current Time and elapsed Time
    double currentTime = watch.getTimeSinceStart();
    double elapsedTime = watch.getTimeElapsed();

    //Read the Receiver Signals
    rc.read();

    //Poll the imu
    orientation.loop(elapsedTime);

    //We will hard code a simple PID loop here

    //First get the receiver signals
    double throttle = rc.in.rx_array[0];
    double aileron = rc.in.rx_array[1];
    double elevator = rc.in.rx_array[2];
    double rudder = rc.in.rx_array[3];
    //Then convert them to roll pitch and yaw commands
    double roll_command = (aileron-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
    double pitch_command = -(elevator-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
    double yaw_rate_command = (rudder-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
    //Get the angle and rate
    double roll = orientation.roll;
    double pitch = orientation.pitch;
    double yaw = orientation.yaw;
    double roll_rate = orientation.roll_rate;
    double pitch_rate = orientation.pitch_rate;
    double yaw_rate = orientation.yaw_rate;
    //Set the gains of the controller
    double kp = 10.0,kd = 2.0,kyaw = 5.0;
    //PID loop
    double droll = kp*(roll-roll_command) + kd*(roll_rate);
    droll = CONSTRAIN(droll,-500,500);
    double dpitch = kp*(pitch-pitch_command) + kd*(pitch_rate);
    dpitch = CONSTRAIN(dpitch,-500,500);
    double dyaw = kyaw*(yaw_rate-yaw_rate_command);
    dyaw = CONSTRAIN(dyaw,-500,500);
    //Mix all three controllers
    double motor_upper_left = throttle - droll - dpitch - dyaw;
    double motor_upper_right = throttle + droll - dpitch + dyaw;
    double motor_lower_left = throttle - droll + dpitch + dyaw;
    double motor_lower_right = throttle + droll + dpitch - dyaw;
    //Send motor signals to array
    rc.out.pwm_array[0] = motor_upper_left;
    rc.out.pwm_array[1] = motor_upper_right;
    rc.out.pwm_array[2] = motor_lower_left;
    rc.out.pwm_array[3] = motor_lower_right;

    //PRINT TO STDOUT
    if (lastPRINTtime < currentTime) {
      lastPRINTtime+=PRINTRATE;
      //Time
      printf("%lf ",currentTime);
      //First 4 receiver signals
      rc.printIn(-4);
      //Print Roll Pitch and Yaw
      printf("%lf %lf %lf ",orientation.roll,orientation.pitch,orientation.yaw);
      //PQR
      printf("%lf %lf %lf ",orientation.roll_rate,orientation.pitch_rate,orientation.yaw_rate);
      //Print PWM Channels
      rc.printOut();
      //Newline
      printf("\n");
    }
  }
}
