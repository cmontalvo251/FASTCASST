#include "Controller.h"
#include "MATLAB.h"
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "mathp.h"

using namespace std;

//Constructor
Controller::Controller()
{
  controls.zeros(4,1,"Controls");
  controls.set(4,1,THROTTLE_MIN);
}

void Controller::print_to_file(FILE* outfile) {
  //Print control values
  for(int i = 1;i<=4;i++){
    fprintf(outfile,"%lf ",controls.get(i,1)); //de,da,dr,mewt
  }
}

void Controller::read1Row() {
  t0 = tf;
  dele0 = delef;
  dela0 = delaf;
  delr0 = delrf;
  mewt0 = mewtf;
  measured_state0 = measured_statef;
  fscanf(infile,"%lf %lf %lf %lf %lf %lf \n",&tf,&delef,&delaf,&delrf,&mewtf,&measured_statef);
  //printf("%lf %lf %lf %lf %lf \n",tf,delef,delaf,delrf,mewtf);
}

void Controller::read2Rows() {
  read1Row();
  read1Row();
}

void Controller::setup() {
  if (CONTROLTYPE == 2) {
    infile = fopen("Input_Files/A++.CTL","r"); //hard code for now because I'm lazy
    if (infile) {
      read2Rows();
      //printf("Read 2 Rows \n");
    } else {
      //printf("Could not Find Input_Files/A++.CTL");
      exit(1);
    }
  }
}

void Controller::printcost() {
  //cout << "Final Cost = " << cost << endl;
  FILE* outfile = fopen("Output_Files/A++.COST","w");
  fprintf(outfile,"%lf \n",cost);
}

void Controller::compute(double time,MATLAB state,MATLAB dxdt,double dt) {

  //First Zero out controls
  controls.mult_eq(0);
  controls.set(4,1,THROTTLE_MIN); //except for motor signal

  //Call the Control routine
  if (CONTROLTYPE == 1) {
    //Run the PID Routine
    InnerLoopOuterLoop(state,dxdt,dt);
  } else if (CONTROLTYPE == 2) {
    //Then check that we have the right limits
    while (time > tf) {
      //printf("time = %lf \n",time);
      read1Row();
    }
    //When this while loop break t0,time,tf should all be sequential
    //printf("%lf %lf %lf \n",t0,time,tf);
    //So we can interpolate now
    double pt_slope = (1.0/(tf-t0))*(time-t0);
    dele = PWM2RAD((delef-dele0)*pt_slope + dele0,SERVO_MIN,SERVO_MAX,SERVO_MID,MAX_ANGLE_ELEVATOR);
    dela = PWM2RAD((delaf-dela0)*pt_slope + dela0,SERVO_MIN,SERVO_MAX,SERVO_MID,MAX_ANGLE_AILERON);
    delr = PWM2RAD((delrf-delr0)*pt_slope + delr0,RUDDER_SERVO_MIN,RUDDER_SERVO_MAX,RUDDER_SERVO_MID,MAX_ANGLE_RUDDER);
    mewt = (mewtf-mewt0)*pt_slope + mewt0;

    //This is for our cost function computation
    double measured_state = (measured_statef-measured_state0)*pt_slope + measured_state0;
    //double sim_state = state.get(11,1); //Really I should make this number 10 a parameter that I import but I'll just hardcode it for now
    //double sim_state = state.get(10,1); //For roll rate
    double sim_state = state.get(12,1); //For Yaw Rate
    // printf("%lf %lf %lf \n",time,measured_state,sim_state);
    cost+= pow(measured_state-sim_state,2);

    // double pwm = SERVO_MAX;
    // double rad = PWM2RAD(pwm,SERVO_MIN,SERVO_MAX,SERVO_MID,MAX_ANGLE_ELEVATOR);
    // double pwm2 = RAD2PWM(rad,SERVO_MIN,SERVO_MAX,SERVO_MID,MAX_ANGLE_ELEVATOR);

    // printf("%lf %lf %lf \n",pwm,rad,pwm2);
    
    //printf("%lf %lf %lf %lf %lf %lf \n",t0,time,tf,mewt0,mewt,mewtf);
    controls.set(1,1,dele);
    controls.set(2,1,dela);
    controls.set(3,1,delr);
    controls.set(4,1,mewt);
  } else if (CONTROLTYPE == 3) {
    //Constant control inputs
    
    //controls.set(1,1,MAX_ANGLE_ELEVATOR); //Maximum positive elevator deflection
    //controls.set(2,1,MAX_ANGLE_AILERON); //Maximum positive aileron deflection
    //controls.set(3,1,-MAX_ANGLE_RUDDER); //Rudder
    controls.set(4,1,THROTTLE_MIN);
  }    
}

void Controller::AntiWindup(){
  uint = 0;
}

void Controller::InnerLoop(double roll_command,double pitch_command,double u_command,double roll,double pitch,double u,double p,double q,double r,double dt) {

  //////////////////////ELEVATOR CONTROL/////////////////////////////////////
  
  //double Kpe = -1.0; //Tested this on 06-22-19. Flight was semi-stable. It had some oscillations but did not flip upside down.
  //double Kde = -0.25; //Tested this on 06-22-19. ^^^^^^
  //double Kpe = -0.8; //Reduced previous gains by 20% to try to get more stable flight with little to no oscillations.
  //double Kde = -0.2; //^^^^^^
  //double Kpe = -1.0; //The elevator was good and not twitchy but I could use some more authority. So suggest increasing kp but keeping kd constant.
  //double Kde = -0.2; //Tested on 06-28-19.

  ///////////////FINAL GAINS///////////////////  
  double Kpe = -0.80; //Elevator was twitchy during flight on 06-28-19. Reduced derivative gain by 20% again.
  double Kde = -0.09;
  ////////////////////////////////////////////

  dele = Kpe*(pitch_command-pitch)+Kde*(0-q); //the 0 means the rate command is zero

  /////////////////////AILERON CONTROL////////////////////////////////////////
  

  //double Kpa = 1.0; //Tested this on 06-22-19. Flight was semi-stable. It had some oscillations but did not flip upside down.
  //double Kdp = 0.25; //Tested this on 06-22-19. ^^^^^^
  //double Kpa = -1.0; //Tested this on 06-22-19. Flight was semi-stable. It had some oscillations but did not flip upside down.
  //double Kdp = -0.25; //Tested this on 06-22-19. ^^^^^^
  //double Kpa = -0.8; //Reduced previous gains by 20% to try to get more stable flight with little to no oscillations.
  //double Kdp = -0.2; //^^^^^^
  //double Kpa = -0.8; //^^Gains above tested on 6/28 and roll was still twitchy. Response to a step was slow however so I suggest keeping Kp the same and reducing Kde.
  //double Kdp = -0.1; //Tested on 06-28-19.
  //double Kpa = -1.0; //Response to a step was still slow to respond on flight test 06-28-19. Increased proportional gain by 20% to speed up response.
  //double Kdp = -0.1; ^^^^^^^^^^^^^
  //double Kpa = 1.0; //Tested on 08-08-19. Flight was twitchy and semi-stable. Going to reduce the roll gains by 20% to see if this will get us stable flight. Will test 08-15-19.
  //double Kdp = 0.1; //Signs were flipped to agree with sign convention

  /////////////////?FINAL GAINS///////////////////////
  double Kpa = 0.50;
  double Kdp = 0.05;
  ///////////////////////////////////////////////
  
  dela = Kpa*(roll_command-roll)+Kdp*(0-p); //the 0 just means that the rate command is zero

  ///////////////////////RUDDER CONTROL//////////////////////////////////////
  
  //double Kv = 2.0; //#Guess and check -- after performing a bench test on 7/17/2019 this is too big
  //double Kv = 1.0; //After performing a ground test on 7/29/2019 this is too big
  //double Kv = 0.25; //
  //delr = Kv*r;

  //Instead of feeding yaw rate we really need to feed sideslip. Since we can't get that value with our hardware
  //We will perform an open loop manuever
  // roll_command -> delr
  double rudder_mixing = -0.2; //This is about to be flight tested on 8/8/2019
  double delr = rudder_mixing*roll_command;

  //printf("%lf %lf %lf \n",roll_command,rudder_command,delr);
  
  //////////////////THROTTLE CONTROL///////////////////////////////////////

  double Kpt = 75.0; //Will test this on 07-11-19.
  //double Kit = 1.0; //Ground test on 7/11/2019
  
  /////////////////FINAL GAIN//////////////////
  double Kit = 2.0; //Ground test on 7/29/2019
  ///////////////////////////////////////////
  
  double uerror = -99;

  if (u_command != -99) {
    uerror = (u_command - u);
    uint += Kit*uerror*dt;
    mewt = Kpt*uerror + SERVO_MIN + uint;
  }  else {
    AntiWindup();
    mewt = SERVO_MAX; 
  }

  //////////////////CHECK FOR LIMITS////////////////////////////////////

  SaturationBlock();

  ///////////////////DEBUGGGING///////////////////////////////

  //printf("%lf %lf %lf %lf \n",roll_command,roll,p,dela);
  //printf("%lf %lf %lf %lf \n",pitch_command,pitch,q,dele);

  ////////////////////SET CONTROLS////////////////////////

  controls.set(1,1,dele);
  controls.set(2,1,dela);
  controls.set(3,1,delr);
  controls.set(4,1,mewt);
}

double Controller::RAD2PWM(double angle,double min,double max,double mid,double angle_limit) {
  //Take an angle in degrees and convert to microsecond signal
  return (angle*(max-min)/(2.0*angle_limit)+mid); //Convert from radians to microsecond pulse
  //I was very tired when I was editing this code. It turns out the line above works but the line below also works
  //Notice the two is removed but instead of max-min it's max-mid essentially taking advantage of the symmetry
  //return (angle*(max-mid)/(angle_limit)+mid); //Convert from radians to microsecond pulse
}

double Controller::PWM2RAD(double signal,double min,double max,double mid,double angle_limit) {
  //Take an signal in microsecond and convert to angle in degrees
  return (signal - mid)*2.0*angle_limit/(max-min);
  //Similary you can use the line above or you can use the line below again taking advantage of symmetry
  //I ended up having a typo in one of my python scripts and I spend probable an hour just trying to fix it and ended
  //up stumbling upon the 2*angle_limit solution. It wasn't until right before I went to bed that I realized that since the slope is constant
  //angle_limit/(max-mid) is equivalent to 2*angle_limit/(max-min). Anyway. For now I will use the 2*angle_limit but just know you can do it
  //a different way
  //return (signal - mid)*angle_limit/(max-mid);
} 

void Controller::InnerLoopOuterLoop(MATLAB state,MATLAB dxdt,double elapsedTime) {

  //////////////////EXTRACT STATES//////////////////////////////
  
  double z = state.get(3,1);
  double phi = state.get(4,1);
  double theta = state.get(5,1);
  double u = state.get(7,1);
  double zdot = dxdt.get(3,1);
  double p = state.get(10,1);
  double q = state.get(11,1);
  double r = state.get(12,1);

  //////////////////ALTITUDE CONTROL///////////////////////
  double zc = -110.0;
  double zdotc = 0.;

  /////////////////PITCH CONTROL/////////////////////////

  //If z = -100
  // zc = -120
  // z - zc = -100 - - 120 = -100 + 120 = 20
  
  double pitch_command = 0.5*(z-zc) + 0.2*(zdot-zdotc);
  //pitch_command = CONSTRAIN(pitch_command,-MAX_ANGLE_PITCH,MAX_ANGLE_PITCH);
  //pitch_command = 2*PI/180.0;
  //printf("Pitch Command = %lf \n",pitch_command);

  /////////////////ROLL CONTROL////////////////////////

  double roll_command = 0.; //#I want wings level
  //roll_command = 10*PI/180.0;

  ////////////////VELOCITY CONTROL/////////////////////
  
  double u_command = 12;

  /////////////////INNER LOOP////////////////////////////
  InnerLoop(roll_command,pitch_command,u_command,phi,theta,u,p,q,r,elapsedTime);
}

void Controller::SaturationBlock() {
  mewt = CONSTRAIN(mewt,THROTTLE_MIN,THROTTLE_MAX);
  //cout << "mewt = " << mewt << endl;
  dele = CONSTRAIN(dele,-MAX_ANGLE_ELEVATOR,MAX_ANGLE_ELEVATOR);
  dela = CONSTRAIN(dela,-MAX_ANGLE_AILERON,MAX_ANGLE_AILERON);
  delr = CONSTRAIN(delr,-MAX_ANGLE_RUDDER,MAX_ANGLE_RUDDER);
}
