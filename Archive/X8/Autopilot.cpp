#include "Autopilot.h"

//Constructor
Autopilot::Autopilot() {

  //Defaults for Bumblebee are set in Propulsion.cpp constructor

  //If you want to change them you need to do so here but you also need to
  //set the appropriate #define in Autopilot.h

  #ifdef BDE

  //////////////Use BDE values////////////////////////
  mass = 50.3/GEARTHENG; //lbf to slugs
  rx = 22.0/12.0; //inches to feet forward from center line
  ry = 22.0/12.0; //inches to feet out from center line
  rz = 13.5/12.0; //inches to feet up from center of mass
  rxpusher = 30.0/12.0; //inches to feet behind center of mass
  //Ixx = 11.169004635155744;
  //Iyy = 3.5094793018757002;
  //Izz = 20.515634020037396;
  Ixx = 15;
  Iyy = 15;
  Izz = 20;
  //These come from data sheets
  //at a signal of
  pwm_datapt = 2016.0; //Max PWM Signal - Microseconds - /home/pi/C/FASTPWM.h
  //thrust is
  Tdatapt = (8004.0/1000)*KG2LBF; //Max Thrust - g to kg to lbf - Motor Data Sheet
  //angular velocity is
  omegaRPMdatapt = 557.52797562; //RPM of Motor at Max Thrust converted to rad/s - Motor Data Sheet
  //Rotor Size
  R = (12.0/12.0); //props radius - inches to feet
  ///////////////////////////////////////////////////
  
  #else

  ////////////Otherwise grab defaults from propulsion class/////////
  //////////////BUMBLEBEE VALUES///////////////////////////////
  Ixx = propel.I_.get(1,1);
  Iyy = propel.I_.get(2,2);
  Izz = propel.I_.get(3,3);
  mass = propel.mass_;
  rx = propel.rx_;
  ry = propel.ry_;
  rz = propel.rz_;
  rxpusher = propel.rxpusher_;
  Tdatapt = propel.Tdatapt_;
  pwm_datapt = propel.pwm_datapt_;
  omegaRPMdatapt = propel.omegaRPMdatapt_;
  R = propel.R_;
  
  #endif

  I.eye(3,"Inertia");
  I.set(1,1,Ixx);
  I.set(2,2,Iyy);
  I.set(3,3,Izz);
  printdouble(mass,"mass");
  I.disp();
  rxyz.zeros(3,1,"motor positions");
  rxyz.set(1,1,rx);
  rxyz.set(2,1,ry);
  rxyz.set(3,1,rz);
  datapts.zeros(3,1,"data points for motors");
  datapts.set(1,1,Tdatapt);
  datapts.set(2,1,pwm_datapt);
  datapts.set(3,1,omegaRPMdatapt);

  //Setup Motors
  propel.MotorsSetup(mass,I,SERVO_MIN,SERVO_MAX,rxyz,rxpusher,datapts,R);

  //This is the new overhauled code here. This has been tested on 3/12/2019 with 8 motors, 4 motors and.....
  propel.RemoveMotors(4); //This will allocated matrices to remove motors

  //If you want to keep using 4 motors then uncomment this otherwise you can change this variable to turn motors on and off
  //If you take a look at Autopilot::loop() you can see that the config switch sets the number
  //of running motors
  //propel.MOTORSRUNNING = propel.NUMMOTORS;
  //If you want to turn on motors midway through flight just do this
  //propel.MOTORSRUNNING = propel.NUMMOTORS - propel.MOTORSOFF;  
  
}

void Autopilot::update(float currentTime,float elapsedTime,int FILEOPEN) {
  //////////////////POLLING IMU AS QUICKLY AS POSSIBLE////////////

  //Filter value is 0.45 for meta but I think it is 0.35 for bumblebee
  //might be different for BDE
  orientation.loop(elapsedTime,0.35); //second number is the filter value
  #ifdef PRINTSEVERYWHERE
  printf("IMU... \n");
  #endif
  ///////////////////////////////////////////////////////////////////
  
}

void Autopilot::loop(double rollrc,double pitchrc, double yawrc,double throttle, double configrc,double pusherpwm,float elapsedTime){

  if(configrc < 1150) {
    propel.MOTORSRUNNING = propel.NUMMOTORS;
  }else{
    propel.MOTORSRUNNING = propel.NUMMOTORS - propel.MOTORSOFF;
  }

  //================make commands===============
  //i need to take the pwm signals and make them angle commands
  //These commands only need to be generated when you poll the receiver
  roll_command = (50.0/500.0)*(float(rollrc)-1500.0);
  pitch_command = (50.0/500.0)*(float(pitchrc)-1500.0);
  yaw_rx = (135.0/500.0)*(float(yawrc)-1500.0); //this is really a yawRate command
  thrust_desired = ((8.0*propel.MOTORS[0].Max_Thrust)/((double)THROTTLE_MAX-(double)THROTTLE_MIN))*(throttle-THROTTLE_MIN);
  
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
  //kp = 3.0;
  //kd = 0.0005;
  //ki = 0.0007;
  //kyaw = 0.01;
  //^^^Alright well the quad still twitches a bit but it's way better than it was before. If I had to guess I would say that we could maybe lower kd a bit more but I'm not sure. That would be my next guess

  //3/10/2020
  //Quad twitch was bothering me. I realized that inertias in propulsion.cpp were off by a factor of 32.2 because I did not convert to slugs-ft^2 but dividing the inertia by 32.2 made the quad uncontrollable.
  //Is it possible then to leave the 32.2 to convert to the right units and then multiply these PD gains by 32.2

  //Note that dividing the inertia by 32.2 got rid of the high frequency oscillations. So I took the inertia and multiplied by 20 which is not quite 32.2 to make the quad controllable but still getting rid of high frequency
  //oscillations.

  //My recommendation if you have time is to get rid of the *20 in the inertia calc and multiply the gains here by 20 to see if that fixes anything.
  //One issue that may come up is the CONSTRAIN commands below. If kp = 3*20 = 60 so 250/60 = 4.1 which means we would saturate the controller at 4.1 degrees.
  //From looking at the control scheme below it looks like you would just need to multiple kd and ki by 20.
  //Need to rederive the controller to make sure. Can do this in nonlinear controls on Tuesday.

  //So I just rederived the controller and sure enough if you want to lower the inertias you need to multiply the gains.
  //So if you get rid of the *20 in the propulsion class you need to mulitply kd and ki by 20 which would be

  //kp = 3.0;
  //kd = 0.01; //0.0005*20;
  //ki = 0.014; //0.0007*20;
  //kyaw = 0.01; //Leave kyaw as is for now but you may need to increase by 20 as well (0.01*20 = 0.2)

  //Ok flight test on 3/11/2020 went ok. Some wierd oscillations popped up in the roll channel.
  //I'm gonna go ahead and change kd and ki and change the inertias to remove the *20.
  //kp = 3.0;
  //kd = 0.01;
  //ki = 0.014;
  //kyaw = 0.01;

  //Yaw gain needs an increase of *20 as wel
  //problem is the roll channel still oscillated like crazy. 
  //kp = 3.0;
  //kd = 0.01;
  //ki = 0.014;
  //kyaw = 0.2;

  //Yaw seems good but drone just seems unresponsive. I did change roll and pitch inertias to be the same and rx and ry to be the same at least for bumblebee. so let's just put the gains back to the *32.2 factor.
  //Yaw is good just leave it.
  //kp = 3.0;
  //kd = 0.016;
  //ki = 0.0225;
  //kyaw = 0.2;

  //Drone seemed alot better but the battery died and I couldn't take off. So I'd say the drone flies but it's still not 100%. Need to charge a battery and try again. You could probably up the gains by another 10% if you wanted
  //kp = 3.0;
  //kd = 0.018;
  //kd = 0.025;
  //kyaw = 0.3; //go ahead and put yaw at 0.3 (0.01*32.2)

  //Alright so I actually simulated this on SUAM and it turns out that ki is too high which is what's causing the horrible oscillations. So I have two ideas.
  //One is to lower ki by a factor of 10 or increase kd by a factor of 10. Increasing yaw is totally fine and increasing kp is fine as well but my vote is kd or ki

  //Lower ki by a factor of 10. Increase kyaw to 0.3

  //Ok turns out ki was too high. The autopilot routine in X8 and SUAM were actually different so I moved the autopilot routine from X8 over to SUAM and the system was completely
  //unstable. It means that we had an unstable pole. I increased kd by a factor of 10 and that fixed it. I also lowered ki by a factor of 10 and that fixed it too. I like lowering gains
  //more than increasing them on hardware because of noise issues so I tested the drone on 3/13/2020 and it flew great. Here are the gains I used.
  kp = 3.0;
  kd = 0.016;
  ki = 0.00225;
  kyaw = 0.3;
  
  #endif
  
  //Using Control Architecture from KyrellGod
  //Compute outer loop commands
  //2/26/2019 - twichiness so lowered from 4.5
  float phidot_command = kp*(roll_command - orientation.roll);
  float thetadot_command = kp*(pitch_command - orientation.pitch);

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
  float cos_theta = cos(orientation.pitch*DEG2RAD);
  float sin_theta = sin(orientation.pitch*DEG2RAD);
  float cos_phi = cos(orientation.roll*DEG2RAD);
  float sin_phi = sin(orientation.roll*DEG2RAD);
  float p_command = phidot_command - sin_theta*psidot_command;
  float q_command = cos_phi*thetadot_command + sin_phi*cos_theta*psidot_command;
  float r_command = -sin_phi*thetadot_command + cos_phi*cos_theta*psidot_command;

  //We then compute the inner loop commands which is a PI controller except for yaw
  float perror = (p_command - orientation.roll_rate);
  float qerror = (q_command - orientation.pitch_rate);
  float rerror = (r_command - orientation.yaw_rate);

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
  float roll_error = roll_command - orientation.roll;
  rollerrorIntegral+=roll_error*elapsedTime;
  rollerrorIntegral = CONSTRAIN(rollerrorIntegral,-50.0,50.0);
  float kp_roll_error = kproll*roll_error;
  kp_roll_error = CONSTRAIN(kp_roll_error,-250.0,250.0); //May need this
  d_roll = kp_roll_error - kdroll*orientation.roll_rate + kiroll*rollerrorIntegral;
  
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
  float pitch_error = pitch_command - orientation.pitch;
  pitcherrorIntegral+=pitch_error*elapsedTime;
  pitcherrorIntegral = CONSTRAIN(pitcherrorIntegral,-50.0,50.0);
  float kp_pitch_error = kppitch*pitch_error;
  kp_pitch_error = CONSTRAIN(kp_pitch_error,-250.0,250.0); //May need this
  d_pitch = kp_pitch_error - kdpitch*orientation.pitch_rate + kipitch*pitcherrorIntegral;
  
  //==================yaw====================
  //kyaw = 0.8;
  //The gains above are from guess and check but the gain below is from adapting from KyrellGod's code
  float kyaw = 1.0;
  d_yaw = kyaw*(yaw_rx - orientation.yaw_rate);
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
    
  }
  #else
  MOTORSOUT[0] =   throttle + d_roll - d_pitch - d_yaw + IDLE - SERVO_MIN;
  MOTORSOUT[1] =  throttle - d_roll - d_pitch + d_yaw + IDLE - SERVO_MIN;
  MOTORSOUT[2] = throttle - d_roll + d_pitch - d_yaw + IDLE - SERVO_MIN;
  MOTORSOUT[3] =  throttle + d_roll + d_pitch + d_yaw + IDLE - SERVO_MIN;
  #endif

  //Saturation Blocks
  for (int i = 0;i<=8;i++) {
    if (i < propel.MOTORSRUNNING) {
      MOTORSOUT[i] = CONSTRAIN(MOTORSOUT[i],IDLE,SERVO_MAX);
    } else {
      MOTORSOUT[i] = SERVO_MIN;
    }
  }
  pusherout = pusherpwm;
}
  
void Autopilot::reset() {
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
  pusherout = SERVO_MIN;
}
