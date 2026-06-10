#include "Autopilot.h"

//Constructor
Autopilot::Autopilot() {
  //Estimate the mass of the vehicle
  mass = ((735./1000.)*KG2LBF)/GEARTHENG; //grams to kg to lbf to slugs
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

  //Setup Motors
  propel.MotorsSetup(mass,I,SERVO_MAX,IDLE);

  //Do you want to remove some motors? Sure.
  //propel.RemoveMotorsRPY(4); -- This was flight tested on 3/11/2019 and it worked.
  //^^^Code has since changed though see below.
  //propel.RemoveMotorsRPY(2); --- This did not work so Propulsion class was overhauled.

  //This is the new overhauled code here. This has been tested on 3/12/2019 with 8 motors, 4 motors and.....
  propel.RemoveMotors(3); //This will allocated matrices to remove motors

  //If you want to keep using 4 motors then uncomment this otherwise you can change this variable to turn motors on and off
  //If you take a look at Autopilot::loop() you can see that the config switch sets the number
  //of running motors
  //propel.MOTORSRUNNING = propel.NUMMOTORS;
  //If you want to turn on motors midway through flight just do this
  //propel.MOTORSRUNNING = propel.NUMMOTORS - propel.MOTORSOFF;  
  
}

void Autopilot::update(float currentTime,float elapsedTime,int FILEOPEN) {
  //////////////////POLLING IMU AS QUICKLY AS POSSIBLE////////////
  orientation.loop(elapsedTime);
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
  kp = 3.0;
  kd = 0.0005;
  ki = 0.0007;
  kyaw = 0.01;
  //^^^Alright well the quad still twitches a bit but it's way better than it was before. If I had to guess I would say that we could maybe lower kd a bit more but I'm not sure. That would be my next guess
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

  pusherout = pusherpwm;
  
  //top_front_left_pwm = CONSTRAIN(top_front_left_pwm,IDLE,SERVO_MAX);
  //top_front_right_pwm = CONSTRAIN(top_front_right_pwm,IDLE,SERVO_MAX);
  //top_back_right_pwm = CONSTRAIN(top_back_right_pwm,IDLE,SERVO_MAX);
  //top_back_left_pwm = CONSTRAIN(top_back_left_pwm,IDLE,SERVO_MAX);
  //bottom_front_left_pwm = CONSTRAIN(bottom_front_left_pwm,IDLE,SERVO_MAX);
  //bottom_front_right_pwm = CONSTRAIN(bottom_front_right_pwm,IDLE,SERVO_MAX);
  //bottom_back_right_pwm = CONSTRAIN(bottom_back_right_pwm,IDLE,SERVO_MAX);
  //bottom_back_left_pwm = CONSTRAIN(bottom_back_left_pwm,IDLE,SERVO_MAX);
  
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
