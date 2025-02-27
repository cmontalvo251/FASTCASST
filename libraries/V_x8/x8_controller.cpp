//This is an x8 quad with 4 rotors on top and 4 rotors on bottom

#include "x8_controller.h"

////////////////////////////////////////////////////////////////Start Motor Class////////////////////////////////////////////////////////////////
//Motor class constructor 
Motor::Motor() {
    //Initialize vectors
    r.zeros(3, 1, "r vector");
    n.zeros(3, 1, "n vector");
    Tn.zeros(3, 1, "Thrust vector");
    datapts.zeros(3, 1, "Motor data points");

    //Rotor radius - from Propulsion constructor in Propulsion.cpp in SUAM
    R = (5.0 / 12.0) * FT2M;
}

//Function to calculate coefficient of thrust and torque coefficient
void Motor::MotorCalcs(double Tdatapt, double pwm_datapt, double omegaRPMdatapt) {
    //Compute Kt
    float dpwm = pwm_datapt - OUTMIN;
    kt = Tdatapt / (dpwm * dpwm);

    //Angular Velocity computation
    a = (omegaRPMdatapt * 2 * PI / 60.0) / dpwm;

    //Constants
    rho = 1.225;     //kg/m^3
    A = PI * R * R;  //m^2

    //Compute ct and cq
    ct = kt / (rho * A * R * R * a * a); //.0335?
    cq = sqrt(ct * ct * ct) / sqrt(2);   //lbf-s/ft^2 /.004?

    //Set controller ct and cq to calculated values?

    //Debug prints
    //printf("Ct and Cq = %lf %lf \n",ct,cq);
    //printf("pwm_datapt and ct = %lf %lf \n", pwm_datapt, ct);
}

//Function to compute pwm signal based on thrust
double Motor::compute_signal_NO_OMEGA(double thrust_req) {
    //Assume Thrust = kt*(pwm_signal - MINSIGNAL)**2
    double pwm_req = sqrt(thrust_req / kt) + OUTMIN;
    return pwm_req;
}

//Function to compute thrust based on pwm
void Motor::compute_thrust_NO_OMEGA() {
    thrust = kt * (pwm_signal - OUTMIN) * (pwm_signal - OUTMIN);
    Tn.mult(n, thrust);
}

//Function to compute torque based on thrust
void Motor::compute_torque_NO_OMEGA() {
    // printdouble(thrust,"thrust");
    // printdouble(R,"R");
    // printdouble(cq,"cq");
    // printdouble(ct,"ct");
    torque = thrust * R * cq / ct;
}
/////////////////////////////////////////////////////////////////End Motor Class/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////Start Controller Class//////////////////////////////////////////////////////////////
//controller class constructor - sets system parameters and sets up motors
controller::controller() {
    //Mass (from Propulsion constructor in Propulsion.cpp in SUAM)
    mass = 0.735;

    //Inertia (from Propulsion constructor in Propulsion.cpp in SUAM)
    Ixx = 0.07413; //Autopilot.cpp says 0.045
    Iyy = Ixx;     //Autopilot.cpp says 0.07413
    Izz = Ixx + Iyy - mass * 0.65 * 0.65 / 6;

    //Coefficient of thrust (from MotorCalcs function in Motor.cpp in SUAM)
    ct = .0335; //Paper says 0.011321 - Call from motor?

    //Torque coefficient (from MotorCalcs function in Motor.cpp in SUAM)
    cq = .004; //Paper says 0.00085 - Call from motor?

    //Prop Radius (from Propulsion constructor in Propulsion.cpp in SUAM)
    Rrotor = (5.0 / 12.0) * FT2M;

    //r vector components (from Propulsion constructor in Propulsion.cpp in SUAM)
    rx = 0.10115;
    ry = rx;
    rz = 0.0508;

    //From Propulsion constructor in Propulsion.cpp from SUAM
    //These come from data sheets
    //at a signal of
    pwm_datapt_ = 1766.0;
    //thrust is
    Tdatapt_ = (5.0 / GEARTH); //Newtons to kg
    //angular velocity is
    omegaRPMdatapt_ = 3500.0;
    
    //Wrap defaults into matlab vector
    datapts.zeros(3, 1, "Data points for motors");
    datapts.set(1, 1, Tdatapt_);
    datapts.set(2, 1, pwm_datapt_);
    datapts.set(3, 1, omegaRPMdatapt_);

    //Compute nominal and max thrusts and configuration matrix
    MotorsSetup(datapts); 

    //Test remove motor - remove from recent
    RemoveMotors(2);
    
    //Remove specific number of motors
    motorsToRemove = 4;

    //Initialize MOTORSOFF vector for removing specific motors
    REMOVEMOTORS.zeros(motorsToRemove, 1, "Vector of motors to remove");

    //Remove input remove array for left side
    REMOVEMOTORS.set(1, 1, 1); //top_front_left
    REMOVEMOTORS.set(2, 1, 4); //top_back_left
    REMOVEMOTORS.set(3, 1, 5); //bottom_front_left
    REMOVEMOTORS.set(4, 1, 8); //bottom_back_left
    
    //Test remove motor - remove specific
    //RemoveMotors(motorsToRemove, REMOVEMOTORS);
    
};

//Function to add a motor to system
void controller::addMotor(double rx, double ry, double rz, double nx, double ny, double nz, int direction, int op) {
    //Increment the number of motors by 1
    NUMMOTORS++;

    //Set motor position vector
    MOTORS[NUMMOTORS - 1].r.set(1, 1, rx);
    MOTORS[NUMMOTORS - 1].r.set(2, 1, ry);
    MOTORS[NUMMOTORS - 1].r.set(3, 1, rz);

    //Set motor normal vector
    MOTORS[NUMMOTORS - 1].n.set(1, 1, nx);
    MOTORS[NUMMOTORS - 1].n.set(2, 1, ny);
    MOTORS[NUMMOTORS - 1].n.set(3, 1, nz);

    //Set motor spin direction
    MOTORS[NUMMOTORS - 1].dir = direction;

    //Set motor pwm placeholder to minimum output
    MOTORS[NUMMOTORS - 1].pwm_signal = OUTMIN;

    //Set whether motor is ON or OFF
    MOTORS[NUMMOTORS - 1].op_flag = op;

    //Show that motor was added
    //cout << "Motor " << NUMMOTORS << " Set" << endl;

    // Debug prints
    // MOTORS[NUMMOTORS-1].r.disp();
    // MOTORS[NUMMOTORS-1].n.disp();
}

//Function to compute configuration matrix [Q]
void controller::MotorBeep(MATLAB datapts_IN) {
    //Compute weight of drone
    double weight = GEARTH * mass;

    //Compute thrust required for each motor
    double nominal_thrust = weight / (double)NUMMOTORS;

    //Pull out values for motors
    Tdatapt_ = datapts_IN.get(1, 1);
    pwm_datapt_ = datapts_IN.get(2, 1);
    omegaRPMdatapt_ = datapts_IN.get(3, 1);

    //Debug prints
    //cout << "weight = " << weight << " mass = " << mass << endl;
    //cout << "thrust req = " << thrust_req << endl;

    //Compute max thrust - number of motors ON to total number of motors
    MOTORSRUNNING = NUMMOTORS;

    //Set Nominal Thrust of all motors
    for (int i = 0; i < NUMMOTORS; i++) {
        //Re run motor calcs based on new values
        MOTORS[i].MotorCalcs(Tdatapt_, pwm_datapt_, omegaRPMdatapt_);
        MOTORS[i].Nominal_Thrust = nominal_thrust;
        MOTORS[i].pwm_nominal = MOTORS[i].compute_signal_NO_OMEGA(nominal_thrust);

        //This computes Max Thrust so you have two data points
        MOTORS[i].pwm_signal = OUTMAX;
        MOTORS[i].compute_thrust_NO_OMEGA();
        MOTORS[i].Max_Thrust = MOTORS[i].thrust;
        MOTORS[i].pwm_signal = OUTMIN;
    }

    //Debug print statements
    //printf("Max thrust and nominal thrust: %lf %lf \n", MOTORS[0].Max_Thrust, nominal_thrust);

    //This is where we make our configuration matrices
    H.zeros(4, NUMMOTORS, "Configuration Matrix");
    HT.zeros(NUMMOTORS, 4, "Configuration Matrix Transposed");
    HHT.zeros(4, 4, "HHT");
    HHT_inv.zeros(4, 4, "HHT_inv");
    Q.zeros(NUMMOTORS, 4, "Control Matrix");
    M.zeros(4, 4, "Mass Matrix");
    M.set(1, 1, mass);
    M.set(2, 2, Ixx);
    M.set(3, 3, Iyy);
    M.set(4, 4, Izz);
    U.zeros(4, 1, "Gamma Matrix");
    CHI.zeros(NUMMOTORS, 1, "Thrust Required");

    //The controller works like this
    //M*U = H*CHI -- H is a 4xN thus there is no unique solution
    //thus you must use Lagranges Method -- Q = H'*inv(H*H')*M
    //then CHI = Q*U where U is our desired values


    for (int i = 1; i <= NUMMOTORS; i++) {
        //From paper, H = [-1 -ryi rxi (sigmai * cq * Rrotor / ct)]^T
        H.set(1, i, -1.0); 
        H.set(2, i, -MOTORS[i - 1].r.get(2, 1));
        H.set(3, i, MOTORS[i - 1].r.get(1, 1));
        H.set(4, i, (MOTORS[i - 1].dir * cq * Rrotor / ct));
    }
    //HT*inv(HHT)*M
    //H = 4xN
    //HT = Nx4
    //HHT = 4x4
    HT.transpose_not_square(H);
    HHT.mult(H, HT);

    //M.disp();
    //H.disp();
    //HT.disp();
    //HHT.disp();

    //Shit. Need a 4x4 inverse routine
    //Alright added crazy shit inverse functions
    HHT_inv.matrix_inverse(HHT, 4);

    //HHT_inv.disp();

    //Then we need HT*inv(HHT) == N-1 x 4 - 4 x 4
    HT_inv_HHT.zeros(NUMMOTORS, 4, "HT_inv_HHT");
    HT_inv_HHT.mult(HT, HHT_inv);
    //HT_inv_HHT.disp();

    //Finally we can get Q
    Q.mult(HT_inv_HHT, M);
    //Q.disp();
}

//Function to add the 8 motors of BumbleBee
void controller::MotorsSetup(MATLAB datapts_IN) {
    //Top Motors
    addMotor(rx, -ry, -rz, 0.0, 0.0, -1.0, 1, 1);   //Motor 1 - upper_left_top 
    addMotor(rx, ry, -rz, 0.0, 0.0, -1.0, -1, 1);   //Motor 2 - upper_right_top
    addMotor(-rx, ry, -rz, 0.0, 0.0, -1.0, 1, 1);   //Motor 3 - lower_right_top
    addMotor(-rx, -ry, -rz, 0.0, 0.0, -1.0, -1, 1); //Motor 4 - lower_left_top

    //Bottom Motors
    addMotor(rx, -ry, rz, 0.0, 0.0, -1.0, -1, 1);   //Motor 5 - upper_left_bottom
    addMotor(rx, ry, rz, 0.0, 0.0, -1.0, 1, 1);     //Motor 6 - upper_right_bottom
    addMotor(-rx, ry, rz, 0.0, 0.0, -1.0, -1, 1);   //Motor 7 - lower_right_bottom
    addMotor(-rx, -ry, rz, 0.0, 0.0, -1.0, 1, 1);   //Motor 8 - lower_left_bottom

    //Debug print
    //for (int i = 1; i <= NUMMOTORS; i++) {
    //    printf("Motor %i op_flag %i \n", i, MOTORS[i - 1].op_flag);
    //}

    //Finalize motor calcs
    MotorBeep(datapts_IN);
}

//Function to remove set number of motors from most recent to oldest
void controller::RemoveMotors(int removemotors) {

    //Increase MOTORSOFF
    MOTORSOFF = removemotors;

    //With motors removed we need to reinitialize all the control matrices
    MOTORSRUNNING = NUMMOTORS - MOTORSOFF;

    //Set operational flag = 0 for turned off motors
    for (int i = NUMMOTORS; i > MOTORSRUNNING; i--) {
        MOTORS[i - 1].op_flag = 0;
    }

    //Indicator statement
    printf("Total Motors = %i, Removing %i motors. Running Motors = %i \n", NUMMOTORS, removemotors, MOTORSRUNNING);

    //This is where we make our configuration matrices
    Hprime.zeros(4, MOTORSRUNNING, "Configuration Matrix Prime");
    HTprime.zeros(MOTORSRUNNING, 4, "Configuration Matrix Transposed Prime");
    HHTprime.zeros(4, 4, "H*HT Prime");
    HHT_invprime.zeros(4, 4, "Inv(HHTprime)");
    HT_inv_HHTprime.zeros(MOTORSRUNNING, 4, "HT_inv_HHTprime");
    Qprime.zeros(MOTORSRUNNING, 4, "Control Matrix Prime");
    CHIprime.zeros(MOTORSRUNNING, 1, "Thrust Required Prime");

    for (int i = 1; i <= MOTORSRUNNING; i++) {
        //From paper, H = [-1 -ryi rxi (sigmai * cq * Rrotor / ct)]^T
        Hprime.set(1, i, -1.0);
        Hprime.set(2, i, -MOTORS[i - 1].r.get(2, 1));
        Hprime.set(3, i, MOTORS[i - 1].r.get(1, 1));
        Hprime.set(4, i, (MOTORS[i - 1].dir * cq * Rrotor / ct));
    }

    //HT*inv(HHT)*M
    //H = 4xN
    //HT = Nx4
    //HHT = 4x4
    
    //If four motors get turned off, it is a square matrix
    if (MOTORSRUNNING == 4) {
        Hprime.transpose();
        for (int i = 1; i <= MOTORSRUNNING; i++) {
            HTprime.set(i, 1, Hprime.get(i, 1));
        }
    }
    else {
        HTprime.transpose_not_square(Hprime);
    }

    HHTprime.mult(Hprime, HTprime);

    //M.disp();
    //Hprime.disp();
    //HTprime.disp();
    //HHTprime.disp();

    //Shit. Need a 4x4 inverse routine
    //Alright added crazy shit inverse functions
    HHT_invprime.matrix_inverse(HHTprime, 4);

    //HHT_invprime.disp();

    //Then we need HT*inv(HHT) == N-1 x 4 - 4 x 4
    HT_inv_HHTprime.mult(HTprime, HHT_invprime);
    //HT_inv_HHTprime.disp();

    //Finally we can get Q
    Qprime.mult(HT_inv_HHTprime, M);
    //Qprime.disp();
}

//Function to remove specific set number of motors
void controller::RemoveMotors(int removemotors, MATLAB motorsRemoved) {
    //Debug
    //cout << "In RemoveMotors" << endl;

    //Set MOTORSOFF
    MOTORSOFF = removemotors;

    //With motors removed we need to reinitialize all the control matrices
    MOTORSRUNNING = NUMMOTORS - MOTORSOFF;

    //Set operational flag = 0 for turned off motors
    for (int i = 1; i <= MOTORSOFF; i++) {
        int motorNum = motorsRemoved.get(i, 1);
        
        for (int j = 1; j <= NUMMOTORS; j++) {
            if (j == motorNum) {
                MOTORS[j - 1].op_flag = 0;

                //Debug
                //printf("Setting motor %i op_flag to %i \n", motorNum, MOTORS[j - 1].op_flag);
            }
        }
    }

    //Debug
    //PAUSE();

    //Indicator statement
    printf("Total Motors = %d, Removing %d motors. Running Motors = %d \n", NUMMOTORS, removemotors, MOTORSRUNNING);

    //This is where we make our configuration matrices
    Hprime.zeros(4, MOTORSRUNNING, "Configuration Matrix Prime");
    HTprime.zeros(MOTORSRUNNING, 4, "Configuration Matrix Transposed Prime");
    HHTprime.zeros(4, 4, "H*HT Prime");
    HHT_invprime.zeros(4, 4, "Inv(HHTprime)");
    HT_inv_HHTprime.zeros(MOTORSRUNNING, 4, "HT_inv_HHTprime");
    Qprime.zeros(MOTORSRUNNING, 4, "Control Matrix Prime");
    CHIprime.zeros(MOTORSRUNNING, 1, "Thrust Required Prime");

    //Set the H matrix for motors that are turned on
    for (int i = 1; i <= MOTORSRUNNING; i++) {
        if (MOTORS[i - 1].op_flag == 1) {
            //From paper, H = [-1 -ryi rxi (sigmai * cq * Rrotor / ct)]^T
            Hprime.set(1, i, - 1.0);
            Hprime.set(2, i, -MOTORS[i - 1].r.get(2, 1));
            Hprime.set(3, i, MOTORS[i - 1].r.get(1, 1));
            Hprime.set(4, i, (MOTORS[i - 1].dir * cq * Rrotor / ct));
        }
    }

    //HT*inv(HHT)*M
    //H = 4xN
    //HT = Nx4
    //HHT = 4x4

    //If four motors get turned off, it is a square matrix - I hate how this works
    if (MOTORSRUNNING == 4) {
        Hprime.transpose();
        for (int i = 1; i <= MOTORSRUNNING; i++) {
            HTprime.set(i, 1, Hprime.get(i, 1));
        }
    }
    else {
        HTprime.transpose_not_square(Hprime);
    }
    
    HHTprime.mult(Hprime, HTprime);

    //M.disp();
    //Hprime.disp();
    //HTprime.disp();
    //HHTprime.disp();

    //Shit. Need a 4x4 inverse routine
    //Alright added crazy shit inverse functions
    HHT_invprime.matrix_inverse(HHTprime, 4);

    //HHT_invprime.disp();

    //Then we need HT*inv(HHT) == N-1 x 4 - 4 x 4
    HT_inv_HHTprime.mult(HTprime, HHT_invprime);
    //HT_inv_HHTprime.disp();

    //Finally we can get Q
    Qprime.mult(HT_inv_HHTprime, M);
    //Qprime.disp();

    //Debug
    //cout << "Finished RemoveMotors" << endl;
}

//Function to compute reconfigurable matrix [CHI / CHIprime]
void controller::computeReconfigurable(double thrust, double d_roll, double d_pitch, double d_yaw) {
    //Set U
    U.set(1, 1, thrust);
    U.set(2, 1, d_roll);
    U.set(3, 1, d_pitch);
    U.set(4, 1, d_yaw);

    //Then we compute the Thrusts required for control
    //chi = Q*U
    //Q.disp();
    //U.disp();

    //Placeholder value
    double value;

    //Check if all motors are ON or if any are OFF
    if (MOTORSRUNNING == NUMMOTORS) {
        //Compute reconfiguration matrix
        CHI.mult(Q, U);
        //CHI.disp();

        //Once we know the thrust required we need to compute the pwm signal required
        //Constrain Motors between 0 and Max_Thrust
        for (int i = 0; i < NUMMOTORS; i++) {
            //Need a bunch of saturation filters
            //Need to make sure if any thrust is less than zero we change it using the CONSTRAIN function
            value = CHI.get(i + 1, 1);
            value = CONSTRAIN(value, 0.0, MOTORS[0].Max_Thrust);
            CHI.set(i + 1, 1, value);
            //Then compute Motor signals required for that amount of thrust
            MOTORS[i].pwm_signal = MOTORS[i].compute_signal_NO_OMEGA(CHI.get(i + 1, 1));
            //printf("Motor signals = %lf \n",MOTORS[i].pwm_signal);
        }
    }
    else {
        //Motors out
        //Qprime.disp();
        //U.disp();

        //Compute reconfiguration matrix
        CHIprime.mult(Qprime, U);

        //We need to multiply CHI by a factor otherwise will blow this shit up
        //I don't think we need this anymore
        //CHIprime.mult_eq((double)MOTORSRUNNING/(double)NUMMOTORS);
        //CHIprime.disp();
        
        //Flag increment for pulling value from chi
        int chi = 0;

        //Constrain thrust between 0 and Max_Thrust
        //Check whether motors are ON or OFF
        for (int i = 0; i < NUMMOTORS; i++) {
            if (MOTORS[i].op_flag == 1) {
                value = CHIprime.get(chi + 1, 1);
                value = CONSTRAIN(value, 0.0, MOTORS[0].Max_Thrust);
                CHIprime.set(chi + 1, 1, value);
                //Then compute Motor signals required for that amount of thrust
                MOTORS[i].pwm_signal = MOTORS[i].compute_signal_NO_OMEGA(CHIprime.get(chi + 1, 1));
                chi++;
            }
            else {
                MOTORS[i].pwm_signal = OUTMIN;
            }
        }
    }
}

//Function to initialize control matrix
void controller::init(MATLAB in_configuration_matrix) {
  control_matrix.zeros(NUMSIGNALS,1,"Control Signals"); //The standards must be TAERA1A2A3A4
  set_defaults();
  printf("Controller Received Configuration Matrix \n");
  //in_configuration_matrix.disp();
  CONTROLLER_FLAG = in_configuration_matrix.get(11,1);
  printf("Controller Setup \n");
}

//Set control matrix to minimum pwm
void controller::set_defaults() {
  control_matrix.set(1,1,OUTMIN);
  control_matrix.set(2,1,OUTMIN);
  control_matrix.set(3,1,OUTMIN);
  control_matrix.set(4,1,OUTMIN);
  control_matrix.set(5,1,OUTMIN);
  control_matrix.set(6,1,OUTMIN);
  control_matrix.set(7,1,OUTMIN);
  control_matrix.set(8,1,OUTMIN);
}

//Print control matrix
void controller::print() {
  for (int i = 1;i<=NUMSIGNALS;i++) {
    printf("%d ",int(control_matrix.get(i,1)));
  }
}

//Main controller loop
void controller::loop(double currentTime,int rx_array[],MATLAB sense_matrix) {
  //The sensor matrix is a 29x1. See sensors.cpp for list of sensors
  //At a minimum you need to just feed through the rxcomms into the control_matrix
  //Which means you can't have more control signals than receiver signals

  //Default Control Signals
  set_defaults();

  //I want to keep track of timeElapsed so that I can run integrators
  //and compute derivates
  elapsedTime = currentTime - lastTime;
  lastTime = currentTime;

  //At a minimum you need to compute the 8 motor signals based on 
  double motor_upper_left_top = OUTMIN;
  double motor_upper_right_top = OUTMIN;
  double motor_lower_left_top = OUTMIN;
  double motor_lower_right_top = OUTMIN;
  double motor_upper_left_bottom = OUTMIN;
  double motor_upper_right_bottom = OUTMIN;
  double motor_lower_left_bottom = OUTMIN;
  double motor_lower_right_bottom = OUTMIN;

  //First extract the relavent commands from the receiver.
  double throttle = rx_array[0];
  double aileron = rx_array[1];
  double elevator = rx_array[2];
  double rudder = rx_array[3];
  double autopilot = rx_array[4];
  bool icontrol = 0;

  switch (CONTROLLER_FLAG) {
  case -1:
    //User decides
    if (autopilot > STICK_MID) {
      icontrol = 1;
    } else {
      icontrol = 0;
    }
    break;
  case 0:
    //Always off
    icontrol = 0;
    break;
  case 1:
    //Always on
    icontrol = 1;
    break;
  }

  //Then you can run any control loop you want.
  if (icontrol) {
      //Stabilize Mode - Nonreconfigurable
      /*
      //STABILIZE MODE
      //printf(" STAB ");
      double roll_command = (aileron-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
      double pitch_command = -(elevator-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
      double yaw_rate_command = (rudder-STICK_MID)*50.0/((STICK_MAX-STICK_MIN)/2.0);
      double roll = sense_matrix.get(4,1);
      double pitch = sense_matrix.get(5,1);
      double yaw = sense_matrix.get(6,1);
      double roll_rate = sense_matrix.get(10,1); //For SIL/SIMONLY see Sensors.cpp
      double pitch_rate = sense_matrix.get(11,1); //These are already in deg/s
      double yaw_rate = sense_matrix.get(12,1); //Check IMU.cpp to see for HIL
      //state.disp();
      //printf("PQR Rate in Controller %lf %lf %lf \n",roll_rate,pitch_rate,yaw_rate);
      double kp = 10.0;
      double kd = 2.0;
      double kyaw = 0.2;
      double droll = kp*(roll-roll_command) + kd*(roll_rate);
      droll = CONSTRAIN(droll,-500,500);
      double dpitch = kp*(pitch-pitch_command) + kd*(pitch_rate);
      dpitch = CONSTRAIN(dpitch,-500,500);
      double dyaw = kyaw*(yaw_rate-yaw_rate_command);
      //printf("YAW RATE = %lf YAW RATE COMMAND = %lf DYAW = %lf \n",yaw_rate,yaw_rate_command,dyaw);
      dyaw = CONSTRAIN(dyaw,-500,500);
      //printf("d = %lf %lf %lf ",droll,dpitch,dyaw);
      //printf(" Roll Command = %lf ",roll_command);
      throttle = 1500.0; // Just for debugging. Don't yell at your past self.
      motor_upper_left_top = throttle - droll - dpitch - dyaw;
      motor_upper_right_top = throttle + droll - dpitch + dyaw;
      motor_lower_left_top = throttle - droll + dpitch + dyaw;
      motor_lower_right_top = throttle + droll + dpitch - dyaw;
      motor_upper_left_bottom = throttle - droll - dpitch + dyaw;
      motor_upper_right_bottom = throttle + droll - dpitch - dyaw;
      motor_lower_left_bottom = throttle - droll + dpitch - dyaw;
      motor_lower_right_bottom = throttle + droll + dpitch + dyaw;
      */

      //Stabilize Mode - Reconfigurable

      //Euler angle controller gains - euler angles don't use integral controllers
      double kp_roll = 0.5;   //0.5
      double kd_roll = 1.0;   //1.0
      double kp_pitch = 0.5;  //0.5
      double kd_pitch = 1.0;  //1.0
      double kp_yaw = 0;      //0 - controlling yaw itself makes it spaz out
      double kd_yaw = 0.5;    //0.5

      //Roll: [0.5 1.0] stabilizes drone in ~1 sec for IC of 1 rad/s
      //Pitch: [0.5 1.0] stabilizes drone in ~1 sec for IC of 1 rad/s
      //Yaw: [0 0.5] stabilizes yaw rate almost instantaneously for IC of 1 rad/s

      //Measure commands
      double roll_command = (aileron - STICK_MID) * 50.0 / ((STICK_MAX - STICK_MIN) / 2.0);
      double pitch_command = -(elevator - STICK_MID) * 50.0 / ((STICK_MAX - STICK_MIN) / 2.0);
      double yaw_rate_command = (rudder - STICK_MID) * 50.0 / ((STICK_MAX - STICK_MIN) / 2.0); //rudder does yaw, but here it will control yaw_rate
      //yaw itself will have to be controlled by user commands

      //Set non-stick commands
      double roll_rate_command = 0;
      double pitch_rate_command = 0;
      double yaw_command = 0;

      //Verification tests - make sure model works for different angle commands
      //roll_command = 45;
      //pitch_command = 45;
      //yaw_rate_command = 1;

      //Debug
      //printf("Roll pitch yaw command = %lf %lf %lf \n",roll_command,pitch_command,yaw_rate_command);
      //PAUSE();

      //Measure state
      double roll = sense_matrix.get(4, 1);
      double pitch = sense_matrix.get(5, 1);
      double yaw = sense_matrix.get(6, 1);
      double roll_rate = sense_matrix.get(10, 1);  //For SIL/SIMONLY see Sensors.cpp
      double pitch_rate = sense_matrix.get(11, 1); //These are already in deg/s
      double yaw_rate = sense_matrix.get(12, 1);   //Check IMU.cpp to see for HIL

      //PID control on Euler commands - u = k * (reference - measured) 
      double droll = kp_roll * (roll_command - roll) + kd_roll * (roll_rate_command - roll_rate);
      droll = CONSTRAIN(droll, -500, 500);
      double dpitch = kp_pitch * (pitch_command - pitch) + kd_pitch * (pitch_rate_command - pitch_rate);
      dpitch = CONSTRAIN(dpitch, -500, 500);
      double dyaw = kp_yaw * (yaw_command - yaw) + kd_yaw * (yaw_rate_command - yaw_rate);
      dyaw = CONSTRAIN(dyaw, -500, 500);

      //Extra filters for if nan - thanks, IEEE
      if (droll != droll) {
          if (droll > 0) {
              droll = 500;
          }
          else {
              droll = -500;
          }
      }

      if (dpitch != dpitch) {
          if (dpitch > 0) {
              dpitch = 500;
          }
          else {
              dpitch = -500;
          }
          
      }

      if (dyaw != dyaw) {
          if (dyaw > 0) {
              dyaw = 500;
          }
          else {
              dyaw = -500;
          }
      }

      //Altitude control gains
      double kp_z = 85;    //80
      double ki_z = 0.5;   //0.5
      double kd_z = 60;    //60

      //[85, 0.5, 60] - dips by about 25 m, minimal oscillation, pretty long settle time

      //Measure altitude and vertical velocity
      double z = sense_matrix.get(3,1);
      double zdot = 0;

      //If altitude_prev has been set compute a first order derivative
      if (zprev != -999) {
        zdot = (z - zprev) / elapsedTime;
      }

      //Then set the previous value
      zprev = z;   

      //Set z and zdot commands to 100 m [negative is up]
      double ZCOMMAND = -100;
      double ZDOTCOMMAND = 0;

      //Compute error and integral of error
      double zerror = ZCOMMAND - z;
      zint += zerror * elapsedTime;

      //Compute throttle
      throttle = OUTMIN - kp_z*zerror - kd_z*(ZDOTCOMMAND-zdot) - ki_z*zint;
      throttle = CONSTRAIN(throttle, OUTMIN, OUTMAX);

      //Extra filter for if it goes nan
      if (throttle != throttle) {
          if (throttle > 0) {
              throttle = OUTMAX;
          }
          else {
              throttle = OUTMIN;
          }
      }

      //Debug
      //printf("Throttle = %lf \n",throttle);

      //Compute desired thrust
      double dthrust = ((8.0 * MOTORS[0].Max_Thrust) / ((double)OUTMAX - (double)OUTMIN)) * (throttle - OUTMIN);

      //Debug
      //printf("dthrust droll dpitch dyaw %d %d %d %d \n", dthrust, droll, dpitch, dyaw);

      //Compute thrust needed for each motor
      computeReconfigurable(-dthrust, droll, dpitch, -dyaw);
      //dthrust needs to be negative. "Up is down. That's just maddeningly unhelpful. Why are these things never clear?" - Captain Jack Sparrow

      //Set pwm motor variables for control_matrix - can the long ass variables be changed to the MOTORS[i].pwm_signal like a for loop with
      //for (int i = 0; i <= NUMSIGNALS; i++) {control_matrix.set(i, 1, MOTORS[i].pwm_signal;} ?
      /*
      
      */

      //Controller
      motor_upper_left_bottom = MOTORS[0].pwm_signal;
      motor_upper_right_bottom = MOTORS[1].pwm_signal;
      motor_lower_right_bottom = MOTORS[2].pwm_signal;
      motor_lower_left_bottom = MOTORS[3].pwm_signal;
      motor_upper_left_top = MOTORS[4].pwm_signal;
      motor_upper_right_top = MOTORS[5].pwm_signal;
      motor_lower_right_top = MOTORS[6].pwm_signal;
      motor_lower_left_top = MOTORS[7].pwm_signal;
      
      //Debug - See how roll/pitch react when one side of motors is off and other is at max or mid
      /*
      //Left OFF & Right MID
      motor_upper_left_bottom = OUTMIN;
      motor_upper_right_bottom = OUTMID;
      motor_lower_right_bottom = OUTMID;
      motor_lower_left_bottom = OUTMIN;
      motor_upper_left_top = OUTMIN;
      motor_upper_right_top = OUTMID;
      motor_lower_right_top = OUTMID;
      motor_lower_left_top = OUTMIN;
      
      //Right OFF & Left MID
      motor_upper_left_bottom = OUTMID;
      motor_upper_right_bottom = OUTMIN;
      motor_lower_right_bottom = OUTMIN;
      motor_lower_left_bottom = OUTMID;
      motor_upper_left_top = OUTMID;
      motor_upper_right_top = OUTMIN;
      motor_lower_right_top = OUTMIN;
      motor_lower_left_top = OUTMID;

      //Front OFF & Back MID
      motor_upper_left_bottom = OUTMIN;
      motor_upper_right_bottom = OUTMIN;
      motor_lower_right_bottom = OUTMID;
      motor_lower_left_bottom = OUTMID;
      motor_upper_left_top = OUTMIN;
      motor_upper_right_top = OUTMIN;
      motor_lower_right_top = OUTMID;
      motor_lower_left_top = OUTMID;

      //Back OFF & Front MID
      motor_upper_left_bottom = OUTMID;
      motor_upper_right_bottom = OUTMID;
      motor_lower_right_bottom = OUTMIN;
      motor_lower_left_bottom = OUTMIN;
      motor_upper_left_top = OUTMID;
      motor_upper_right_top = OUTMID;
      motor_lower_right_top = OUTMIN;
      motor_lower_left_top = OUTMIN;
      */

      //Debug print statements
      //printf("d = %lf %lf %lf ",droll,dpitch,dyaw);
      //printf(" Roll Command = %lf ",roll_command);
      //printf("YAW RATE = %lf YAW RATE COMMAND = %lf DYAW = %lf \n",yaw_rate,yaw_rate_command,dyaw);
    
    
  } else {
    //ACRO MODE
    motor_upper_left_bottom = throttle + (aileron-STICK_MID) - (elevator-STICK_MID) + (rudder-STICK_MID);
    motor_upper_right_bottom = throttle - (aileron-STICK_MID) - (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_lower_right_bottom = throttle - (aileron-STICK_MID) + (elevator-STICK_MID) + (rudder-STICK_MID);
    motor_lower_left_bottom = throttle + (aileron-STICK_MID) + (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_upper_left_top = throttle + (aileron-STICK_MID) - (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_upper_right_top = throttle - (aileron-STICK_MID) - (elevator-STICK_MID) + (rudder-STICK_MID);
    motor_lower_right_top = throttle - (aileron-STICK_MID) + (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_lower_left_top = throttle + (aileron-STICK_MID) + (elevator-STICK_MID) + (rudder-STICK_MID);		
  }
  
  //Send the motor commands to the control_matrix values
  //control_matrix.mult_eq(0);
  //control_matrix.plus_eq(STICK_MIN);
  control_matrix.set(1,1,motor_upper_left_bottom);
  control_matrix.set(2,1,motor_upper_right_bottom);
  control_matrix.set(3,1,motor_lower_right_bottom);
  control_matrix.set(4,1,motor_lower_left_bottom);
  control_matrix.set(5,1,motor_upper_left_top);
  control_matrix.set(6,1,motor_upper_right_top);
  control_matrix.set(7,1,motor_lower_right_top);
  control_matrix.set(8,1,motor_lower_left_top);

  //Debug
  /*  for (int i = 0;i<5;i++) {
    printf(" %d ",rx_array[i]);
  }
  printf("\n");*/
  //control_matrix.disp();
}