//This is an x8 quad with 4 rotors on top and 4 rotors on bottom
//Check notes at bottom of file for useful information

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
    //RemoveMotors(4);
    
    //Remove specific number of motors
    motorsToRemove = 4;

    //Initialize MOTORSOFF vector for removing specific motors
    REMOVEMOTORS.zeros(motorsToRemove, 1, "Vector of motors to remove");

    //Remove input - Make sure to change the first indices to start at 1 and increase to motorsToRemove
    REMOVEMOTORS.set(1, 1, 1); //top_front_left
    REMOVEMOTORS.set(2, 1, 2); //top_front_right
    REMOVEMOTORS.set(3, 1, 3); //top_back_right
    REMOVEMOTORS.set(4, 1, 4); //top_back_left
    //REMOVEMOTORS.set(1, 1, 5); //bottom_front_left
    //REMOVEMOTORS.set(2, 1, 6); //bottom_front_right
    //REMOVEMOTORS.set(3, 1, 7); //bottom_back_right
    //REMOVEMOTORS.set(4, 1, 8); //bottom_back_left
    
    //Test remove motor - remove specific
    //RemoveMotors(motorsToRemove, REMOVEMOTORS);

    //Waypoint Control Flag
    WaypointControl = false;

    //Stay flag to tell controller to stop and hover after reaching waypoint
    STAY = false;
    
    //Number of Active Waypoints
    NUMWAYPTS = 4;

    //Waypoint Control Vector - right now just meters for sims. Should change to lat, lon later for real
    WAYPOINTS.zeros(5, 3, "Waypoint Matrix");

    //Star Pattern
    /*
    //Waypoint 1: (0, 0, -200) - IC
    WAYPOINTS.set(1, 1, 0);
    WAYPOINTS.set(1, 2, 0);
    WAYPOINTS.set(1, 3, -200);

    //Waypoint 2: (30, 80, -150)
    WAYPOINTS.set(2, 1, 30);
    WAYPOINTS.set(2, 2, 80);
    WAYPOINTS.set(2, 3, -150);

    //Waypoint 3: (60, 0, -100)
    WAYPOINTS.set(3, 1, 60);
    WAYPOINTS.set(3, 2, 0);
    WAYPOINTS.set(3, 3, -100);

    //Waypoint 4: (-20, 50, -150)
    WAYPOINTS.set(4, 1, -20);
    WAYPOINTS.set(4, 2, 50);
    WAYPOINTS.set(4, 3, -150);

    //Waypoint 5: (80, 50, -100)
    WAYPOINTS.set(5, 1, 80);
    WAYPOINTS.set(5, 2, 50);
    WAYPOINTS.set(5, 3, -100);
    */
    
    //Rectangle
    //Waypoint 1: (0, 0, -200) - IC
    WAYPOINTS.set(1, 1, 0);
    WAYPOINTS.set(1, 2, 0);
    WAYPOINTS.set(1, 3, -200);

    //Waypoint 2: (100, 0, -150)
    WAYPOINTS.set(2, 1, 100);
    WAYPOINTS.set(2, 2, 0);
    WAYPOINTS.set(2, 3, -150);

    //Waypoint 3: (100, 100, -100)
    WAYPOINTS.set(3, 1, 100);
    WAYPOINTS.set(3, 2, 100);
    WAYPOINTS.set(3, 3, -100);

    //Waypoint 4: (0, 100, -150)
    WAYPOINTS.set(4, 1, 0);
    WAYPOINTS.set(4, 2, 100);
    WAYPOINTS.set(4, 3, -150);

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
    addMotor(rx, -ry, -rz, 0.0, 0.0, -1.0, -1, 1);   //Motor 1 - upper_left_top - green prop w/ red dot
    addMotor(rx, ry, -rz, 0.0, 0.0, -1.0, 1, 1);     //Motor 2 - upper_right_top - red prop w/ green dot
    addMotor(-rx, ry, -rz, 0.0, 0.0, -1.0, -1, 1);   //Motor 3 - lower_right_top - red prop w/ purple dot
    addMotor(-rx, -ry, -rz, 0.0, 0.0, -1.0, 1, 1);   //Motor 4 - lower_left_top - orange prop w/ blue dot

    //Bottom Motors
    addMotor(rx, -ry, rz, 0.0, 0.0, -1.0, 1, 1);     //Motor 5 - upper_left_bottom - orange prop w/ black dot
    addMotor(rx, ry, rz, 0.0, 0.0, -1.0, -1, 1);     //Motor 6 - upper_right_bottom - green prop w/ purple dot
    addMotor(-rx, ry, rz, 0.0, 0.0, -1.0, 1, 1);     //Motor 7 - lower_right_bottom - green prop w/ red dot
    addMotor(-rx, -ry, rz, 0.0, 0.0, -1.0, -1, 1);   //Motor 8 - lower_left_bottom - red prop w/ yellow dot

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
        HTprime.transpose(Hprime);
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
                printf("Setting motor %i op_flag to %i \n", motorNum, MOTORS[j - 1].op_flag);
            }
        }
    }

    //Debug
    //PAUSE();

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

    //Counter for adding on motors to Hprime
    int ctr = 1;

    //Set the H matrix for motors that are turned on
    for (int i = 1; i <= NUMMOTORS; i++) {
        if (MOTORS[i - 1].op_flag == 1) {
            //From paper, H = [-1 -ryi rxi (sigmai * cq * Rrotor / ct)]^T
            Hprime.set(1, ctr, - 1.0);
            Hprime.set(2, ctr, -MOTORS[i - 1].r.get(2, 1));
            Hprime.set(3, ctr, MOTORS[i - 1].r.get(1, 1));
            Hprime.set(4, ctr, (MOTORS[i - 1].dir * cq * Rrotor / ct));

            //Increment counter
            ctr++;
        }
        else {
            printf("Motor % i is off \n", i);
        }
    }

    //HT*inv(HHT)*M
    //H = 4xN
    //HT = Nx4
    //HHT = 4x4

    //If four motors get turned off, it is a square matrix - I hate how this works
    if (MOTORSRUNNING == 4) {
        HTprime.transpose(Hprime);
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

  //Hardware debug
  //control_matrix.set(1,1,motor_upper_left_bottom);
  //control_matrix.set(2,1,motor_upper_right_bottom);
  //control_matrix.set(3,1,motor_lower_right_bottom);
  //control_matrix.set(4,1,motor_lower_left_bottom);
  //control_matrix.set(5,1,motor_upper_left_top);
  //control_matrix.set(6,1,motor_upper_right_top);
  //control_matrix.set(7,1,motor_lower_right_top);
  //control_matrix.set(8,1,motor_lower_left_top);
  //return;

  //First extract the relavent commands from the receiver.
  double throttle = rx_array[0];
  double aileron = rx_array[1];
  double elevator = rx_array[2];
  double rudder = rx_array[3];
  double autopilot = rx_array[4]; //Autopilot - OUTMIN = cutoff, OUTMID = ACRO, OUTMAX = AUTOPILOT
  bool icontrol = 0;

  //Debug
  //printf("rx [5] [6] [7] [8] %lf %lf %lf %lf \n",rx_array[5], rx_array[6], rx_array[7], rx_array[8]);

  switch (CONTROLLER_FLAG) {
  case -1:
    //User decides
    if (autopilot > 1.2 * STICK_MID) {
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
      //Stabilize Mode - Nonreconfigurable - Replacing ACRO with this since ACRO is unstable
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

      //Stabilize Mode - Reconfigurable + Waypoint Control

      //Initialize Commands - If WaypointControl = false, these commands are default stabilize commands
      double roll_command = 0;
      double pitch_command = 0;
      double yaw_command = 0;
      double yaw_rate_command = 0;
      double ZCOMMAND = -100;

      //Initialize errors - xerror and yerror are for waypoint control. zerror and yaw_error are for both.
      double xerror = 0;
      double yerror = 0;
      double zerror = 0;
      double yaw_error = 0;

      //Measure state before control loop

      //Measure Position - x and y are for waypoint control. z is for both.
      double x = sense_matrix.get(1, 1);
      double y = sense_matrix.get(2, 1);
      double z = sense_matrix.get(3, 1);

      //Measure xy velocity components - for waypoint control
      double u = sense_matrix.get(7, 1);  //u and v in GPS.cpp were changed to body frame u and v instead of GPS u and v for x8
      double v = -sense_matrix.get(8, 1); //it may be sensing v wrong, so added negative sign

      //Measure state - need all of these for both.
      double roll = sense_matrix.get(4, 1);
      double pitch = sense_matrix.get(5, 1);
      double yaw = sense_matrix.get(20, 1);    //changed from 6 because IMU heading is better than compass which uses GPS.
      double roll_rate = sense_matrix.get(10, 1);  //For SIL/SIMONLY see Sensors.cpp
      double pitch_rate = sense_matrix.get(11, 1); //These are already in deg/s
      double yaw_rate = sense_matrix.get(12, 1);   //Check IMU.cpp to see for HIL

      //Get Yaw from GPS and IMU - Sense Yaw and GPS Yaw reset every second. IMU yaw works
      double GPS_heading = sense_matrix.get(19, 1);
      double IMU_heading = sense_matrix.get(20, 1);
      //yaw = 1.0 * GPS_heading + 0.0 * IMU_heading;

      //Iniitalize altitude control gains to normal sim gains - Low gains for high altitude (sims) and high gains for low altitude (sims/real hardware)
      double kp_z = 30;
      double ki_z = 1.0;
      double kd_z = 55;
      //[30, 1.0, 55] - overshoot of approximately 12 m, no oscillation, 100 s settle time - Waypoint control gains
      //Need to retest for bug fixes

      //Calculate pwm needed for hover
      double throttle_hover = OUTMIN;

      //Calculate weight
      double weight = mass * GEARTH;

      //Find pwm for nominal thrust
      double hover_thrust = weight / (double)MOTORSRUNNING;
      for (int i = 0; i < NUMMOTORS; i++){
          if (MOTORS[i].op_flag == 1) {
               throttle_hover = MOTORS[i].compute_signal_NO_OMEGA(hover_thrust);          
          }
      }

      //If you want waypoint control, set WaypointControl variable in constructor to true; else, only stabilize mode
      if (WaypointControl) {
          ////////////////////////////////////////////////////////////////Waypoint Control Mode///////////////////////////////////////////////////////////
          
          /************************Heading Controller**********************/

          //Heading Control Gains
          double kp_x = 1.0;
          double ki_x = 0.0;
          double kd_x = 0.0;
          double kp_y = 1.0;
          double ki_y = 0.0;
          double kd_y = 0.0;

          //Measure waypoint command
          double XCOMMAND = WAYPOINTS.get(WayCtr, 1);
          double YCOMMAND = WAYPOINTS.get(WayCtr, 2);
          ZCOMMAND = WAYPOINTS.get(WayCtr, 3);

          //Tell stay to go true if stopped at waypoint for 10 seconds
          if ((currentTime >= timeWaypoint + 10) && (STAY == true)) {
              STAY = false;
          }

          //Hover at final destination
          /*
          if (WayCtr > NUMWAYPTS) {
              XCOMMAND = WAYPOINTS.get(1, NUMWAYPTS);
              YCOMMAND = WAYPOINTS.get(2, NUMWAYPTS);
              ZCOMMAND = WAYPOINTS.get(3, NUMWAYPTS);
          }
          */

          //Compute Error
          xerror = XCOMMAND - x;
          yerror = YCOMMAND - y;
          zerror = ZCOMMAND - z;

          //Compute integral error
          xint += xerror * elapsedTime;
          yint += yerror * elapsedTime;

          //Initialize derivative error
          double xdot = 0;
          double ydot = 0;

          //Compute derivative error
          if (xprev != -999) {
              xdot = (x - xprev) / elapsedTime;
          }

          if (yprev != -999) {
              ydot = (y - yprev) / elapsedTime;
          }

          //Increment value
          xprev = x;
          yprev = y;

          //Check if arrived at waypoint and increment waypoint counter if arrived; Tolerance of 1 m for x,y and 10 m for z - Need to make z controller better
          if ((abs(xerror) < 1.0) && (abs(yerror) < 1.0) && (abs(zerror) < 10.0) && (STAY == false)) {
              //Let know that at waypoint
              printf("Arrived at Waypoint %i \n", WayCtr);

              //Increment waypoint to next point
              WayCtr += 1;

              //Tell controller to stop and hover
              STAY = true;

              //Set time at waypoint
              timeWaypoint = currentTime;

              //Reset waypoint circuit if reaching last waypoint
              if (WayCtr > NUMWAYPTS) {
                  WayCtr = 1;
              }

          }

          //Heading gains
          double x_head = kp_x * xerror + ki_x * xint + kd_x * xdot;
          double y_head = kp_y * yerror + ki_y * yint + kd_x * ydot;

          //Calculate Yaw based on Waypoint
          double yaw_command = atan2(yerror, xerror) * RAD2DEG;

          //Debug
          //printf("GPS_heading IMU_heading yaw yaw_command %lf %lf %lf %lf \n", GPS_heading, IMU_heading, yaw, yaw_command);
          //if (WayCtr == 4) {
          //    printf("x y xerror yerror yaw_command %lf %lf %lf %lf %lf \n", x, y, xerror, yerror, yaw_command);
          //}

          //Constrain yaw command to match sensors
          yaw_command = CONSTRAIN(yaw_command, -180, 180);

          //Calculate yaw error
          yaw_error = yaw_command - IMU_heading;

          //Wrap error
          if (yaw_error > 180) {
              yaw_error -= 360;
          }
          else if (yaw_error < -180) {
              yaw_error += 360;
          }

          //Constrain yaw error to limit aggressive turns
          yaw_error = CONSTRAIN(yaw_error, -30, 30);

          /**************Foward Velocity + Sideslip Controllers************/

          //Velocity Controllers - Until pusher gets added, pitch will be used to move forward
          //Control gains
          double kp_u = 0.1;
          double ki_u = 0.0;
          double kd_u = 0.1;
          double kp_v = 0.01;  //0.2
          double ki_v = 0.0;
          double kd_v = 0.007;   //0.005

          //Velocity commands - only want forward velocity (u) when pointed at waypoint
          double uc = 0;
          double vc = 0;

          //Set max velocity [m/s]
          double umax = 1.5;

          //Set forward velocity command as a function to speed up then slow down between waypoints

          //Initialize waypoint positions
          double x1, y1, x2, y2 = 0;

          //If returning to start
          if (WayCtr == 1) {
              //Previous waypoint is last waypoint
              x1 = WAYPOINTS.get(NUMWAYPTS, 1);
              y1 = WAYPOINTS.get(NUMWAYPTS, 2);
          }
          //If any in between waypoints
          else {
              //Previous waypoint is just previous
              x1 = WAYPOINTS.get((WayCtr - 1), 1);
              y1 = WAYPOINTS.get((WayCtr - 1), 2);
          }

          //Current waypoint
          x2 = WAYPOINTS.get(WayCtr, 1);
          y2 = WAYPOINTS.get(WayCtr, 2);

          //Compute distance to between waypoints (optimal path)
          double d = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));

          //Compute distance to next waypoint
          double dn = sqrt(pow(xerror, 2) + pow(yerror, 2));

          //Set acceleration, cruise, and deceleration phases - each distance defines speed limits certain radii from waypoint
          double da = d;         //Start accelerating at or greater than optimal distance to waypoint
          double dc = 0.6 * d;   //Stop accelerating when within 80% distance to next waypoint 
          double dd = 0.2 * d;   //Start decelerating when within 40% distance next waypoint
          double ds = 0.5;       //Stop distance is when xerror and yerror < 0.5 m, so when within 0.5 m of waypoint
          double dcut = 1.5 * d; //Define cutoff boundary condition. If greater than this distance, it is going wrong way.

          //Set velocity command based on position
          if (dn > dcut) {
              //If past boundary, go in reverse because it is probably going the wrong way
              //uc = -topSpeed;
          }
          else if (dn <= dcut && dn >= dc) {
              //Acceleration phase - linear acceleration from 0 to top speed when leaving accel distance. 
              uc = 0.75 * umax * ((dn - d) / (dc - d) + 1);

              //Set top achieved speed for cruise and decel.
              topSpeed = u;

              //Constrain
              topSpeed = CONSTRAIN(topSpeed, 0.1, umax);
          }
          else if (dn < dc && dn >= dd) {
              //Cruise phase - keep top speed achieved in accel phase
              uc = topSpeed;
          }
          else if(dn < dd && dn >= ds) {
              //Deceleration phase - linear deceleration from top speed to zero
              uc = topSpeed * (1 - (dn - dd) / (ds - dd));
          }
          else if(dn < ds || STAY == true) {
              //Stop phase - stop at waypoint
              uc = 0;
          }

          //Velocity failsafe - if sideslip or velocity get too large, stop to correct
          if (abs(v) > 1 || abs(u) > 5) {
              uc = 0;
          }

          //Constrain command 
          uc = CONSTRAIN(uc, 0, umax);

          //Calculate error in forward and side velocity
          double uerror = uc - u;
          double verror = vc - v;

          //Calculate integral error
          uint += uerror * elapsedTime;
          vint += verror * elapsedTime;

          //Anti-Integral Windup
          if (abs(uint) > 5) {
              uint = 5 * sign(uint);
          }
          if (abs(vint) > 5) {
              vint = 5 * sign(vint);
          }

          //Initialize derivative of velocity
          double udot = 0;
          double vdot = 0;

          //Calculate derivative of velocity
          if (uprev != -999) {
              udot = (u - uprev) / elapsedTime;
          }
          if (vprev != -999) {
              vdot = (v - vprev) / elapsedTime;
          }

          //Set prev value
          uprev = u;
          vprev = v;

          //Calculate pitch and roll commands
          pitch_command = -atan2((kp_u * uerror + ki_u * uint + kd_u * udot), GEARTH) * RAD2DEG;
          roll_command = atan2((kp_v * verror + ki_v * vint + kd_v * vdot), GEARTH) * RAD2DEG;
          //Both are negative kp because if you want to go foward, you have to tilt forward which is negative pitch.
          //If you are sliding right, you will want to roll left (-) to stop.
          if (WayCtr == 1) {
              printf("xerror yerror u uc v roll_command %lf %lf %lf %lf %lf %lf \n", xerror, yerror, u, uc, v, roll_command);
          }

          //Constrain roll and pitch
          roll_command = CONSTRAIN(roll_command, -5, 5);
          pitch_command = CONSTRAIN(pitch_command, -5, 5);

          //Debug
          //printf("GPS_heading IMU_heading yaw %lf %lf %lf \n", GPS_heading, IMU_heading, yaw);
      }
      else {
          //////////////////////////////////////////////////////////////Stabilize Only Mode/////////////////////////////////////////////////////

          //Measure commands from pilot
          roll_command = (aileron - STICK_MID) * 50.0 / ((STICK_MAX - STICK_MIN) / 2.0);
          pitch_command = -(elevator - STICK_MID) * 50.0 / ((STICK_MAX - STICK_MIN) / 2.0);
          yaw_rate_command = (rudder - STICK_MID) * 50.0 / ((STICK_MAX - STICK_MIN) / 2.0);

          //Compute yaw error
          yaw_error = yaw_command - IMU_heading;

          //Constrain yaw error to limit aggressive turns
          yaw_error = CONSTRAIN(yaw_error, -30, 30);

          //Altitude Command
          ZCOMMAND = -1;

          //Altitude control gains 
          kp_z = 1.0;
          ki_z = 0.1;
          kd_z = 0.5;
          //[80000, 1000, 100000] - roughly hovers in place - Autopilot gains - real hardware

          //Combine throttle input and throttle hover as weighted average - Commented out to turn off altitude controller
          //throttle = 0.3 * throttle_hover + 0.7 * throttle;

          //Verification tests - make sure model works for different angle commands
          //roll_command = 45;
          //roll_command = 1;
          //pitch_command = 45;
          //yaw_rate_command = 1;

          //Test both roll and pitch commands and yaw rate command in one go for brevity of thesis
          /*
          if (currentTime < 20) {
              roll_command = 45;
              //roll_command = 30; //For four motors removed, max roll/pitch command is 30 degrees for 20 seconds without hitting ground
          }
          else if (currentTime > 50 and currentTime < 80) {
              pitch_command = 45;
              //pitch_command = 20; //For four motors removed, max roll/pitch command is 30 degrees for 20 seconds without hitting ground
          }
          else {
              yaw_rate_command = 1;
          }
          */

          //Debug
          //printf("Roll pitch yaw command = %lf %lf %lf \n",roll_command,pitch_command,yaw_rate_command);
          //PAUSE();
      }
      
      /**********************Euler Angle Controllers*******************/

      //Euler angle controller gains - euler angles don't use integral controllers
      double kp_roll = 0.0;    //0.5
      double kd_roll = 0.0;    //1.0
      double kp_pitch = 0.0;   //0.5
      double kd_pitch = 0.0;   //1.0
      double kp_yaw = 0.0;     //0.1 - setting to zero for hardware testing cause BB is being weird
      double kd_yaw = 0.1;     //0.5

      //Set non-stick commands
      double roll_rate_command = 0;
      double pitch_rate_command = 0;
      //double yaw_rate_command = 0; //changed stick yaw to yaw rate for hardware testing

      //PID control on Euler commands - u = k * (reference - measured) 
      //droll and dpitch control roll and pitch
      //dyaw controls yaw rate
      double droll = kp_roll * (roll_command - roll) + kd_roll * (roll_rate_command - roll_rate);
      droll = CONSTRAIN(droll, -500, 500);
      double dpitch = kp_pitch * (pitch_command - pitch) + kd_pitch * (pitch_rate_command - pitch_rate);
      dpitch = CONSTRAIN(dpitch, -500, 500);
      double dyaw = kp_yaw * yaw_error + kd_yaw * (yaw_rate_command - yaw_rate);
      dyaw = CONSTRAIN(dyaw, -500, 500);

      //Debug
      printf("yaw_rate_command yaw_rate %lf %lf \n", yaw_rate_command, yaw_rate);

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

      /************************Altitude Controller*********************/

      //PID Control on Altitude
      //ALtitude measurement moved before split for waypoint control
      //Error calculation moved in else loop 

      //Initialize vertical velocity
      double zdot = 0;

      //If altitude_prev has been set compute a first order derivative
      if (zprev != -999) {
        zdot = (z - zprev) / elapsedTime;
      }

      //Then set the previous value
      zprev = z;   

      //Set z and zdot commands to 100 m [negative is up]
      //double ZCOMMAND = -100; //Commented out for waypoint testing
      double ZDOTCOMMAND = 0;

      //Compute integral of error
      zint += zerror * elapsedTime;

      //Compute throttle - Commented out to turn off altitude controller. Altitude will be stick only for hardware testing
      //throttle = throttle - kp_z*zerror - kd_z*(ZDOTCOMMAND-zdot) - ki_z*zint;
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
      double dthrust = (((double)MOTORSRUNNING * MOTORS[0].Max_Thrust) / ((double)OUTMAX - (double)OUTMIN)) * (throttle - OUTMIN);

      //Debug
      //printf("dthrust droll dpitch dyaw %d %d %d %d \n", dthrust, droll, dpitch, dyaw);

      //Compute thrust needed for each motor
      computeReconfigurable(-dthrust, droll, dpitch, -dyaw);
      
      //Notes:
      //dthrust needs to be negative. "Up is down. That's just maddeningly unhelpful. Why are these things never clear?" - Captain Jack Sparrow
      //dyaw also needs to be negative apparently. 

      //Suggestion:
      //Set pwm motor variables for control_matrix - can the long ass variables be changed to the MOTORS[i].pwm_signal like a for loop with
      //for (int i = 0; i <= NUMSIGNALS; i++) {control_matrix.set(i, 1, MOTORS[i].pwm_signal;} ?

      //Keep for copy/paste when running tests
      /*
      motor_upper_left_top = MOTORS[0].pwm_signal;
      motor_upper_right_top = MOTORS[1].pwm_signal;
      motor_lower_right_top = MOTORS[2].pwm_signal;
      motor_lower_left_top = MOTORS[3].pwm_signal;
      motor_upper_left_bottom = MOTORS[4].pwm_signal;
      motor_upper_right_bottom = MOTORS[5].pwm_signal;
      motor_lower_right_bottom = MOTORS[6].pwm_signal;
      motor_lower_left_bottom = MOTORS[7].pwm_signal;
      */

      //Set PWM signals
      motor_upper_left_top = MOTORS[0].pwm_signal;
      motor_upper_right_top = MOTORS[1].pwm_signal;
      motor_lower_right_top = MOTORS[2].pwm_signal;
      motor_lower_left_top = MOTORS[3].pwm_signal;
      motor_upper_left_bottom = MOTORS[4].pwm_signal;
      motor_upper_right_bottom = MOTORS[5].pwm_signal;
      motor_lower_right_bottom = MOTORS[6].pwm_signal;
      motor_lower_left_bottom = MOTORS[7].pwm_signal;
      
      //Debug - See how roll/pitch react when one side of motors is off and other is at max or mid. Keep in case tests need to be ran again.
      /*
      //Left OFF & Right MID
      motor_upper_left_top = OUTMIN;
      motor_upper_right_top = OUTMID;
      motor_lower_right_top = OUTMID;
      motor_lower_left_top = OUTMIN;
      motor_upper_left_bottom = OUTMIN;
      motor_upper_right_bottom = OUTMID;
      motor_lower_right_bottom = OUTMID;
      motor_lower_left_bottom = OUTMIN;
      
      //Right OFF & Left MID
      motor_upper_left_top = OUTMID;
      motor_upper_right_top = OUTMIN;
      motor_lower_right_top = OUTMIN;
      motor_lower_left_top = OUTMID;
      motor_upper_left_bottom = OUTMID;
      motor_upper_right_bottom = OUTMIN;
      motor_lower_right_bottom = OUTMIN;
      motor_lower_left_bottom = OUTMID;

      //Front OFF & Back MID
      motor_upper_left_top = OUTMIN;
      motor_upper_right_top = OUTMIN;
      motor_lower_right_top = OUTMID;
      motor_lower_left_top = OUTMID;
      motor_upper_left_bottom = OUTMIN;
      motor_upper_right_bottom = OUTMIN;
      motor_lower_right_bottom = OUTMID;
      motor_lower_left_bottom = OUTMID;

      //Back OFF & Front MID
      motor_upper_left_top = OUTMID;
      motor_upper_right_top = OUTMID;
      motor_lower_right_top = OUTMIN;
      motor_lower_left_top = OUTMIN;
      motor_upper_left_bottom = OUTMID;
      motor_upper_right_bottom = OUTMID;
      motor_lower_right_bottom = OUTMIN;
      motor_lower_left_bottom = OUTMIN;
      */

      //Debug print statements
      //printf("d = %lf %lf %lf ",droll,dpitch,dyaw);
      //printf(" Roll Command = %lf ",roll_command);
      //printf("YAW RATE = %lf YAW RATE COMMAND = %lf DYAW = %lf \n",yaw_rate,yaw_rate_command,dyaw);
    
    
  } else {
    //ACRO MODE - Unstable, so replacing with old stabilize mode
    /*
    motor_upper_left_bottom = throttle + (aileron - STICK_MID) - (elevator - STICK_MID) + (rudder - STICK_MID);
    motor_upper_right_bottom = throttle - (aileron-STICK_MID) - (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_lower_right_bottom = throttle - (aileron-STICK_MID) + (elevator-STICK_MID) + (rudder-STICK_MID);
    motor_lower_left_bottom = throttle + (aileron-STICK_MID) + (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_upper_left_top = throttle + (aileron-STICK_MID) - (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_upper_right_top = throttle - (aileron-STICK_MID) - (elevator-STICK_MID) + (rudder-STICK_MID);
    motor_lower_right_top = throttle - (aileron-STICK_MID) + (elevator-STICK_MID) - (rudder-STICK_MID);
    motor_lower_left_top = throttle + (aileron-STICK_MID) + (elevator-STICK_MID) + (rudder-STICK_MID);
    */	

    //STABILIZE MODE
    //printf(" STAB ");
    double roll_command = (aileron - STICK_MID) * 50.0 / ((STICK_MAX - STICK_MIN) / 2.0);
    double pitch_command = -(elevator - STICK_MID) * 50.0 / ((STICK_MAX - STICK_MIN) / 2.0);
    double yaw_rate_command = (rudder - STICK_MID) * 50.0 / ((STICK_MAX - STICK_MIN) / 2.0);

    double roll = sense_matrix.get(4, 1);
    double pitch = sense_matrix.get(5, 1);
    double yaw = sense_matrix.get(6, 1);
    double roll_rate = sense_matrix.get(10, 1); //For SIL/SIMONLY see Sensors.cpp
    double pitch_rate = sense_matrix.get(11, 1); //These are already in deg/s
    double yaw_rate = sense_matrix.get(12, 1); //Check IMU.cpp to see for HIL

    //state.disp();
    //printf("PQR Rate in Controller %lf %lf %lf \n",roll_rate,pitch_rate,yaw_rate);

    double kp = 10.0; //10.0
    double kd = 0.0;  //2.0
    double kyaw = 5.0;//0.2

    double droll = kp * (roll - roll_command) + kd * (roll_rate);
    droll = CONSTRAIN(droll, -500, 500);

    double dpitch = kp * (pitch - pitch_command) + kd * (pitch_rate);
    dpitch = CONSTRAIN(dpitch, -500, 500);

    double dyaw = kyaw * (yaw_rate - yaw_rate_command);
    //printf("YAW RATE = %lf YAW RATE COMMAND = %lf DYAW = %lf \n",yaw_rate,yaw_rate_command,dyaw);
    dyaw = CONSTRAIN(dyaw, -500, 500);
    //printf("d = %lf %lf %lf ",droll,dpitch,dyaw);
    //printf(" Roll Command = %lf ",roll_command);
    //throttle = 1500.0; // Just for debugging. Don't yell at your past self.

    motor_upper_left_top = throttle - droll - dpitch - dyaw;
    motor_upper_right_top = throttle + droll - dpitch + dyaw;
    motor_lower_left_top = throttle - droll + dpitch + dyaw;
    motor_lower_right_top = throttle + droll + dpitch - dyaw;
    motor_upper_left_bottom = throttle - droll - dpitch + dyaw;
    motor_upper_right_bottom = throttle + droll - dpitch - dyaw;
    motor_lower_left_bottom = throttle - droll + dpitch - dyaw;
    motor_lower_right_bottom = throttle + droll + dpitch + dyaw;

  }
  
  //Send the motor commands to the control_matrix values
  //control_matrix.mult_eq(0);
  //control_matrix.plus_eq(STICK_MIN);

  //Suggestion:
  //Again, could just use MOTORS[i].pwm_signal and use for(int i = 0; i < NUMMOTORS; i++) {control_matrix.set(i,1,MOTORS[i].pwm_signal;}
  //Using MOTORS[i].pwm_signal would avoid needing to define these variables

  //If switch set to STD, cut off motors
  if (autopilot < 0.8 * STICK_MID) {
      set_defaults();
  }
  //Else if switch set to LTR or AUTO, do stabilize or reconfigurable mode. LTR is stabilize, and AUTO is reconfigurable.
  else if (autopilot > 0.8 * STICK_MID) {
      //For hardware testing:
      //1. Turn all gains to zero.
      //2. Make sure motors go from min to max for thrust, roll, pitch, and yaw rate commands.
      //3. Set all but one motor to min and test spin of each motor individually.
      //4. Don't really try to get off the ground. Just try to get it to roll, pitch, and yaw a little and correctly with stick commands.
      //5. Add proportional gain only and test roll, pitch, and yaw rate commands individually.
      //6. Add derivative gain and test roll, pitch, and yaw rate commands individually.
      control_matrix.set(1, 1, motor_upper_left_top);     //OUTMIN); //motor_upper_left_top);
      control_matrix.set(2, 1, motor_upper_right_top);    //OUTMIN); //motor_upper_right_top);
      control_matrix.set(3, 1, motor_lower_right_top);    //OUTMIN); //motor_lower_right_top);
      control_matrix.set(4, 1, motor_lower_left_top);     //OUTMIN); //motor_lower_left_top);
      control_matrix.set(5, 1, motor_upper_left_bottom);  //OUTMIN); //motor_upper_left_bottom);
      control_matrix.set(6, 1, motor_upper_right_bottom); //OUTMIN); //motor_upper_right_bottom);
      control_matrix.set(7, 1, motor_lower_right_bottom); //OUTMIN); //motor_lower_right_bottom);
      control_matrix.set(8, 1, motor_lower_left_bottom);  //OUTMIN); //motor_lower_left_bottom);
  }

  //Debug
  /*  for (int i = 0;i<5;i++) {
    printf(" %d ",rx_array[i]);
  }
  printf("\n");*/
  //control_matrix.disp();
}

//Notes:
/*
1. PWM Used to be PWM 1-8 controlling {1, 4, 2, 3, 6, 8, 7, 5}. Unknown why. Just logged in case need to be changed back.

2. FASTCASST data print read as: T, dt time time step RX Thrust Aileron Elevator Rudder Autopilot ? RPY Roll Pitch Yaw PWM 1 2 3 4 5 6 7 8 LL(B)H Longitude Latitude Altitude

3. Autopilot switch (rx_array[4]) has added motor cutoff. On IRIS transmitter, that is the 3 positon switch. STD is STICK_MIN which is motor cutoff, LTR is STICK_MID
   which is old stabiliize mode, and AUTO is STICK_MAX which is reconfigurable control law autopilot. 
   3.a "ACRO mode is impossible to fly. Some sort of proportional control is required." - Dr. Montalvo 3/14/2025. So don't use ACRO cause it's unstable.
   3.b Since ACRO mode is unstable, it was replaced with the old stabilize code that is not the reconfigurable control law.
   3.c For future waypoint control, may need to have WaypointControl flag as a switch so that if drone starts to fritz, the pilot can regain turn it off and regain control.

4. For props, red bolts are righty tighty, and black bolts are lefty tighty.Very annoying.

5. To run SIMONLY in WSL: 
   5.a If os.system is commented out in plotdata_sim.py:
       cd FASTCASST
       make clean 
       make simonly MODEL="x8"
       python3 plotdata_sim.py
   5.b If make clean and make simonly not commented out:
       cd FASTCASST
       python3 plotdata_sim.py
   5.c MODEL has to be all caps. The string is just whatever is after V_ for the model folder being used.

6. To run auto on a raspberry pi:
   login: -ask Dr. Montalvo-
   password: -ask Dr. Montalvo-
   cd FASTCASST
   git pull origin [branch]
   make clean
   make auto MODEL="x8"
   sudo ./auto.exe x8/

   6.a At the time of writing this, work was done in the reconfig branch. If you have your own, it'll be yours.

7. Common WSL commands for running FASTCASST:
   7.a ls -> list; lists files and folders in current directory
   7.b cd -> change directory; changes between folders
   7.c cd - -> go back one directory
   7.d \cd -> return to root directory
   7.e nano [name].filetype -> opens file in nano for editing code
   7.f sudo -> super user do; execute commands that require special privileges
   7.g sudo python3 [name].py -> run Python file

8. Common GitBash commands used during research:
   8.a git branch -> tells you what branch you are in
   8.b git status -> lists changed files
   8.c git diff -> shows difference between what you have and what is online
   8.d git commit -am "Description" -> log changes and description of changes before pushing code
   8.e git push origin [branch] -> pushes your code to specified branch
   8.f git pull origin [branch] -> pulls code down from specified branch on GitHub
   8.g git checkout [branch] -> swaps to specified branch

9. To push code, you need an SSH key and a GitHub account, and you are gonna have to look that up cause it's complicated.
*/