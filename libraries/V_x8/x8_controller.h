#ifndef CONTROLLER_H
#define CONTROLLER_H

//This is another class that is craft dependent and as such
//must adhere to specific standards

#include <MATLAB/MATLAB.h>  //this is for MATLAB vectors/matrices
#include <RCIO/RCIO.h>      //this is for STICK values
#include <Mathp/mathp.h>    //this is for conversion variables
#include <math.h>           //this is for trig functions

//Motor class for defining motor attributes - See SUAM motor.h/cpp
class Motor {
private:
public:
	MATLAB r, n, Tn, datapts;
	int dir, op_flag;
	double pwm_signal, pwm_nominal, thrust, torque, kt, rho, a, A, R, ct, cq, Max_Thrust, Nominal_Thrust;
	Motor();
	void MotorCalcs(double T, double p, double w);
	double compute_signal_NO_OMEGA(double);
	void compute_thrust_NO_OMEGA();
	void compute_torque_NO_OMEGA();

	//List of public variables and function
	/*r - vector from center of mass (CM) to motor; [m]
	* n - normal vector for motor, [0 0 -1]^T; [?]
	* Tn - thrust vector for motor; [N]
	* datapts - vector of [T pwm omega_RPM] datapoints for motor calculations;
	* dir - direction of motor where -1 = CCW and 1 = CW; []
	* op_flag - operational flag for motor where 0 = OFF and 1 = ON; []
	* pwm_output - placeholder pwm var for assigning pwm to long ass var after computing signals; []
	* thrust - thrust computed by throttle signal; [N]
	* torque - torque computed by throttle signal; [N*m]
	* kt - K factor of battery?; [?]
	* rho - denisty of air; [kg/m^3]
	* a - angular velocity computed by throttle; [rad/s]
	* A - motor sweep area; [m^2]
	* R - rotor radius; [m]
	* ct - coefficient of thrust; []
	* cq - torque coefficient; [?]
	* Max_Thrust - variable to set max thrust of motor; [N]
	* Nominal_Thrust - variable to set nominal thrust of motor; [N]
	* Motor - constructor for Motor class, initializes r and n vectors;
	* MotorCalcs - function to calculate ct and cq for thrust datapoint (T), pwm datapoint (p), and ang vel datapoint (w);
	*/
};

//Controller class for computing PWM motor signals using fucky math magic
class controller {
private:
	double elapsedTime = 0, lastTime=0;
	double mass, Ixx, Iyy, Izz, ct, cq, Rrotor, rx, ry, rz, nx, ny, nz;
	int CONTROLLER_FLAG = -99, motorsToRemove = 0;
	double xprev = -999, xint = 0, yprev = -999, yint = 0, uprev = -999, uint = 0, vprev = -999, vint = 0, zprev = -999, zint = 0;
	void set_defaults();

	//List of private variables and functions
	/*elapsedTime - currentTime - lastTime, used for delta(t) for integration; [s]
	* lastTime - keeps track of previous loop time for delta(t); [s]
	* mass - mass of BumbleBee; [kg]
	* Ixx - x inertia; [kg*m^2]
	* Iyy - y inertia; [kg*m^2]
	* Izz - z inertia; [kg*m^2]
	* ct - coefficient of thrust; []
	* cq - torque coefficient; [?] 
	* Rrotor - radius of prop; [m]
	* rx - x-component of r vector; [m]
	* ry - y-component of r vector; [m]
	* rz - z-component of r vector; [m]
	* nx - x-component of n vector, usually 0; [m]
	* ny - y-component of n vector, usually 0; [m]
	* nz - z-component of n vector, usually -1; [m]
	* CONTROLLER_FLAG - sets controller state, -1 -> User decides, 0 - OFF, 1 - ON; []
	* motorsToRemove - sets number of motors to remove / sets size for specific array; []
	* xprev - previous x for derivative control of heading; [m]
	* xint - integral error for x for integral control of heading; [m * s]
	* yprev - previous y for derivative control of heading; [m]
	* yint - integral error for y for integral control of heading; [m * s]
	* uprev - previous u val for derivative control of u; [m/s]
	* uint - integral error for u; [m]
	* vprev - previous v val for derivative control of v; [m/s]
	* vint - integral error for v; [m]
	* zprev - previous altitude value for derivative control of z; [m]
	* zint - integral error for z; [m*s]
	* set_defaults - sets motor control matrix to minimum PWM signal;
	*/

public:
	int NUMMOTORS = 0, MOTORSOFF = 0, NUMSIGNALS = 8;
	int MOTORSRUNNING;
	MATLAB control_matrix, REMOVEMOTORS;
	MATLAB M, U, H, HT, HHT, HHT_inv, HT_inv_HHT, Q, CHI;
	MATLAB Hprime, HTprime, HHTprime, HHT_invprime, HT_inv_HHTprime, Qprime, CHIprime, datapts;
	MATLAB WAYPOINTS;
	int WayCtr = 1, NUMWAYPTS;
	bool WaypointControl, STAY;
	double timeWaypoint, topSpeed;
	double Tdatapt_, pwm_datapt_, omegaRPMdatapt_;
	Motor MOTORS[20];
	void addMotor(double, double, double, double, double, double, int, int);
	void MotorBeep(MATLAB);
	void MotorsSetup(MATLAB datapts);
	void RemoveMotors(int);
	void RemoveMotors(int, MATLAB);
	void computeReconfigurable(double, double, double, double);
	void loop(double currentTime,int rx_array[],MATLAB sense_matrix);
	void init(MATLAB in_configuration_matrix);
	void print();
	controller();

	//List of public variables and functions
	/*NUMMOTORS - total number of motors that are ON, initialized to zero, increment/decrement when adding/removing motors; []
	* MOTORSOFF - number of motors that are OFF, initialized to zero, increment/decrement when removing/adding motors; []
	* NUMSIGNALS - number of signals needed to calculate, 8 due to 8 motors, if motor is OFF, signal is OUTMIN; []
	* MOTORSRUNNING - keeps track of number of motors that are ON; []
	* control_matrix - control vector of PWM signals, vector in form of [Thrust Aileron Elevator Rudder A1 A2 A3 A4]^T; []
	* REMOVEMOTRS - vector of motor numbers to turn OFF; []
	* M - 4x4 mass matrix for calcs, diag[m Ixx Iyy Izz]; [kg & kg*m^2]
	* U - 4x1 control vector for calcs, [dthrust droll dpitch dyaw]^T; [?]
	* H - 4x8 parameter matrix (P in AIAA paper) for calcs, [-1 -ryi rxi cq*R*sigmai/ct]^T; [?]
	* HT - 8x4 matrix for transpose_not_square(H); [?]
	* HHT - 4x4 matrix for mult(H,HT); [?]
	* HHT_inv - 4x4 matrix for inv(HHT); [?]
	* HT_inv_HHT - 8x4 matrix for mult(HT, HHT_inv); [?]
	* Q - 8x4 configuration matrix for mult(HT_inv_HHT, M); [?]
	* CHI - 8x1 calculated control thrust vector; [N]
	* Hprime - 4xN parameter matrix for when motors are OFF; [?]
	* HTprime - Nx4 matrix for transpose_not_square(Hprime); [?]
	* HHTprime - 4x4 matrix for mult(Hprime, HTprime); [?]
	* HHT_invprime - 4x4 matrix for inv(HHTprime); [?]
	* HT_inv_HHTprime - Nx4 matrix for mult(HTprime, HHT_invprime); [?]
	* Qprime - Nx4 configuration matrix for mult(HT_inv_HHTprime, M); [?]
	* CHIprime - Nx1 calculated control thrust vector; [N]
	* WAYPOINTS - Nx3 matrix for waypoints, vector seen as {x y z}; [m]
	* WayCtr - counter to increment waypoint;
	* NUMWAYPTS - number of active waypoints;
	* WaypointControl - bool to activate or deactive waypoint control, if not active, just stabilization code;
	* STAY - boolean to tell controller to stop moving after hitting a waypoint;
	* timeWaypoint - log when reaching waypoint and set velocity command to zero for 10 s;
	* topSpeed - top speed achieved before reaching cruise phase; [m/s]
	* Tdatapt_ - thrust datapoint for motor calculations; [N]
	* pwm_datapt_ - pwm datapoint that corresponds to Tdatapt_; []
	* omegaRPMdatapt_ - ang vel datapoint that corresponds to Tdatapt_; []
	* MOTORS - create 20 motors "just cuz I can" - quote from Propulsion.h in SUAM;
	* addMotor - function to add a motor, has inputs of (rx ry rz nx ny nz dir op_flag);
	* MotorBeep - function to compute configuration matrix [Q] for all motors;
	* MotorsSetup - function to create the 8 motors and set r, n, dir, and op_flag; 
	* RemoveMotors(int) - function to remove a set number of motors starting at the last added and working in;
	* RemoveMotors(int, MATLAB) - function to remove a specific set of motors from a given array;
	* computeReconfigurable - function to compute the reconfigurable matrix [CHI / CHIprime] and set the motor object's pwm_signal to computed signal;
	* loop - main controller loop that decides controller mode, calculates controller commands, and sets control_matrix;
	* init - initialization function for control_matrix and sets to minimum pwm output;
	* print - print function for debugging;
	* controller - constructor for controller class;
	*/
};

#endif
