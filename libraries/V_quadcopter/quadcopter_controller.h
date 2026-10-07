#ifndef CONTROLLER_H
#define CONTROLLER_H

//This is another class that is craft dependent and as such
//must adhere to specific standards

#include <MATLAB/MATLAB.h>  //this is for MATLAB vectors/matrices
#include <RCIO/RCIO.h>      //this is for STICK values
#include <Mathp/mathp.h>    //this is for conversion variables
#include <math.h>           //this is for trig functions
#include <vector> 		//this is for vectors

//Controller class for computing PWM motor signals using fucky math magic
class controller {
private:
	double elapsedTime = 0, lastTime=0;
	double mass, Ixx, Iyy, Izz, ct, cq, Rrotor, rx, ry, rz, nx, ny, nz;
	int CONTROLLER_FLAG = -99;
	double xprev = -999, xint = 0, yprev = -999, yint = 0, uprev = -999, uint = 0, vprev = -999, vint = 0, altitude_prev = -999, zint = 0;
	double roll_command = -99, pitch_command = -99, yaw_command = -99, velocity_command = -99, altitude_command = 99;
	double throttle = OUTMIN, aileron = OUTMID, elevator = OUTMID, rudder = OUTMID, autopilot = OUTMIN;
	double altitude_int = 0,velocity_int=0;
	double distance_dot = 0, distance_prev = -999, prevTime = 0;
	void set_defaults();
	void AttitudeLoop(MATLAB sense_matrix);
	void AltitudeLoop(MATLAB sense_matrix);
	void VelocityLoop(MATLAB sense_matrix);
	void WaypointLoop(MATLAB sense_matrix,double currentTime);
	int NUMWAYPOINTS = 0; 
  	int WAYINDEX = 0;
	int PRINTER = 0;
  	std::vector<double> WAYPOINTS_X;
  	std::vector<double> WAYPOINTS_Y;
	//At a minimum you need to compute the 8 motor signals
  	double motor_upper_left = OUTMIN;
  	double motor_upper_right = OUTMIN;
  	double motor_lower_left = OUTMIN;
  	double motor_lower_right = OUTMIN;
public:
	int NUMMOTORS = 0, MOTORSOFF = 0, NUMSIGNALS = 4;
	int MOTORSRUNNING;
	MATLAB control_matrix;
	MATLAB M, U, H, HT, HHT, HHT_inv, HT_inv_HHT, Q, CHI;
	MATLAB Hprime, HTprime, HHTprime, HHT_invprime, HT_inv_HHTprime, Qprime, CHIprime, datapts;
	MATLAB WAYPOINTS;
	int WayCtr = 1, NUMWAYPTS;
	bool WaypointControl, STAY;
	double timeWaypoint, topSpeed;
	double Tdatapt_, pwm_datapt_, omegaRPMdatapt_;
	void loop(double currentTime,int rx_array[],MATLAB sense_matrix);
	void init(MATLAB in_configuration_matrix);
	void print();
	controller();
};

#endif
