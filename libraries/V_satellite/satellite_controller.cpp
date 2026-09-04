//This is a portal cube template. You must adhere to these standards if you write your
//own

#include "satellite_controller.h"

controller::controller() {
};

void controller::init(MATLAB in_configuration_matrix) {
  control_matrix.zeros(NUMSIGNALS,1,"Control Signals"); //NUMSIGNALS SET AS NUMTORQUERS+NUMRWS IN PARAMS.H and SATELLITE_CONTROLLER.H
  set_defaults();
  printf("Controller Received Configuration Matrix \n");
  //in_configuration_matrix.disp();
  CONTROLLER_FLAG = in_configuration_matrix.get(11,1);
  printf("Controller Setup \n");
  pqr.zeros(3,1,"PQR");
  mxyz.zeros(3,1,"MXYZ");
  desired_moments.zeros(3,1,"desired moment");
}

void controller::set_defaults() {
  //First 3 here are magTs
  control_matrix.set(1,1,STICK_MID);
  control_matrix.set(2,1,STICK_MID);
  control_matrix.set(3,1,STICK_MID);
  //The next ones are reaction wheel speeds
  control_matrix.set(4,1,STICK_MID);
  control_matrix.set(5,1,STICK_MID);
  control_matrix.set(6,1,STICK_MID);
}

void controller::print() {
  for (int i = 1;i<=NUMSIGNALS;i++) {
    printf("%d ",int(control_matrix.get(i,1)));
  }
}

void controller::loop(double currentTime,int rx_array[],MATLAB sense_matrix) {
  //The sensor state is a 12x1 of standard 6DOF sensors
  //At a minimum you need to just feed through the rxcomms into the ctlcomms
  //Which means you can't have more control signals than receiver signals
  //Default Control Signals
  set_defaults();

  //I want to keep track of timeElapsed so that I can run integrators
  //and compute derivates
  elapsedTime = currentTime - lastTime;
  lastTime = currentTime;
  
  //First extract the relavent commands from the receiver.
  double throttle = rx_array[0];
  double aileron = rx_array[1];
  double elevator = rx_array[2];
  double rudder = rx_array[3];
  double autopilot = rx_array[4];
  bool icontrol = 0;
  //printf("autopilot = %lf \n",autopilot);

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
    //printf("RUNNING DETUMBLING \n");
    //Extract PQR in (deg/s)
    //sense_matrix.disp();
    pqr.vecset(1,3,sense_matrix,10);
    //pqr.disp();
    //Convert to (rad/s) 
    pqr.mult_eq(PI/180.0);
    if (pqr.norm() < 0.1) { //0.1 rad per second is around 5.7 deg/s
      //Defaults already set for magTs so just leave those blank
      //pqr.disp();
      //printdouble(pqr.norm(),"PQR Norm = ");
      //Here we turn on reaction wheels to spin up to a speed that will allow us to detumble
      //For now we'll set them to zero until we're sure everything is working
      printf("RUNNING REACTION WHEEL SPINUP \n");
      return;
    }
    //pqr.disp();
    //Extract Magnetomter Readings
    mxyz.vecset(1,3,sense_matrix,13);
    //mxyz.disp();
    //mu_ideal = k*(omega cross B)
    desired_moments.cross(pqr,mxyz);
    desired_moments.mult_eq(KMAG);
    //Convert to currents
    desired_moments.mult_eq(1.0/(NUMTURNS*AREA));
    desired_moments.plus_eq(STICK_MID);
    //send to ctlcomms (but might not be 3 actuators)
    for (int i = 0;i<NUMTORQUERS;i++) { //NUMSIGNALS is set in params.h
      control_matrix.set(i+1,1,desired_moments.get(i+1,1));
    }
  } 
  //control_matrix.disp();
  //printf("SUM = %lf \n",ctlcomms.abssum());
}
