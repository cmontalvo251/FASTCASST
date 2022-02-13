#include "modeling.h"

//Constructor
modeling::modeling() {
}

//Initialization 
void modeling::init(char root_folder_name[],MATLAB in_simulation_matrix,MATLAB in_configuration_matrix) {
  printf("Modeling Root Folder Name = %s \n",root_folder_name);
  //in_simulation_matrix.disp();

  ////////EXTRACT SIMULATION MATRIX VARIABLES
  TFINAL=in_simulation_matrix.get(1,1);
  TIMESTEP=in_simulation_matrix.get(2,1);

  //Initialize Matrices
  NUMVARS = 30; //Make sure this is the same as the sense states
  model_matrix.zeros(NUMVARS,1,"Model Matrix"); 
  NUMINTEGRATIONSTATES=13; //Only integrating 13 states for a 6DOF system
  integration_matrix.zeros(NUMINTEGRATIONSTATES,1,"Integration Matrix");

  //Set initial conditions of integration matrix
  for (int i = 1;i<NUMINTEGRATIONSTATES;i++) {
    integration_matrix.set(i,1,in_simulation_matrix.get(i+2,1));
  }

  //Get log rate
  LOGRATE = in_configuration_matrix.get(2,1);
  //Set names of headers
  headernames = (char**)malloc(NUMVARS*sizeof(char*));
  headernames[0] = "X(m)";
  headernames[1] = "Y(m)";
  headernames[2] = "Z(m)";
  headernames[3] = "Q0";
  headernames[4] = "Q1";
  headernames[5] = "Q2";
  headernames[6] = "Q3";
  headernames[7] = "U(m/s)";
  headernames[8] = "V(m/s)";
  headernames[9] = "W(m/s)";
  headernames[10] = "P(deg/s)";
  headernames[11] = "Q(deg/s)";
  headernames[12] = "R(deg/s)";
  headernames[13] = "Mx(Gauss)";
  headernames[14] = "My(Gauss)";
  headernames[15] = "Mz(Gauss)";
  headernames[16] = "GPS Latitude (deg)";
  headernames[17] = "GPS Longitude (deg)";
  headernames[18] = "GPS Altitude (m)";
  headernames[19] = "GPS Heading (deg)";
  headernames[20] = "IMU Heading (deg)";
  headernames[21] = "Analog 1 (V)";
  headernames[22] = "Analog 2 (V)";
  headernames[23] = "Analog 3 (V)";
  headernames[24] = "Analog 4 (V)";
  headernames[25] = "Analog 5 (V)";
  headernames[26] = "Analog 6 (V)";
  headernames[27] = "Pressure (Pa)";
  headernames[28] = "Pressure Altitude (m)";
  headernames[29] = "Temperature (C)";
  //Initialize Logger
  logger.init("logs/",NUMVARS);
  //Set and log headers
  logger.appendheader("Time (sec)");
  logger.appendheaders(headernames,NUMVARS);
  logger.printheaders();

  //Get Mass and Inertia parameters
  mass = in_configuration_matrix.get(8,1);
  I.zeros(3,3,"Inertia");
  I.set(1,1,in_configuration_matrix.get(9,1));
  I.set(2,2,in_configuration_matrix.get(10,1));
  I.set(3,3,in_configuration_matrix.get(11,1));
  Iinv.zeros(3,3,"Inverse Inertia");
  Iinv.overwrite(I);
  Iinv.inverse();
  
  //integration_matrix.disp();
  //PAUSE();

  //Copy the states over to the model matrix for opengl and hardware loops
  model_matrix.vecset(1,NUMINTEGRATIONSTATES,integration_matrix,1);

  //Initialize Integrator
  integrator.init(NUMINTEGRATIONSTATES,TIMESTEP);
  //Then initialize the Initial conditions
  integrator.set_ICs(integration_matrix);

  //6DOF VARS
  //Always create these just because I don't want to think about
  //when we actually need them and it's really easy to create them
  cg.zeros(3,1,"Center of Mass");
  ptp.zeros(3,1,"Roll Pitch Yaw");
  FTOTALI.zeros(3,1,"Total Forces Inertial Frame");
  q0123.zeros(4,1,"Quaternions");
  cgdotI.zeros(3,1,"Velocity Inertial");
  cgdotB.zeros(3,1,"Velocity Body Frame");
  ptpdot.zeros(3,1,"Euler Derivatives");
  pqr.zeros(3,1,"Angular Velocity Body Frame");
  pqrdot.zeros(3,1,"Derivative of Angular Velocity");
  uvwdot.zeros(3,1,"Derivaitves of Velocity Body Frame");
  FGNDB.zeros(3,1,"Ground Forces Body Frame");
  FTOTALB.zeros(3,1,"Total Forces Body Frame");
  MTOTALI.zeros(3,1,"Total Moments Inertial Frame");
  MTOTALB.zeros(3,1,"Total Moments Body Frame");
  MGNDB.zeros(3,1,"Ground Moments Body Frame");
  I_pqr.zeros(3,1,"I times pqr");
  pqrskew_I_pqr.zeros(3,1,"pqr cross I times pqr");
  Kuvw_pqr.zeros(3,1,"uvw cross pqr");
  BVECB.zeros(3,1,"Body Frame Magnetic Field (nT)");
  BVECB_Tesla.zeros(3,1,"Body Frame Magnetic Field (Teslas)");
}

///Loop
void modeling::loop(double currentTime,int pwm_array[]) {
  //printf("Modeling Loop \n");
  if (currentTime > TFINAL) {
    //Simulation is over.
    ok = 0;
    //break
    return;
  }
  //integration_matrix.disp();

  //Run RK4 Loop
  rk4step(currentTime,pwm_array);

  //Reset the Integration matrix
  integration_matrix.overwrite(integrator.State);

  //Copy the states over to the model matrix for opengl and hardware loops
  model_matrix.vecset(1,NUMINTEGRATIONSTATES,integration_matrix,1);

  //Add sensor noise if needed

  //Note we have a bunch more variables that aren't included in the integration
  //states. GPS LLH is an example. 

  //Log data if needed
  if (currentTime > nextLOGtime) {
    logger.printvar(currentTime);
    logger.println(model_matrix);
    nextLOGtime=currentTime+LOGRATE;
  }

}

void modeling::rk4step(double currentTime,int pwm_array[]) {
  //Integrate one timestep by running a 4 step RK4 integrator
  for (int i = 1;i<=4;i++){
    Derivatives(currentTime,pwm_array);
    integrator.integrate(i);
  }
}


void modeling::Derivatives(double currentTime,int pwm_array[]) {
  //The Derivatives are vehicle independent except for the 
  //forces and moments

  //Actuator Error Model is a simple first order filter
  /*if (NUMACTUATORS > 0) {
    ///Get the error actuator state
    double val = 0;
    for (int i = 0;i<NUMACTUATORS;i++) {  
      val = actuatorState.get(i+1,1)*actuatorErrorPercentage.get(i+1,1);
      actuatorError.set(i+1,1,val);  
    }
    //Integrate Actuator Dynamics
    //input will be ctlcomms and the output will be actuator_state
    for (int i = 0;i<NUMACTUATORS;i++) {
      k.set(i+NUMSTATES+1,1,actuatorTimeConstants.get(i+1,1)*(rcout.pwmcomms[i] - actuatorState.get(i+1,1)));
    }
  } else {
    //Otherwise just pass through the ctlcomms
    actuatorError.overwrite(ctl.ctlcomms);
    }*/

  ////////////////////KINEMATICS///////////////

  //Extract individual States from State Vector
  cgdotB.vecset(1,3,integrator.StateDel,8); //uvw
  q0123.vecset(1,4,integrator.StateDel,4);
  pqr.vecset(1,3,integrator.StateDel,11);
  //This is used to rotate things from body to inertial and vice versa
  ine2bod321.L321(q0123, 1);

  ///Linear Kinematics (xyzdot = TIB uvw)
  ine2bod321.rotateBody2Inertial(cgdotI,cgdotB);
  integrator.k.vecset(1,3,cgdotI,1);

  ///Rotational Kinematics (Quaternion Derivatives)
  double q0 = q0123.get(1,1);
  double q1 = q0123.get(2,1);
  double q2 = q0123.get(3,1);
  double q3 = q0123.get(4,1);
  double p = pqr.get(1,1);
  double q = pqr.get(2,1);
  double r = pqr.get(3,1);
  integrator.k.set(4,1,(-p*q1-q*q2-r*q3)/2.0);
  integrator.k.set(5,1,(p*q0+r*q2-q*q3)/2.0);
  integrator.k.set(6,1,(q*q0-r*q1+p*q3)/2.0);
  integrator.k.set(7,1,(r*q0+q*q1-p*q2)/2.0);

  ////////////////FORCE AND MOMENT MODEL///////////////////////

  //Gravity Model and Magnetic Field model
  //env.gravitymodel(integrator.StateDel);
  //env.groundcontactmodel(integrator.StateDel,integrator.k);
  //env.getCurrentMagnetic(t,integrator.StateDel);
  //The getCurrentMagnetic routine populates env.BVECINE which is the magnetometer
  //value in the inertial frame. we need to rotate this to the body frame and
  //convert to teslas
  //ine2bod321.rotateInertial2Body(BVECB,env.BVECINE);
  //BVECB_Tesla.overwrite(BVECB);
  //BVECB_Tesla.mult_eq(1e-9);
  //Send to environment model
  //env.BVECB_Tesla.overwrite(BVECB_Tesla);

  //External Forces Model
  //Send the external forces model the actuator_state instead of the ctlcomms
  //extforces.ForceMoment(t,integrator.StateDel,integrator.k,pwmarray,env);

  //Add Up Forces and Moments
  //FTOTALI.overwrite(env.FGRAVI); //add gravity 

  //Rotate Forces to body frame
  //ine2bod321.rotateInertial2Body(FGNDB,env.FGNDI);
  //ine2bod321.rotateInertial2Body(FTOTALB,FTOTALI);

  //Add External Forces and Moments
  //FTOTALB.disp();
  //env.FGRAVI.disp();
  //FTOTALB.plus_eq(extforces.FB);
  //FTOTALB.plus_eq(FGNDB);
  //extforces.FB.disp();
  //FGNDB.disp();
  //FTOTALB.disp();  
  //if (FTOTALB.get(1,1) > 0) {
  //  PAUSE();
  //}

  //Translational Dynamics
  Kuvw_pqr.cross(pqr,cgdotB);
  FTOTALB.mult_eq(1.0/mass); 
  uvwdot.minus(FTOTALB,Kuvw_pqr); 
  integrator.k.vecset(8,10,uvwdot,1);

  //Rotate Ground Contact Friction to Body
  //env.MGNDI.disp();
  //ine2bod321.rotateInertial2Body(MGNDB,env.MGNDI);
  //MGNDB.disp();
  //PAUSE();

  //Moments vector
  //MTOTALB.overwrite(extforces.MB);
  //MTOTALB.plus_eq(MGNDB);

  ///Rotational Dynamics
  //pqrskew = [0 -r q;r 0 -p;-q p 0];  
  //pqrdot = Iinv*(LMN-pqrskew*I*pqr);
  //pqr.disp();
  I_pqr.mult(I,pqr);
  //I.disp();
  //I_pqr.disp();
  pqrskew_I_pqr.cross(pqr,I_pqr);
  //pqrskew_I_pqr.disp();
  MTOTALB.minus_eq(pqrskew_I_pqr);
  //LMN.disp();
  //Iinv.disp();
  pqrdot.mult(Iinv,MTOTALB);
  
  //Save pqrdot
  integrator.k.vecset(11,13,pqrdot,1);

  ////////////////////////////////////////////////////////////////
}
