#include "modeling.h"

//Constructor
modeling::modeling() {
}

//Initialization 
void modeling::init(char root_folder_name[],MATLAB in_simulation_matrix,MATLAB in_configuration_matrix,int argc,char** argv) {
  printf("Modeling Root Folder Name = %s \n",root_folder_name);
  //in_simulation_matrix.disp();

  ////////EXTRACT SIMULATION MATRIX VARIABLES
  TFINAL=in_simulation_matrix.get(1,1);
  TIMESTEP=in_simulation_matrix.get(2,1);
  integrationTime=0;

  //Initialize Matrices
  NUMVARS = 30; //Make sure this is the same as the sense states+1
  model_matrix.zeros(NUMVARS,1,"Model Matrix"); 
  output_matrix.zeros(NUMVARS-1,1,"OUTPUT Matrix"); //-1 for quaternions

  //Get number of actuators
  NUMACTUATORS = in_simulation_matrix.get(39,1);
  pwmnames = (char**)malloc((NUMACTUATORS)*sizeof(char*));
  for (int i = 1;i<=NUMACTUATORS;i++) {
    pwmnames[i-1] = (char*)malloc((11)*sizeof(char));
    sprintf(pwmnames[i-1],"PWM Model %d",i);
  }
  pwm_dynamics_array = (int *) calloc(NUMACTUATORS,sizeof(int));

  NUMINTEGRATIONSTATES=13+NUMACTUATORS; //Only integrating 13 states for a 6DOF system + actuators
  integration_matrix.zeros(NUMINTEGRATIONSTATES,1,"Integration Matrix");
  settling_time_matrix.zeros(NUMACTUATORS,1,"settling time matrix");
  actuatorStates.zeros(NUMACTUATORS,1,"actuatorStates");
  //Set initial conditions of integration matrix
  for (int i = 1;i<=NUMINTEGRATIONSTATES-NUMACTUATORS;i++) {
    integration_matrix.set(i,1,in_simulation_matrix.get(i+2,1));
  }
  //Initial conditions of actuators are in a different spot
  int counter = 39 + NUMACTUATORS + 1;
  for (int i = 14;i<=NUMINTEGRATIONSTATES;i++) {
    integration_matrix.set(i,1,in_simulation_matrix.get(counter,1));
    actuatorStates.set(i-13,1,in_simulation_matrix.get(counter,1));
    settling_time_matrix.set(i-13,1,in_simulation_matrix.get(counter-NUMACTUATORS,1));
    counter = counter + 1;
  }

  //Get log rate
  LOGRATE = in_configuration_matrix.get(2,1);
  //Set names of headers
  headernames = (char**)malloc((NUMVARS-1)*sizeof(char*)); //-1 because of quaternions
  headernames[0] = "X(m)"; /// set(1,1);
  headernames[1] = "Y(m)";
  headernames[2] = "Z(m)";
  headernames[3] = "Roll (deg)";
  headernames[4] = "Pitch (deg)";
  headernames[5] = "Yaw (deg)";
  headernames[6] = "U(m/s)";
  headernames[7] = "V(m/s)";
  headernames[8] = "W(m/s)";
  headernames[9] = "P(rad/s)";
  headernames[10] = "Q(rad/s)";
  headernames[11] = "R(rad/s)";
  headernames[12] = "Mx(Gauss)";
  headernames[13] = "My(Gauss)";
  headernames[14] = "Mz(Gauss)";
  headernames[15] = "GPS Latitude (deg)"; //set(17,1); +2 (+1 for C++ and then +1 for quaternions)
  headernames[16] = "GPS Longitude (deg)";
  headernames[17] = "GPS Altitude (m)";
  headernames[18] = "GPS Heading (deg)";
  headernames[19] = "IMU Heading (deg)";
  headernames[20] = "Analog 1 (V)";
  headernames[21] = "Analog 2 (V)";
  headernames[22] = "Analog 3 (V)";
  headernames[23] = "Analog 4 (V)";
  headernames[24] = "Analog 5 (V)";
  headernames[25] = "Analog 6 (V)";
  headernames[26] = "Pressure (Pa)";  //set(28,1);
  headernames[27] = "Pressure Altitude (m)"; //set(29,1);
  headernames[28] = "Temperature (C)";

  //Initialize Logger
  logger.init("logs/",NUMVARS+NUMACTUATORS); //Not minus 1 because you add time
  //Set and log headers
  logger.appendheader("Time (sec)");
  logger.appendheaders(headernames,NUMVARS-1); //-1 because of quaternions
  logger.appendheaders(pwmnames,NUMACTUATORS);
  logger.printheaders();

  //Compute the bias in the actuators
  IACTUATORERROR = in_simulation_matrix.get(37,1);
  ACTUATORPERCENTERROR = in_simulation_matrix.get(38,1);
  pwm_error.zeros(NUMACTUATORS,1,"pwm percent error");
  for (int i = 1;i<=NUMACTUATORS;i++) {
    double sign = rand() % 100 + 1;
    if (sign > 50) { 
      sign = 1;
    } else {
      sign = -1;
    }
    pwm_error.set(i,1,ACTUATORPERCENTERROR*sign);
  }
  //pwm_error.disp();
  //PAUSE();

  //Get Mass and Inertia parameters
  mass = in_configuration_matrix.get(12,1);
  I.zeros(3,3,"Inertia");
  I.set(1,1,in_configuration_matrix.get(13,1));
  I.set(2,2,in_configuration_matrix.get(14,1));
  I.set(3,3,in_configuration_matrix.get(15,1));
  Iinv.zeros(3,3,"Inverse Inertia");
  Iinv.overwrite(I);
  Iinv.inverse();

  //Initialize the environment
  env.setMass(mass);
  env.init(in_simulation_matrix);

  //Initialize the external force model
  FORCES_FLAG = in_simulation_matrix.get(16,1);
  
  //integration_matrix.disp();
  //PAUSE();

  //Copy the states over to the model matrix for opengl and hardware loops
  model_matrix.vecset(1,NUMINTEGRATIONSTATES-NUMACTUATORS,integration_matrix,1);

  //Initialize X and Y Origin of GPS
  X_origin = MOBX;
  Y_origin = MOBY;
  //And then set GPS coordinates
  SetGPS();

  //Initialize Integrator
  integrator.init(NUMINTEGRATIONSTATES,TIMESTEP);
  //Then initialize the Initial conditions
  integrator.set_ICs(integration_matrix);

  //6DOF VARS
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

  //Kick off the render loop in its own thread if open gl is on
  #ifdef OPENGL_H
  printf("Kicking off OpenGL \n");
  boost::thread render(renderloop,root_folder_name,argc,argv);
  //Wait for the opengl routine to actually start
  while (glhandle_g.ready == 0) {
    cross_sleep(1);
  }
  #endif

}

///render loop if OPENGL is on
#ifdef OPENGL_H
void renderloop(char* root_folder_name,int argc,char** argv) {
  int Farplane = 10000;
  int width = 600;
  int height = 600;
  glhandle_g.loop(argc,argv,root_folder_name,Farplane,width,height);
}
#endif

void modeling::SetGPS() {
  X = model_matrix.get(1,1);
  Y = model_matrix.get(2,1);
  Z = model_matrix.get(3,1);
  //printf("X Y Z = %lf %lf %lf \n",model_matrix.get(1,1),model_matrix.get(2,1),model_matrix.get(3,1));  
  XYZ[0] = X;
  XYZ[1] = Y;
  XYZ[2] = Z;
  ConvertXYZ2LLH(XYZ,LLH,X_origin,Y_origin);
  latitude = LLH[0];
  longitude = LLH[1];
  altitude = LLH[2];
  //printf("LLH = %lf %lf %lf \n",latitude,longitude,altitude);
  model_matrix.set(17,1,latitude); //17 because model_matrix has quaternions
  model_matrix.set(18,1,longitude);
  model_matrix.set(19,1,altitude);
}

///Loop
void modeling::loop(double currentTime,int pwm_array[]) {

  //Check to see if we're integrating too fast
  if (currentTime < integrationTime) {
    //if this loop is true we need to break prematurely because if not we will 
    //integrate faster than the real time clock
    return;
  }

  //printf("Modeling Loop \n");
  if (currentTime > TFINAL) {
    //Simulation is over.
    ok = 0;
    //break
    return;
  }
  //integration_matrix.disp();

  //Before we send the pwm_array to the integrator we need to add some bias
  //If IACTUATORERROR IS ON
  if (IACTUATORERROR) {
    for (int i = 0;i<NUMACTUATORS;i++){
      pwm_array[i]=pwm_array[i]*(100+pwm_error.get(i+1,1))/100;
    }
  }

  //Run RK4 Loop
  rk4step(currentTime,pwm_array);

  //Reset the Integration matrix
  integration_matrix.overwrite(integrator.State);
  //Copy over actuators
  if (IACTUATORERROR) {
    actuatorStates.vecset(1,NUMACTUATORS,integrator.State,14);
  }

  //Copy the states over to the model matrix for opengl and hardware loops
  model_matrix.vecset(1,NUMINTEGRATIONSTATES-NUMACTUATORS,integration_matrix,1);

  //Convert XYZ to latitude longitude altitude and put into model_matrix.
  SetGPS();

  //Set pressure
  double pressure = ConvertZ2Pressure(model_matrix.get(3,1));
  model_matrix.set(28,1,pressure);
  //Just set pressure altitude to the actual Z coordinate
  model_matrix.set(29,1,-model_matrix.get(3,1)); 

  //Send the model matrix to opengl
  #ifdef OPENGL_H
  GLmutex.lock();
  if (glhandle_g.ready == 1) {
    glhandle_g.state.UpdateRender(currentTime,cg,ptp,1,keyboardVars);
    //printf("Key = ");
    //for (int i = 0;i<4;i++) {
    //printf("%lf ",keyboardVars[i]);
    //}
    //printf("\n");
  } else {
    printf("GL Handle Closed \n");
    GLmutex.unlock();
    //pthread_cancel(render.native_handle());
    exit(1);
  }
  GLmutex.unlock();
  #endif

  //Log data if needed
  if (currentTime >= nextLOGtime) {
    //printf("Model Logging %lf \n",currentTime);
    logger.printvar(currentTime);
    //Need to move model matrix over to output matrix
    //First grab x,y,z
    output_matrix.vecset(1,3,model_matrix,1);
    //Then copy ptp
    //ptp.disp();
    for (int i = 1;i<4;i++) {
      output_matrix.set(i+3,1,ptp.get(i,1)*180/PI);
    }
    //output_matrix.vecset(4,6,ptp,1);
    //Then the rest of the matrix
    output_matrix.vecset(7,NUMVARS-1,model_matrix,8);
    //output_matrix.disp();
    logger.print(output_matrix);
    //Then output the pwm array
    //logger.printarrayln(pwm_array,NUMACTUATORS);
    logger.println(actuatorStates);
    nextLOGtime=currentTime+LOGRATE;
  }

  //increment integration time
  integrationTime+=TIMESTEP;
}

void modeling::rk4step(double currentTime,int pwm_array[]) {
  //Integrate one timestep by running a 4 step RK4 integrator
  for (int i = 1;i<=4;i++){
    Derivatives(currentTime,pwm_array);
    integrator.integrate(i);
  }
}


void modeling::Derivatives(double currentTime,int pwm_array[]) {
  //Actuator Dynamics
  double time_constant = 0;
  double settling_time = 0;
  double actuatorDot = 0;
  for (int i = 0;i<NUMACTUATORS;i++) {  
    settling_time = settling_time_matrix.get(i+1,1);
    if ((settling_time == 0) || (IACTUATORERROR == 0)) {
      //Pass through
      actuatorStates.set(i+1,1,pwm_array[i]);
    } else {
      //Integrate
      time_constant = 4.0/settling_time;
      actuatorDot = time_constant*(pwm_array[i] - actuatorStates.get(i+1,1));
      //printf("i = %d, actuatorDot = %lf \n",actuatorDot);
      integrator.k.set(13+i+1,1,actuatorDot);
    }
    pwm_dynamics_array[i] = actuatorStates.get(i+1,1);
  }
  //printf("pwm_array Derivatives = %d \n",pwm_array[2]);
  //printf("actuatorStates = %lf \n",actuatorStates.get(3,1));

  //The Derivatives are vehicle independent except for the 
  //forces and moments
  ////////////////////KINEMATICS///////////////

  //Extract individual States from State Vector
  cg.vecset(1,3,integrator.StateDel,1);
  cgdotB.vecset(1,3,integrator.StateDel,8); //uvw
  q0123.vecset(1,4,integrator.StateDel,4);
  pqr.vecset(1,3,integrator.StateDel,11);
  //This is used to rotate things from body to inertial and vice versa
  ine2bod321.L321(q0123, 1);
  //Set ptp
  ine2bod321.getptp(ptp);

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
  //if (q*180/PI > 100){ 
  //  pqr.disp();
  //}
  double r = pqr.get(3,1);
  integrator.k.set(4,1,(-p*q1-q*q2-r*q3)/2.0);
  integrator.k.set(5,1,(p*q0+r*q2-q*q3)/2.0);
  integrator.k.set(6,1,(q*q0-r*q1+p*q3)/2.0);
  integrator.k.set(7,1,(r*q0+q*q1-p*q2)/2.0);

  ////////////////FORCE AND MOMENT MODEL///////////////////////

  //Gravity Model and Magnetic Field model
  env.gravitymodel(integrator.StateDel);
  env.groundcontactmodel(integrator.StateDel,integrator.k);
  env.getCurrentMagnetic(currentTime,integrator.StateDel);
  //The getCurrentMagnetic routine populates env.BVECINE which is the magnetometer
  //value in the inertial frame. we need to rotate this to the body frame and
  //convert to teslas
  ine2bod321.rotateInertial2Body(BVECB,env.BVECINE);
  BVECB_Tesla.overwrite(BVECB);
  BVECB_Tesla.mult_eq(1e-9);
  //Send to environment model
  env.BVECB_Tesla.overwrite(BVECB_Tesla);

  //External Forces Model
  //Send the external forces model the actuator_state instead of the ctlcomms
  if (FORCES_FLAG) {
    //printf("RUNNING FORCE MODEL \n");
    extforces.ForceMoment(currentTime,integrator.StateDel,integrator.k,pwm_dynamics_array,env);
  } else {
    extforces.FB.mult_eq(0); //Zero these out just to make sure something is in here
    extforces.MB.mult_eq(0);
  }
  
  //Add Up Forces and Moments
  //env.FGRAVI.disp();
  FTOTALI.overwrite(env.FGRAVI); //add gravity force
  //FTOTALI.disp();

  //Rotate Forces to body frame
  ine2bod321.rotateInertial2Body(FGNDB,env.FGNDI);
  ine2bod321.rotateInertial2Body(FTOTALB,FTOTALI);
  //FTOTALB.disp();

  //Add External Forces and Moments
  //FTOTALB.disp();
  //env.FGRAVI.disp();
  FTOTALB.plus_eq(FGNDB);
  //FTOTALB.disp();
  //Only add external forces if FGNDB.norm is zero
  //WAIT WHY IS THIS HERE?????
  #if not defined (car) || (tank)
  if (FGNDB.norm() == 0) {
    FTOTALB.plus_eq(extforces.FB);
  }
  #else
  //Ok need to add these in no matter what if car and tank around
  FTOTALB.plus_eq(extforces.FB);
  #endif
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
  ine2bod321.rotateInertial2Body(MGNDB,env.MGNDI);
  //MGNDB.disp();
  //PAUSE();

  //Moments vector
  MTOTALB.mult_eq(0);
  MTOTALB.overwrite(MGNDB);
  #if not defined (car) || (tank)
  if (FGNDB.norm() == 0) {
    MTOTALB.plus_eq(extforces.MB);
  }
  #else
  MTOTALB.plus_eq(extforces.MB);
  #endif
  //MTOTALB.disp();
  //pqr.disp();
  //PAUSE();

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

  //integrator.k.disp();
  ////////////////////////////////////////////////////////////////
}
