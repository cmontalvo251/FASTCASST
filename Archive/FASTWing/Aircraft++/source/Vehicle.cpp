#include "Vehicle.h"
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace std;

//Constructor 
Vehicle::Vehicle() {
}

void Vehicle::init(char input_file[]) {
  sprintf(input_file_,input_file);
  //Once we copy the file over it's time to import the input deck
  ok = ImportFiles();
  if (ok) {
    //Once we read in the data we need to do a bit of cleanup
    mass = massdata.get(1,1);
    I.eye(3,"I");
    I.set(1,1,massdata.get(2,1));
    I.set(2,2,massdata.get(3,1));
    I.set(3,3,massdata.get(4,1));
    Iinv.zeros(3,3,"Iinv");
    Iinv.overwrite(I);
    Iinv.inverse();
    pqr.zeros(3,1,"pqr");
    I_pqr.zeros(3,1,"I_pqr");
    ptp.zeros(3,1,"ptp");
    uvw.zeros(3,1,"uvw");
    uvwdot.zeros(3,1,"uvwdot");
    pqrdot.zeros(3,1,"pqrdot");
    xyzdot.zeros(3,1,"xyzdot");
    ptpdot.zeros(3,1,"ptpdot");
    Kuvw_pqr.zeros(3,1,"Kuvw_pqr");
    pqrskew_I_pqr.zeros(3,1,"pqrskew_I_pqr");
    LMN.zeros(3,1,"LMN");
    //Create Some Derivatives Vectors
    F.zeros(3,1,"F");
    Fbody.zeros(3,1,"Fbody");
    Fgrav.zeros(3,1,"Fgrav");
    M.zeros(3,1,"M");
    ForceConnection.zeros(3,1,"ForceConnection");
    MomentConnection.zeros(3,1,"MomentConnection");
    //Phi,theta,psi in the input file are in degrees so yeah.
    icdata.set(4,1,icdata.get(4,1)*PI/180.0);
    icdata.set(5,1,icdata.get(5,1)*PI/180.0);
    icdata.set(6,1,icdata.get(6,1)*PI/180.0);
    stateinitial.copy_init(icdata,"Initial State");
    //stateinitial.disp();
    
    //cout << "Import Complete" << endl;
  }
  else {
    //cout << "Import Failed" << endl;
  }
}

int Vehicle::ImportFiles() {
  FILE* mainfile;
  mainfile = fopen(input_file_,"r");
  char simfile[256],massfile[256],icfile[256],aerofile[256];
  //cout << "Reading Input File" << endl;
  if (mainfile) {
    fscanf(mainfile,"%s \n",&simfile);
    fscanf(mainfile,"%s \n",&massfile);
    fscanf(mainfile,"%s \n",&icfile);
    fscanf(mainfile,"%s \n",&aerofile);
    //Read in all file nows
    ok = ImportFile(simfile,&simdata,"simdata",-99);
    if (ok) {
        ok = ImportFile(massfile,&massdata,"massdata",-99);
      if (ok) {
	      ok = ImportFile(icfile,&icdata,"icdata",12);
        if (ok) {
          ok = ImportFile(aerofile,&aerodata,"aerodata",-99);
        }
      } else {
	      return 0;
      }
    } else {
      return 0;
    }
    //At this point the data should be read in. Should be able to run .disp() to see the contents.
    //simdata.disp();
    //massdata.disp();
    //icdata.disp();
    fclose(mainfile);

    return 1;
  } else {
    cout << "File not found = " << input_file_ << endl;
    return 0;
  }
}

int Vehicle::ImportFile(char* filename,MATLAB* data,char* name,int length) {
  //cout << "Reading " << filename << endl;
  //Gonna open this file the old school way
  FILE *file;
  file = fopen(filename,"r");
  char dummy[256];
  if (file) {
    if (length == -99) {
      length = 0;
      while (!feof(file)) {
	fgets(dummy,256,file);
	//cout << dummy << endl;
	length++;
      }
      fclose(file);
    }
    //cout << "File length is " << length << endl;
    //Now that we know how big the file is we need to make a MATLAB vector the same size
    data->zeros(length,1,name); //Because we passed a pointer we have to use the -> instead of .		//data.disp();
    //Then we open the file with FSTREAM and throw it in there. Done. Boom.
    fstream datafile;
    datafile.open(filename);
    if (datafile.is_open()) {
      string input;
      for (int idx = 0;idx<length;idx++) {
	      getline(datafile,input);
      	data->set(idx+1,1,atof(input.c_str()));
      }
      //For debug purposes let's make sure we read everything correctly. File I/O in C++
      //is always hit or miss for me.
      //data->disp();
    } else {
      cout << "Something went wrong in FSTREAM. Maybe the file wasn't closed properly?" << endl;
      return 0;
    }
  } else {
    cout << "File not found = " << filename << endl;
    return 0;
  }
  //This code will automatically generate a MATLAB vector based on how many rows are in
  //the file. Probably need to do an FSEEK or something and then do a ZEROS call to a MATLAB
  //Array.
  //If everything checks out we just return 1
  return 1;
}

void Vehicle::setup(double timestep) {
  //Create RK4 vectors.
  k1.copy_init(stateinitial,"k1");
  k2.copy_init(stateinitial,"k2");
  k3.copy_init(stateinitial,"k3");
  k4.copy_init(stateinitial,"k4");
  state.copy_init(stateinitial,"state");
  rkstate.copy_init(stateinitial,"rkstate");
  phi.copy_init(stateinitial,"phi");

  //Send timestep and Controller parameters to aero file
  aero.dt = timestep;
  aero.ctl.CONTROLTYPE = simdata.get(4,1);
  aero.AEROTYPE = simdata.get(4,1);
  aero.setup(aerodata);
}

void Vehicle::stepstate(double timestep) {
  phi.mult_eq(0);
  phi.plus_mult_eq(k1,1.0/6.0);
  phi.plus_mult_eq(k2,2.0/6.0);
  phi.plus_mult_eq(k3,2.0/6.0);
  phi.plus_mult_eq(k4,1.0/6.0);
  //Step state
  state.plus_mult_eq(phi,timestep);
}

void Vehicle::Integrate() {

  //In order to run the RK4 engine we need the simdata
  //block.
  double tinitial = simdata.get(1,1);
  double tfinal = simdata.get(2,1);
  double timestep = simdata.get(3,1);
  //We don't need to make a time vector in C++. It's a waste of resources.
  //Honestly it's probably a waste of resource in all the other languages
  //too. But whatever. 
  double t = tinitial;

  //For c++ we have to open up an outfile. I'll just use the old school way.
  //printf("Opening State History File \n");
  FILE* outfile = fopen("Output_Files/C++.OUT","w");
  if (outfile) {
    printf("Output File Succesfully Opened \n");
  } else {
    printf("Couldn't open output file \n");
    return;
  }

  //Run a quick setup routine.
  //It may seem silly to have this routine but
  //in order to use Connection.cpp properly you need
  //this quick routine.
  setup(timestep);

  cout << "Begin Integration" << endl;

  while (t <= tfinal) {
    //cout << "Time = " << t << endl;
    //cout << "outputting to file " << endl;
    //Output contents to file.
    fprintf(outfile,"%lf ",t);
    //cout << "state - outputting to file " << endl;
    state.vecfprintf(outfile);
    //cout << "aero - outputting to file " << endl;
    aero.print_to_file(outfile);
    fprintf(outfile,"\n");
    //RK4 Calls

    //cout <<"First call to derivatives" << endl;
    Derivatives(k1,t,state);
    //void plus_mult_eq(MATLAB,double); //This is this = this + MATLAB*double
    rkstate.overwrite(state);
    rkstate.plus_mult_eq(k1,timestep/2.0);
    Derivatives(k2,t+timestep/2.0,rkstate);

    rkstate.overwrite(state);
    rkstate.plus_mult_eq(k2,timestep/2.0);
    Derivatives(k3,t+timestep/2.0,rkstate);

    rkstate.overwrite(state);
    rkstate.plus_mult_eq(k3,timestep);
    Derivatives(k4,t+timestep,rkstate);

    phi.mult_eq(0);
    phi.plus_mult_eq(k1,1.0/6.0);
    phi.plus_mult_eq(k2,2.0/6.0);
    phi.plus_mult_eq(k3,2.0/6.0);
    phi.plus_mult_eq(k4,1.0/6.0);

    //Step state
    t+=timestep;
    state.plus_mult_eq(phi,timestep);
  }
  cout << "Integration Complete" << endl;
  aero.ctl.printcost();
  fclose(outfile);
}

void Vehicle::Derivatives(MATLAB dxdt,double t,MATLAB state) {
  //double x = state.get(1,1);
  //double y = state.get(2,1);
  //double z = state.get(3,1);
  double phi = state.get(4,1);
  double theta = state.get(5,1);
  double psi = state.get(6,1);
  double u = state.get(7,1);
  double v = state.get(8,1);
  double w = state.get(9,1);
  double p = state.get(10,1);
  double q = state.get(11,1);
  double r = state.get(12,1);

  //Set up vectors
  ptp.set(1,1,phi);
  ptp.set(2,1,theta);
  ptp.set(3,1,psi);
  uvw.set(1,1,u);
  uvw.set(2,1,v);
  uvw.set(3,1,w);
  pqr.set(1,1,p);
  pqr.set(2,1,q);
  pqr.set(3,1,r);

  //Kinematics
  ine2bod123.L321(ptp,0);
  ine2bod123.rotateBody2Inertial(xyzdot,uvw);
  dxdt.vecset(1,3,xyzdot,1);
  ptpdot.mult(ine2bod123.H,pqr);
  dxdt.vecset(4,6,ptpdot,1);

  //Force Model
  ForceandMoment(state,dxdt,t);

  //F.disp();

  //Translational Dynamics
  Kuvw_pqr.cross(pqr,uvw);
  F.mult_eq(1.0/mass); //Will this work? I think so
  uvwdot.minus(F,Kuvw_pqr); ///Hey yo. We need to divide by mass here. I mean F needs to be divided by mass
  dxdt.vecset(7,9,uvwdot,1);

  //Rotational Dynamics
  //pqr.disp();
  I_pqr.mult(I,pqr);
  //I.disp();
  //I_pqr.disp();
  //PAUSE();
  pqrskew_I_pqr.cross(pqr,I_pqr);
  M.minus_eq(pqrskew_I_pqr); 
  pqrdot.mult(Iinv,M);
  dxdt.vecset(10,12,pqrdot,1);
}

void Vehicle::ForceandMoment(MATLAB state,MATLAB dxdt,double t) {
  //Gravity in the inertial frame
  Fgrav.set(1,1,0);
  Fgrav.set(2,1,0);
  Fgrav.set(3,1,GRAVITYSI*mass);

  //Add Connection to Gravity
  Fgrav.plus_eq(ForceConnection); //ForceConnection is in the Inertial Frame and must be rotated
  //So we add it to gravity

  //Aerodynamics
  aero.ForceMomentAero(t,state,dxdt);

  //Rotate Gravity to Body Frame
  ine2bod123.rotateInertial2Body(Fbody,Fgrav); 
  //Then since Force connection is part of gravity it will get rotated to the body frame

  //Add everything together
  F.mult_eq(0);
  F.plus_eq(Fbody);
  F.plus_eq(aero.Faero);

  //For moments
  M.mult_eq(0); //we will first multiply by zero
  //Then add the connection moments
  M.plus_eq(MomentConnection); //Which is in the body frame by construction
  M.plus_eq(aero.Maero);

}
