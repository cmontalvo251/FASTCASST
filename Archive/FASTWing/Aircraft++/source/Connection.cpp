#include "Connection.h"
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include "Vehicle.h"

using namespace std;

//Constructor
Connection::Connection()
{
}

int Connection::init(int num_aircraft,char* argv[]) {
  cout << "Simulating Meta Configuration" << endl;
  ///Alright we know how many aircraft we need to simulate so loop through those first
  NUMVEHICLES = num_aircraft;
  int ok = 0;
  for (int i = 0;i<num_aircraft;i++) {
    printf("Creating Vehicle(%d) using Input File: %s \n",i+1,argv[i+1]);
    vehicles[i].init(argv[i+1]);
    ok += vehicles[i].ok;
  }

  ///Let's just for haste hard code the connection file to be 
  FILE * connfile = fopen("Input_Files/A++.CONN","r");
  if (connfile) {
    //First read the number of connections
    char dummy[256];
    fscanf(connfile,"%d %s ",&NUMCONNECTIONS,dummy);
    cout << "These Aircraft have " << NUMCONNECTIONS << " connections" << endl;
    //Once I know how many connections there are I can first read the connection matrix
    for (int i = 0;i<NUMCONNECTIONS;i++) {
      fscanf(connfile,"%d %d %s",&ICONN[0][i],&ICONN[1][i],dummy);
      cout << "Aircraft " << ICONN[0][i] << " is connected to Aircraft " << ICONN[1][i] << endl;
      //Then read the connection point
      //First initialize the two MATLAB arrays
      RCONN[2*i].zeros(3,1,"Aircraft Connection Joint A");
      RCONN[2*i+1].zeros(3,1,"Aircraft Connection Joint B");
      double x,y,z;
      fscanf(connfile,"%lf %lf %lf %s",&x,&y,&z,dummy);
      RCONN[2*i].set(1,1,x);
      RCONN[2*i].set(2,1,y);
      RCONN[2*i].set(3,1,z);
      fscanf(connfile,"%lf %lf %lf %s",&x,&y,&z,dummy);
      RCONN[2*i+1].set(1,1,x);
      RCONN[2*i+1].set(2,1,y);
      RCONN[2*i+1].set(3,1,z);
      cout << "Location of Connection Point on Both Aircraft:" << endl;
      RCONN[2*i].disp();
      RCONN[2*i+1].disp();
    }
    //Once we've read that we need to read our stiffness and damping parameters
    fscanf(connfile,"%lf %lf %s",&KLINEAR,&CLINEAR,dummy);
    fscanf(connfile,"%lf %lf %s",&KROTATIONAL,&CROTATIONAL,dummy);
  } else {
    cout << "Error Reading Connection File" << endl;
    return 0;
  }

  //Create vectors for connection point stuff
  rcg1.zeros(3,1,"cg1");
  rcg2.zeros(3,1,"cg2");
  rcgI1.zeros(3,1,"rcg1I");
  rcgI2.zeros(3,1,"rcg2I");
  wr1.zeros(3,1,"w x r 1");
  wr2.zeros(3,1,"w x r 2");
  wrI1.zeros(3,1,"w x r 1 Inertial");
  wrI2.zeros(3,1,"w x r 2 Inertial");
  conndot1.zeros(3,1,"Connection Point Velocity 1");
  conndot2.zeros(3,1,"Connection Point Velocity 2");
  ptpdiff.zeros(3,1,"ptp deltas");
  ptpdotdiff.zeros(3,1,"ptpdot deltas");
  FCONN.zeros(3,1,"FCONN");
  MCONN.zeros(3,1,"MCONN");
  rF1.zeros(3,1,"r x F 1");
  rF2.zeros(3,1,"r x F 2");  
  return ok;
}

void Connection::ConnectionForce() {
  ///Run through all vehicles and compute the connection force
  ///Ok in order to do this we first need some sort of connection matrix.
  //Like 1 is connect to 2
  //So we first need a ICONN matrix which has 2 rows (the two bodies that are connected) X NUMCONNECTIONS
  //And then an RCONN matrix that is 3 row (x,y,z in body frame) x (2XNUMCONNECTIONS)
  //Alright so all that is setup in the init routine
  //We can now loop through the number of connections
  for (int i = 0;i<NUMCONNECTIONS;i++) {
    //cout << "Aicraft " << ICONN[0][i] << " is connected to " << ICONN[1][i] << endl;
    //cout << "The state of both aircraft is:" << endl;
    //vehicles[ICONN[0][i]-1].rkstate.disp();
    //vehicles[ICONN[1][i]-1].rkstate.disp();
    //cout << "The vector from cg to connection point in body frame is:" << endl;
    //RCONN[2*i].disp();
    //RCONN[2*i+1].disp();
    ///^^^All that above is just debugging stuff
    //What we really need is the location of the wingtip
    //First get the center of masses
    rcg1.set(1,1,vehicles[ICONN[0][i]-1].rkstate.get(1,1));
    rcg1.set(2,1,vehicles[ICONN[0][i]-1].rkstate.get(2,1));
    rcg1.set(3,1,vehicles[ICONN[0][i]-1].rkstate.get(3,1));
    rcg2.set(1,1,vehicles[ICONN[1][i]-1].rkstate.get(1,1));
    rcg2.set(2,1,vehicles[ICONN[1][i]-1].rkstate.get(2,1));
    rcg2.set(3,1,vehicles[ICONN[1][i]-1].rkstate.get(3,1));
    //rcg1.disp();
    //rcg2.disp();
    //Then Rotate the RCONN vectors to the inertial frame
    vehicles[ICONN[0][i]-1].ine2bod123.rotateBody2Inertial(rcgI1,RCONN[2*i]);
    vehicles[ICONN[1][i]-1].ine2bod123.rotateBody2Inertial(rcgI2,RCONN[2*i+1]);
    //rcgI1.disp();
    //rcgI2.disp();
    //Then just to save resources we will take the cg and add the vector from cg to wingtip to it
    rcg1.plus_eq(rcgI1);
    rcg2.plus_eq(rcgI2);
    //rcg1.disp();
    //rcg2.disp();
    //Then finally once again to save resources we will subsctract one from the other
    rcg1.minus_eq(rcg2);
    //rcg1.disp();
    //These are our deltas......crap. We need the velocities too.
    //Alright so we do the same but kinda different. We need xyzdot. Since we ran the derivatives routine
    //I think we have xyzdot already. Let's check
    //vehicles[ICONN[0][i]-1].xyzdot.disp();
    //vehicles[ICONN[0][i]-1].xyzdot.disp();
    //Yes. So we just need to do
    //Vp = VB + w x r
    //But w x r is in the body frame so we need to compute and then rotate.
    wr1.cross(vehicles[ICONN[0][i]-1].pqr,RCONN[2*i]);
    wr2.cross(vehicles[ICONN[1][i]-1].pqr,RCONN[2*i+1]);
    //Then we rotate
    vehicles[ICONN[0][i]-1].ine2bod123.rotateBody2Inertial(wrI1,wr1);
    vehicles[ICONN[1][i]-1].ine2bod123.rotateBody2Inertial(wrI2,wr2);
    //Then we add xyzdot + wrI
    conndot1.plus(vehicles[ICONN[0][i]-1].xyzdot,wrI1);
    conndot2.plus(vehicles[ICONN[1][i]-1].xyzdot,wrI2);
    //Then to save resources we substract 1 from the other
    conndot1.minus_eq(conndot2);
    //conndot1.disp();
    //Ok finally we have our deltas rcg1 and conndot1
    //So......fuck. We need our ptp delta and ptpdot deltas but ptp and ptpdot exists so we just need a vector
    //to get the subsctraction.
    ptpdiff.minus(vehicles[ICONN[0][i]-1].ptp,vehicles[ICONN[1][i]-1].ptp);
    ptpdotdiff.minus(vehicles[ICONN[0][i]-1].ptpdot,vehicles[ICONN[1][i]-1].ptpdot);

    //Ok! Finally! So now we compute Fconn
    FCONN.overwrite(rcg1);
    FCONN.mult_eq(-KLINEAR);
    FCONN.plus_mult_eq(conndot1,-CLINEAR);
    //FCONN.mult_eq(0);
    //and MCONN
    MCONN.overwrite(ptpdiff);
    MCONN.mult_eq(-KROTATIONAL);
    MCONN.plus_mult_eq(ptpdotdiff,-CROTATIONAL);

    //Ok ok ok. One more thing. We need to factor in M = r x F
    //We already have rcgI1 so we just need to compute rF1 and rF2
    rF1.cross(rcgI1,FCONN);
    rF2.cross(rcgI2,FCONN);

    //Then we just add rF1 and rF2 to MCONN.. Wait no. I have to do that for each aircraft

    //MCONN.mult_eq(0);
    //Then we must inject these two forces and moments into the simulation
    vehicles[ICONN[0][i]-1].ForceConnection.overwrite(FCONN);
    vehicles[ICONN[1][i]-1].ForceConnection.overwrite(FCONN);
    vehicles[ICONN[1][i]-1].ForceConnection.mult_eq(-1); //The second aircraft experiences the opposite force
    
    vehicles[ICONN[0][i]-1].MomentConnection.overwrite(MCONN);
    vehicles[ICONN[0][i]-1].MomentConnection.plus_eq(rF1);

    vehicles[ICONN[1][i]-1].MomentConnection.overwrite(MCONN);
    vehicles[ICONN[1][i]-1].MomentConnection.plus_eq(rF2);
    vehicles[ICONN[1][i]-1].MomentConnection.mult_eq(-1); //The second aircraft experiences the opposite force


  }
}

void Connection::Integrate() {

  //In order to run the RK4 engine we need the simdata
  //block which we will default to the first vehicle
  double tinitial = vehicles[0].simdata.get(1,1);
  double tfinal = vehicles[0].simdata.get(2,1);
  double timestep = vehicles[0].simdata.get(3,1);

  //We don't need to make a time vector in C++. It's a waste of resources.
  //Honestly it's probably a waste of resource in all the other languages
  //too. But whatever. 
  double t = tinitial;

  //For c++ we have to open up an outfile. I'll just use the old school way.
  //printf("Opening State History File \n");
  FILE* outfile[NUMVEHICLES];
  char filename[100];
  for (int i = 0;i<NUMVEHICLES;i++){
    sprintf(filename,"%s%d%s","Output_Files/C",i+1,"++.OUT");
    outfile[i] = fopen(filename,"w");
    if (outfile[i]) {
      printf("Output File Succesfully Opened: %s \n",filename);
    } else {
      printf("Couldn't open output file: %s \n",filename);
      return;
    }
    //In this loop we will also run a setup routine
    vehicles[i].setup(timestep);
  }
  
  cout << "Meta Integration Begin" << endl;

  double tthresh = tinitial;
  double tnext = 10;
  
  while (t <= tfinal) {
    if (t > tthresh) {
      cout << "Time = " << t << endl;
      tthresh+=tnext;
    }

    //Output contents to file.
    for (int i = 0;i<NUMVEHICLES;i++) {
      fprintf(outfile[i],"%lf ",t);
      //cout << "state - outputting to file " << endl;
      vehicles[i].state.vecfprintf(outfile[i]);
      //cout << "aero - outputting to file " << endl;
      vehicles[i].aero.print_to_file(outfile[i]);
      fprintf(outfile[i],"\n");

      //We will also put state into rkstate
      vehicles[i].rkstate.overwrite(vehicles[i].state);
      //And call the derivatives routine just to initialize a few variables
      vehicles[i].Derivatives(vehicles[i].k1,t,vehicles[i].state);
    }

    //RK4 Calls
    //cout <<"First call to derivatives" << endl;

    ///COMPUTE CONNECTION FORCES USING vehicles[].rkstate
    ConnectionForce();

    //FIRST DERIVATIVE CALL
    for (int i = 0;i<NUMVEHICLES;i++) {
      vehicles[i].Derivatives(vehicles[i].k1,t,vehicles[i].state);
      vehicles[i].rkstate.plus_mult_eq(vehicles[i].k1,timestep/2.0);
    }

    ////UPDATE CONNECTION FORCES using vehicles[].rkstate
    ConnectionForce();

    ///SECOND DERIVATIVE CALL
    for (int i = 0;i<NUMVEHICLES;i++) {
      vehicles[i].Derivatives(vehicles[i].k2,t+timestep/2.0,vehicles[i].rkstate);
      vehicles[i].rkstate.overwrite(vehicles[i].state);
      vehicles[i].rkstate.plus_mult_eq(vehicles[i].k2,timestep/2.0);
    }

    ////UPDATE CONNECTION FORCES using vehicles[].rkstate
    ConnectionForce();

    ///THIRD DERIVATIVE CALL
    for (int i = 0;i<NUMVEHICLES;i++) {
      vehicles[i].Derivatives(vehicles[i].k3,t+timestep/2.0,vehicles[i].rkstate);
      vehicles[i].rkstate.overwrite(vehicles[i].state);
      vehicles[i].rkstate.plus_mult_eq(vehicles[i].k3,timestep);
    }

    ////UPDATE CONNECTION FORCES using vehicles[].rkstate
    ConnectionForce();

    //FINAL DERIVATIVE CALL
    for (int i = 0;i<NUMVEHICLES;i++) {
      vehicles[i].Derivatives(vehicles[i].k4,t+timestep,vehicles[i].rkstate);
      //Then step state in the same for loop
      vehicles[i].stepstate(timestep);
    } 

    t+=timestep;
  }

  //Close all output files
  for (int i = 0;i<NUMVEHICLES;i++){
    fclose(outfile[i]);
  }
  
  cout << "Meta Integration End" << endl;
}
