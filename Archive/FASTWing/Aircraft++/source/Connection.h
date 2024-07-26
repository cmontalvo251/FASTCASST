#ifndef CONNECTION_H
#define CONNECTION_H

#include "Vehicle.h"
#include "MATLAB.h"

#define MAXAIRCRAFT 2

class Connection {
  friend class Vehicle;
 private:
  int NUMVEHICLES,NUMCONNECTIONS;
  int ICONN[2][MAXAIRCRAFT];
  MATLAB RCONN[2*MAXAIRCRAFT],rcg1,rcg2,rcgI1,rcgI2,wr1,wr2,wrI1,wrI2,conndot1,conndot2;
  MATLAB ptpdiff,ptpdotdiff,MCONN,FCONN,rF1,rF2;
  double KLINEAR,CLINEAR,KROTATIONAL,CROTATIONAL;
  Vehicle vehicles[MAXAIRCRAFT];
 public:
  void Integrate();
  void ConnectionForce();
  int init(int,char**);

  //Constructor
  Connection();
};

#endif
