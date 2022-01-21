#ifndef GPS_H
#define GPS_H

#include <GPS/Ublox.h>
#include <Mathp/mathp.h>
#include <MATLAB/MATLAB.h>

#define GPSPERIOD 0.5
#define NGPS 25

class GPS {
 private:
    double GPSnextTime = 0; //Assume you always start at zero
    double GPSupdateRate = 4.0; //every 4 seconds
 public:
  std::vector<double> pos_data,nav_data;
  //X AND Y are Hardcoded to be zero initially and the origin point
  //is roughly set to Mobile
  double latitude,longitude,altitude,X=0,Y=0,Z=0,xprev=0,yprev=0,zprev=0,X_origin=30.69,Y_origin=-88.17;
  double headingFilterConstant = 0.5;
  double heading;
  Ublox sensor;
  int ok;
  int end_pt = NGPS;
  int start_pt = 1;
  MATLAB dist_vec;
  MATLAB time_vec;
  double speed,dist;
  unsigned long lastTime = 0;
  GPS(); //constructor
  void poll(float,int);
  int status();
  void computeSpeed(double);
  void computeGroundTrack(double);
  void ConvertGPS2XY();
  void ConvertXYZ2LLH();
  void setXYZ(double,double,double);
  void setOrigin(double,double);
};

#endif
