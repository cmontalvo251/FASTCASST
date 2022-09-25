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
    int VALIDGPS = 0;
    double XYZ[3],LLH[3];
 public:
  std::vector<double> pos_data,nav_data;
  //X AND Y are Hardcoded to be zero initially and the origin point
  //is roughly set to Mobile
  double latitude=MOBX,longitude=MOBY,altitude=0.0,X=0,Y=0,Z=0,X_origin=IRVX,Y_origin=IRVY;
  double xprev=0,yprev=0,zprev=0,prev_time=0;
  double headingFilterConstant = 0.0;
  double heading;
  Ublox sensor;
  int ok;
  int end_pt = NGPS;
  int start_pt = 1;
  MATLAB dist_vec;
  MATLAB time_vec;
  double speed=0,dist=0,speed_raw=0;
  unsigned long lastTime = 0;
  GPS(); //constructor
  void poll(float);
  void reset();
  int status();
  void computeSpeed(double);
  void computeGroundTrack(double);
  void computeCOG(double);
  void ConvertGPS2XY();
  //void ConvertXYZ2LLH(); -- This has been moved to mathp.h as a helper function
  void setXYZ(double,double,double);
  void setOrigin(double,double);
};

#endif
