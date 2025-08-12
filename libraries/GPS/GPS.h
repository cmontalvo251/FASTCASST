#ifndef GPS_H
#define GPS_H

#ifndef ARDUINO
#include <Ublox/Ublox.h>
#include <Mathp/mathp.h>
#include <MATLAB/MATLAB.h>
#include <Timer/timer.h>
#include <Datalogger/Datalogger.h>
#else
#include <Adafruit_GPS.h>
#include "mathp.h"
#include "MATLAB.h"
#include "timer.h"
#include "Datalogger.h"
#endif
#include <math.h>

#define GPSPERIOD 0.5
#define NGPS 25

class GPS {
 private:
   #ifdef ARDUINO
   Adafruit_GPS *AdaGPS;
   #endif
   double GPSnextTime = 0; //Assume you always start at zero
   double GPSupdateRate = 4.0; //every 4 seconds
   int VALIDGPS = 0;
   int GPSORIGINSET = 0;
   double XYZ[3],LLH[3];
   double X_origin=-99,Y_origin=-99; 
   double X_origin_SIMULATION=GPSORIGINX,Y_origin_SIMULATION=GPSORIGINY; //These are set in mathp.h
 public:
  void printLLH();
  void init();
  #ifndef ARDUINO
  std::vector<double> pos_data,nav_data;
  Ublox sensor;
  #endif
  //X AND Y are Hardcoded to be zero initially and the origin point
  //is roughly set to Mobile but this isn't a good idea
  //So the latitude and longitude are set to -99
  //This is because we don't know where we are until we have
  //a valid GPS coordinate
  //In all modes except AUTO the origin is set automatically
  //Origin is set in mathp.h
  double latitude=-99,longitude=-99,altitude=0.0,X=0,Y=0,Z=0;
  double xprev=0,yprev=0,zprev=0,prev_time=0;
  double headingFilterConstant = 0.0;
  double heading;
  int ok;
  int end_pt = NGPS;
  int start_pt = 1;
  MATLAB dist_vec;
  MATLAB time_vec;
  double speed=0,sideslip_speed,vertical_speed,dist=0,speed_raw=0;
  double vx = 0, vy = 0, vz = 0; //xyz dots
  unsigned long lastTime = 0;
  GPS(); //constructor
  void poll(float);
  void reset();
  void processGPSCoordinates(double);
  void decodeXYZ();
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
