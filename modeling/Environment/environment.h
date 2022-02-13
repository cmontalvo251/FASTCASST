#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <MATLAB/MATLAB.h>
#include <Rotation/Rotation3.h>
#include <Mathp/mathp.h>
#include <Timer/timer.h>

//The GeographicLib library can be downloaded from the internet by following the README.md locating at the ~/ of this folder
//However for the convenience of the linux user the libraries have been placed in the source folder here so ther
//is really no need to download it. In order to make this work a -I and -L have been added to the Makefile
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#include "GeographicLib\GravityModel.hpp" //Necessary for gravity model
#include "GeographicLib\MagneticModel.hpp" //Necessary for magnetic model
#include "define.h" //this is used mainly for Visual Studio, as the makefile doesn't affect VS projects
#else // linux or unix
#include <GeographicLib/GravityModel.hpp> //Necessary for gravity model
#include <GeographicLib/MagneticModel.hpp> //Necessary for magnetic model
#endif

using namespace GeographicLib; //Necessary for Grav and Magnet models

class environment {
 private:
  char COEFFFILENAME[256];
  int GRAVITY_FLAG;
  double mass;
  GravityModel* egm2008;
  MagneticModel* emm2015;
  MATLAB sph_coord;
  MATLAB SOLAR_DIRECTION;
  Rotation3 sph2ine32;
  int Gravity_Flag = 0,Magnetic_Flag=0;
  double yr = 2000,mo=1,day=1;
  int SOLARWINDMODEL;
  int REFERENCEFRAME = 0;
  double time_magnet = 0,time_magnet_next;
  MATLAB rSun2Earth2000,rSun2EarthToday,rEarth20002EarthToday,rSun2Sat,gSun;
  double julian_today=2451545;
  double X_SOLAR_MAX,X_SOLAR_MIN,Y_SOLAR_MIN,Y_SOLAR_MAX,GRIDX,GRIDY;
  MATLAB XCOORD,YCOORD,TCOORD,SOLARX,SOLARY,SOLARZ,RCOORD,XYZE;
  void sph2ine(double,double,double);
  double getSOLARWINDMODEL(MATLAB,MATLAB,double);
  double getSOLARWINDMODEL(MATLAB,MATLAB,double,MATLAB); //overloaded function for confluence point
 public:
   MATLAB FGRAVI,FGNDI,MGNDI;
   MATLAB BVECINE,BVECSPH,BVECB_Tesla;
   void init(MATLAB);
   void gravitymodel(MATLAB State);
   void groundcontactmodel(MATLAB,MATLAB);
   void setMass(double);
   void getCurrentMagnetic(double,MATLAB);
   double getCurrentDensity(); //For now put this in here. Probably need an aero routine later
   void EarthEphemeris(MATLAB);
   void EarthEphemeris(MATLAB,double);
   environment(); //constructor   
};


#endif
