#include "GPS.h"
#include <math.h>

GPS::GPS() {
  if (headingFilterConstant < 0) {
    headingFilterConstant = 0;
  }
  if (headingFilterConstant > 1.0) {
    headingFilterConstant = 1.0;
  }

  #ifndef DESKTOP
  if(sensor.testConnection()){
    printf("Ublox test OK\n");
    if(!sensor.configureSolutionRate(1000)){
      printf("Setting new rate: FAILED\n");
    }
  } else {
    printf("GPS Test failed \n");
  }
  #else
  printf("Using Fictitious GPS Block For Simulation \n");
  #endif
  dist_vec.zeros(NGPS,1,"dist_vec");
  time_vec.zeros(NGPS,1,"time_vec");
}

void GPS::poll(float currentTime) {
  lastTime = currentTime;
  #ifndef DESKTOP
  sensor.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data);
  #else
  //USE SOFTWARE TO CREATE GPS COORDINATES
  //Assume that X,Y,Z coordinates are already set by some external function
  XYZ[0] = X;
  XYZ[1] = Y;
  XYZ[2] = Z;
  ConvertXYZ2LLH(XYZ,LLH,X_origin,Y_origin);
  latitude = LLH[0];
  longitude = LLH[1];
  altitude = LLH[2];
  //Then populate pos_data so that the routine below still works
  pos_data.resize(5,1);
  pos_data[0] = 0.0; //not really sure what this is
  pos_data[1] = longitude*10000000.0;
  pos_data[2] = latitude*10000000.0;
  pos_data[3] = altitude*1000.0;
  pos_data[4] = 0.0; //not sure what this is either
  #endif
  if (pos_data.size() > 4) {
    latitude = pos_data[2]/10000000.0; //lon - Maxwell says it may be lon lat
    longitude = pos_data[1]/10000000.0; //lat - It really is lon lat
    altitude = pos_data[3]/1000.0; ///height above ellipsoid 1984?
    //If the measurement is good
    if (VALIDGPS) {
      computeGroundTrack(currentTime);
    } else {
      printf("GPS Coordinate initialized. Resetting GPS Vals \n");
      VALIDGPS = 1;
      //Set the origin
      setOrigin(latitude,longitude);
      //Convert to XYZ
      ConvertGPS2XY();
      //Reset Prev Values
      xprev = X;
      yprev = Y;
    }
  }
}

void GPS::reset() {
  if (!VALIDGPS){
    VALIDGPS = 1;
    xprev = X;
    yprev = Y;
  }
}

void GPS::setOrigin(double latitude,double longitude) {
  X_origin = latitude;
  Y_origin = longitude;
  printf("Origin Set = %lf %lf \n",X_origin,Y_origin);
}

void GPS::setXYZ(double Xin,double Yin,double Zin) {
  X = Xin;
  Y = Yin;
  Z = Zin;
  //printf("XYZ Set to = %lf %lf %lf \n",X,Y,Z);
}

int GPS::status() {
  #ifdef PRINTSEVERYWHERE
  printf("Checking GPS Health \n");
  #endif
  //if (timeSinceStart > 10) {
  sensor.decodeSingleMessage(Ublox::NAV_STATUS, nav_data);
  int size = nav_data.size();
  #ifdef PRINTSEVERYWHERE
  printf("Size of Nav_Data = %d \n",size);
  #endif
  ok = 1;
  if (size > 0) {
    ok = (int(nav_data[0]) == 0x00);
  }
  return ok;
}

void GPS::ConvertGPS2XY(){
  ///CONVERT LAT LON TO METERS
  if (longitude == -99) {
    Y = yprev;
  } else {
    Y = (longitude - Y_origin)*GPSVAL*cos(X_origin*DEG2RAD);  
  }
  if (latitude == -99) {
    X = xprev;
  } else {
    X = (latitude - X_origin)*GPSVAL;
  }
  if (altitude == -99) {
    Z = zprev;
  } else {
    Z = -altitude;
  }
  //printf("ConvertLLH2XY: %lf %lf %lf %lf \n",X,Y,latitude,longitude);
}

void GPS::computeGroundTrack(double current_time) {
  //First convert the current measurement to XY
  ConvertGPS2XY();
  //Then proceed with the speed measurement
  double dx = X - xprev;
  double dy = Y - yprev;
  double dt = current_time - prev_time;
  dist = sqrt((pow(dx,2)) + (pow(dy,2)));
  ///Also get heading
  //printf("X = %lf, Y = %lf \n",X,Y);
  //printf("dx = %lf dy = %lf \n",dx,dy);
  double heading_new = atan2(dy,dx)*180.0/M_PI;
  printf("heading = %lf \n",heading_new);

  //Filter heading
  heading = (1-headingFilterConstant)*heading_new + heading*headingFilterConstant;

  ///////////??COMPUTE RAW SPEED AND HEAVILY FILTERED SPEED
  if (dt > 0) {
    double vx = dx/dt;
    double vy = dy/dt;
    speed_raw = sqrt((vx*vx + vy*vy));
  } else {
    speed_raw = 0;
  } 
  printf("speed_raw = %lf \n",speed_raw);

  ////////////FILTERED SPEED
  int em1;
  //Get previous value
  em1 = end_pt - 1;

  if (em1 == 0) {
    em1 = NGPS;
  }
  
  double previous_distance = dist_vec.get(em1,1);
  double new_distance = dist + previous_distance;
  dist_vec.set(end_pt,1,new_distance);
  time_vec.set(end_pt,1,current_time);

  //dist_vec.disp();
  //time_vec.disp();
  double del_dist = dist_vec.get(end_pt,1) - dist_vec.get(start_pt,1);
  double del_time = time_vec.get(end_pt,1) - time_vec.get(start_pt,1);

  //printf("%lf %lf %lf %lf %lf %lf \n",dist_vec.get(end_pt,1),dist_vec.get(start_pt,1),time_vec.get(end_pt,1),time_vec.get(start_pt,1),del_dist,del_time);
  if (del_time == 0) {
    speed = 0;
  } else {
    speed = del_dist/del_time;
  }

  //////////////HARDCODED BYPASS TO GPS SPEED///////////////
  speed = speed_raw;

  //printf("Speed = %lf \n",speed);
  
  end_pt += 1;
  start_pt += 1;
  
  if (end_pt > NGPS) {
    end_pt = 1;
  }
  if (start_pt > NGPS){
    start_pt = 1;
  }

  //UPDATE PREVIOUS VALUES
  xprev = X;
  yprev = Y;
  zprev = Z;
  prev_time = current_time;
}


////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////
/////////DO NOT EDIT THIS ROUTINE IT IS JUST HERE FOR BACKWARDS COMPATIBILITY
void GPS::computeSpeed(double current_time) {
  //First convert the current measurement to XY
  ConvertGPS2XY();
  //Then proceed with the speed measurement
  double dx = X - xprev;
  double dy = Y - yprev;
  dist = sqrt((pow(dx,2)) + (pow(dy,2)));
  int em1;

  //Get previous value
  em1 = end_pt - 1;

  if (em1 == 0) {
    em1 = NGPS;
  }
  
  double previous_distance = dist_vec.get(em1,1);
  double new_distance = dist + previous_distance;
  dist_vec.set(end_pt,1,new_distance);

  time_vec.set(end_pt,1,current_time/1000000.0);

  double del_dist = dist_vec.get(end_pt,1) - dist_vec.get(start_pt,1);
  double del_time = time_vec.get(end_pt,1) - time_vec.get(start_pt,1);

  //printf("%lf %lf %lf %lf %lf %lf \n",dist_vec.get(end_pt,1),dist_vec.get(start_pt,1),time_vec.get(end_pt,1),time_vec.get(start_pt,1),del_dist,del_time);
  
  speed = del_dist/del_time;
  
  end_pt += 1;
  start_pt += 1;
  
  if (end_pt > NGPS) {
    end_pt = 1;
  }
  if (start_pt > NGPS){
    start_pt = 1;
  }

  //UPDATE PREVIOUS VALUES
  xprev = X;
  yprev = Y;
  zprev = Z;
}

