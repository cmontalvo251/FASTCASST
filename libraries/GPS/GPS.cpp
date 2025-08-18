#include "GPS.h"


///CONSTRUCTOR
GPS::GPS() {
}
///END OF CONSTRUCTOR

void GPS::init() {
  printstdout("Initializing GPS...\n");

  if (headingFilterConstant < 0) {
    headingFilterConstant = 0;
  }
  if (headingFilterConstant > 1.0) {
    headingFilterConstant = 1.0;
  }

  #ifndef DESKTOP
  #ifdef ARDUINO
  AdaGPS = new Adafruit_GPS(&Serial1);
  AdaGPS->begin(9600);
  Serial1.begin(9600); //Do I need this?? This should already happen in AdaGPS->begin()
  AdaGPS->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  AdaGPS->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  #else
  if(sensor.testConnection()){
    printf("Ublox test OK\n");
    if(!sensor.configureSolutionRate(1000)){
      printf("Setting new rate: FAILED\n");
    }
  } else {
    printf("GPS Test failed \n");
  }
  dist_vec.zeros(NGPS,1,"dist_vec");
  time_vec.zeros(NGPS,1,"time_vec");
  #endif
  #else
  printf("Using Fictitious GPS Block For Simulation \n");
  #endif
}

void GPS::printLLH() {
  printstdoutdbl(latitude,8);
  printstdoutdbl(longitude,8);
  printstdoutdbl(altitude,8);
}

void GPS::decodeXYZ() {
  //Assume that X,Y,Z coordinates are already set by some external function
  XYZ[0] = X;
  XYZ[1] = Y;
  XYZ[2] = Z;
  //printf("GPS ORIGIN = %lf %lf \n",X_origin,Y_origin);
  #if defined (satellite) || (cubesat) 
  //printf("Using Spherical conversion \n");
  ConvertXYZ2LLHSPHERICAL(XYZ,LLH);
  #else
  //Note we need to use the simulation coordinates so we know where we are on the planet
  //Probably need an overhaul where we set the GPS origin coordinates in Simulation.txt
  //But I don't want to do that right now.
  ConvertXYZ2LLH(XYZ,LLH,X_origin_SIMULATION,Y_origin_SIMULATION);
  #endif
  latitude = LLH[0];
  longitude = LLH[1];
  altitude = LLH[2];
  //printf("GPS XYZ = %lf %lf %lf \n",X,Y,Z);
  //printf("GPS LLH = %lf %lf %lf \n",latitude,longitude,altitude);
  //PAUSE();  
}

void GPS::poll(float currentTime) {
  lastTime = currentTime;
  #ifndef DESKTOP
  #ifdef ARDUINO
  //Put code here to poll GPS
  char c = 1;
  while (c != 0) {
    c = AdaGPS->read();
  }
  if (AdaGPS->newNMEAreceived()) {
    AdaGPS->parse(AdaGPS->lastNMEA());
    //printstdout("GPS NMEA Sentence Received \n");
  }
  #else
  sensor.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data);
  #endif
  #else
  //USE SOFTWARE TO CREATE GPS COORDINATES
  decodeXYZ();
  //Then populate pos_data so that the routine below still works
  pos_data.resize(5,1);
  pos_data[0] = 0.0; //not really sure what this is
  pos_data[1] = longitude*10000000.0;
  pos_data[2] = latitude*10000000.0;
  pos_data[3] = altitude*1000.0;
  pos_data[4] = 0.0; //not sure what this is either
  #endif

  //This runs no matter what
  processGPSCoordinates(currentTime);
}

void GPS::processGPSCoordinates(double currentTime) {
  VALIDGPS = status();
  if (VALIDGPS) {
    #ifdef ARDUINO
    //Get GPS LLH on Arduino
    latitude = AdaGPS->latitudeDegrees;
    longitude = AdaGPS->longitudeDegrees;
    altitude = AdaGPS->altitude;
    #else
    latitude = pos_data[2]/10000000.0; //lon - Maxwell says it may be lon lat
    longitude = pos_data[1]/10000000.0; //lat - It really is lon lat
    altitude = pos_data[3]/1000.0; ///height above ellipsoid 1984?
    #endif
    if (GPSORIGINSET) {
      //Get Speed
      #ifdef ARDUINO
      speed = AdaGPS->speed;
      #else
      computeCOG(currentTime);
      #endif
    } else {
      GPSORIGINSET = 1;
      printstdout("GPS Coordinate initialized. Resetting GPS Vals \n");
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
  if (!GPSORIGINSET){
    GPSORIGINSET = 1;
    printstdout("GPS Coordinate initialized. Resetting GPS Vals \n");
    setOrigin(X_origin_SIMULATION,Y_origin_SIMULATION);
    xprev = X;
    yprev = Y;
  }
}

void GPS::setOrigin(double latitude,double longitude) {
  X_origin = latitude; //These were originally commented out. 
  Y_origin = longitude; //Why? I have a new status() function. Hopefully that fixed it
  printstdout("Origin Set = ");
  printstdoutdbl(X_origin);
  printstdout(" ");
  printstdoutdbl(Y_origin);
  printstdout("\n");
}

void GPS::setXYZ(double Xin,double Yin,double Zin) {
  X = Xin;
  Y = Yin;
  Z = Zin;
  //printf("XYZ Set to = %lf %lf %lf \n",X,Y,Z);
}

int GPS::status() {
  #ifdef PRINTSEVERYWHERE
  printstdout("Checking GPS Health \n");
  #endif
  //if (timeSinceStart > 10) {
  int ok = 0;
  #ifdef ARDUINO
  ////ARDUINO
  ok = AdaGPS->fixquality;
  //Serial.print("Fix Quality = ");
  //Serial.print(ok);
  #else

  #if defined (SIMONLY) || (SIL) 
  return 1; //Assume GPS is always working in SIMONLY / SIL Modes
  #endif

  ///RASPBERRY PI
  sensor.decodeSingleMessage(Ublox::NAV_STATUS, nav_data);
  int size = nav_data.size();
  #ifdef PRINTSEVERYWHERE
  printstdout("Size of Nav_Data = %d \n",size);
  #endif
  if (size > 0) {
    ok = (int(nav_data[0]) == 0x00);
  }
  //Also need to check for pos_data size
  int sizepos_data = pos_data.size();
  if (sizepos_data < 5) { 
    ok = 0;
  }
  #endif

  //printstdout("Returning Ok \n");
  return ok;
}

void GPS::ConvertGPS2XY()  {
  if ((latitude == -99) || (longitude == -99) || (altitude == -99)) {
    X = xprev;
    Y = yprev;
    Z = zprev;
  } else {
    //Populate LLH matrices
    LLH[0] = latitude;
    LLH[1] = longitude;
    LLH[2] = altitude;
    //These functions are in mathp.cpp
    #if defined (satellite) || (cubesat) 
    ConvertLLH2XYZSPHERICAL(XYZ,LLH);
    #else
    ConvertLLH2XYZ(XYZ,LLH,X_origin,Y_origin);
    #endif
    //Extract XYZ
    X = XYZ[0];
    Y = XYZ[1];
    Z = XYZ[2];
  }
}

void GPS::computeCOG(double current_time) {
  //First convert the current measurement to XY
  ConvertGPS2XY(); //This has been moved to mathp.cpp

  //Then proceed with the speed measurement
  double dx = X - xprev;
  double dy = Y - yprev;
  double dz = Z - zprev; 
  double dt = current_time - prev_time;
  dist = sqrt((pow(dx,2)) + (pow(dy,2)));

  if (abs(dx) + abs(dy) < 1e-20) {
    return;
  }
  ///Also get heading
  double heading_new = atan2(dy,dx)*180.0/M_PI;

  //Filter heading
  heading = (1-headingFilterConstant)*heading_new + (headingFilterConstant)*heading;

  //printf("TIME = %lf , LATITUDE = %lf , LONGITUDE = %lf , X = %lf , Y = %lf , dx = %lf , dy = %lf , heading = %lf \n",current_time,latitude,longitude,X,Y,dx,dy,heading);

  ///////////??COMPUTE RAW SPEED AND HEAVILY FILTERED SPEED
  if (dt > 0) {
    //Compute x,y,z dots
    vx = dx/dt; 
    vy = dy/dt; 
    vz = dz/dt; 

    //Take speed parallel to the ground to be the total velocity of vx and vy
    speed = sqrt((vx*vx + vy*vy));
    //speed = vx * cos(heading) + vy * sin(heading); //not implemented because we can just use norm(vx,vy)

    //The sideslip velocity is a standard coordinate transformation based on heading
    sideslip_speed = -vx * sin(heading*M_PI/180.0) + vy * cos(heading*M_PI/180.0);

    //The vertical speed is just vz
    vertical_speed = vz;
    
  } else {
    speed = 0;
    sideslip_speed = 0;
    vertical_speed = 0;
  } 
  //printf("speed = %lf \n",speed);

  //UPDATE PREVIOUS VALUES
  xprev = X;
  yprev = Y;
  zprev = Z;
  prev_time = current_time;
}


////////////////////DO NOT USE THIS FUNCTION BELOW

void GPS::computeGroundTrack(double current_time) {
  //First convert the current measurement to XY
  ConvertGPS2XY();

  //Then proceed with the speed measurement
  double dx = X - xprev;
  double dy = Y - yprev;
  double dt = current_time - prev_time;
  dist = sqrt((pow(dx,2)) + (pow(dy,2)));

  if (abs(dx) + abs(dy) < 0.1) {
    return;
  }
  ///Also get heading
  //printf("X = %lf, Y = %lf \n",X,Y);
  //printf("dx = %lf dy = %lf \n",dx,dy);
  double heading_new = atan2(dy,dx)*180.0/M_PI;
  //printf("heading = %lf \n",heading_new);

  //Filter heading
  heading = (1-headingFilterConstant)*heading_new + (headingFilterConstant)*heading;

  //printf("TIME = %lf , LATITUDE = %lf , LONGITUDE = %lf , X = %lf , Y = %lf , dx = %lf , dy = %lf , heading_new = %lf , heading = %lf \n",current_time,latitude,longitude,X,Y,dx,dy,heading_new,heading);

  ///////////??COMPUTE RAW SPEED AND HEAVILY FILTERED SPEED
  if (dt > 0) {
    double vx = dx/dt;
    double vy = dy/dt;
    speed_raw = sqrt((vx*vx + vy*vy));
  } else {
    speed_raw = 0;
  } 
  //printf("speed_raw = %lf \n",speed_raw);

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

  //////////////HARDCODED BYPASS FILTER///////////////
  speed = speed_raw;
  heading = heading_new;

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

