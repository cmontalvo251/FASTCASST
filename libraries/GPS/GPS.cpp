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
  //Reset to override origin
  reset();
  #endif
}

void GPS::printLLH() {
  printstdoutdbl(latitude,8);
  printstdoutdbl(longitude,8);
  printstdoutdbl(altitude,8);
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
  //USE SOFTWARE TO CREATE GPS COORDINATES from sensors send routine
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
      speed = AdaGPS->speed; //What about heading?
      #else
      compute_heading_velocity(currentTime);
      #endif
    } else {
      GPSORIGINSET = 1;
      printstdout("GPS Coordinate initialized. Resetting GPS Vals \n");
      //Set the origin
      setOrigin(latitude,longitude);
    }
  }
}

void GPS::setOriginOverride(double latitude,double longitude) {
  X_origin_OVERRIDE = latitude;
  Y_origin_OVERRIDE = longitude;
  printstdout("Override GPS Origin Set to = ");
  printstdoutdbl(X_origin_OVERRIDE);
  printstdout(" ");
  printstdoutdbl(Y_origin_OVERRIDE);
  printstdout("\n");
  setOrigin(X_origin_OVERRIDE,Y_origin_OVERRIDE);
  GPSORIGINSET = 1;
  //PAUSE();
}

void GPS::reset() {
  if (!GPSORIGINSET){
    GPSORIGINSET = 1;
    printstdout("GPS Coordinate initialized. Resetting GPS Vals \n");
    setOrigin(X_origin_OVERRIDE,Y_origin_OVERRIDE);
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

void GPS::setLLH(double Xin,double Yin,double Zin) {
  double XYZ[3],LLH[3];
  XYZ[0] = Xin;
  XYZ[1] = Yin;
  XYZ[2] = Zin;
  #if defined (satellite) || (cubesat) 
  ConvertXYZ2LLHSPHERICAL(XYZ,LLH);
  #else
  ConvertXYZ2LLH(XYZ,LLH,X_origin,Y_origin);
  #endif
  latitude = LLH[0];
  longitude = LLH[1];
  altitude = LLH[2];
}

// Converts (lat, lon) degrees to a 3D unit vector on a sphere
Vector3D GPS::LatLonToUnitVector(double lat_deg, double lon_deg) {
    double lat_rad = lat_deg * DEG2RAD;
    double lon_rad = lon_deg * DEG2RAD;

    return {
        std::cos(lat_rad) * std::cos(lon_rad),
        std::cos(lat_rad) * std::sin(lon_rad),
        std::sin(lat_rad)
    };
}

void GPS::compute_heading_velocity(double current_time) {
  //#Get delta lat and delta lon
  double dt = abs(current_time - prev_time);

  //These functions are in mathp.cpp
  //First convert latitude,longitude and altitude to XYZ
  double XYZ[3],LLH[3];
  LLH[0] = latitude;
  LLH[1] = longitude;
  LLH[2] = altitude;
  #if defined (satellite) || (cubesat) 
  ConvertLLH2XYZSPHERICAL(XYZ,LLH);
  #else
  ConvertLLH2XYZ(XYZ,LLH,X_origin,Y_origin);
  #endif
  X = XYZ[0];
  Y = XYZ[1];
  Z = XYZ[2];

  if (latitude_prev != -99) {

    #if defined (cubesat) || (satellite)
    double new_heading = atan2(Y-Y_prev,X-X_prev)*RAD2DEG;
    #else
    //Flat Earth Model
    //HEADING ANGLE COMPUTATION FROM GEMINI
    double phi2 = latitude*PI/180.0;
    double phi1 = latitude_prev*PI/180.0;
    double delta_lambda = (longitude - longitude_prev)*PI/180;
    double yheading = sin(delta_lambda) * cos(phi2);
    double xheading = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lambda);
    //Calculate initial bearing in radians and convert to degrees
    double new_heading = atan2(yheading,xheading)*180/PI;
    #endif

    //Compute Heading
    if (headingFLAG == 0) {
      heading = new_heading;
      headingFLAG = 1;
    } else {
      //Compute heading with filtering to smooth it out.
      heading = new_heading*(1-headingFilterConstant) + heading*(headingFilterConstant);
    }
    if (heading < 0) {
      heading += 360;
    }
    if (heading > 360) {
      heading -= 360;
    }
    //printf("Heading = %lf \n",heading);
    //Compute Speed — guard against dt==0 to prevent NaN (fix for issue #59)
    if (dt > 0) {
      Vector3D p1 = LatLonToUnitVector(latitude_prev, longitude_prev);
      Vector3D p2 = LatLonToUnitVector(latitude, longitude);
      // Dot product: p1 . p2 = cos(theta)
      double dot = p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
      // Clamp dot product to [-1.0, 1.0] to prevent floating-point precision errors in acos
      dot = CONSTRAIN(dot, -1.0, 1.0);
      // Arc distance on unit sphere in radians
      double angular_distance = std::acos(dot);
      // Physical distance over the Earth's surface
      double distance_meters = REARTH * angular_distance;
      //speed = distance_meters / dt;
      double dlat = latitude - latitude_prev;
      double dlon = longitude - longitude_prev;
      #if defined (cubesat) || (satellite)
      double new_speed_x = (X-X_prev)/dt;
      double new_speed_y = (Y-Y_prev)/dt;
      double new_speed_z = (Z-Z_prev)/dt;
      #else
      double new_speed_x = (dlat*111000.)/dt;
      double new_speed_y = (dlon*111000.*cos(latitude*PI/180.))/dt;
      double new_speed_z = 0;
      #endif
      if (speedFLAG == 0) {
        speedFLAG = 1;
        speed = distance_meters/dt;
        speed_x = new_speed_x;
        speed_y = new_speed_y;
        speed_z = new_speed_z;
      } else {
        speed = (distance_meters/dt)*(1-headingFilterConstant) + speed*(headingFilterConstant);
        speed_x = new_speed_x*(1-headingFilterConstant) + speed_x*(headingFilterConstant);
        speed_y = new_speed_y*(1-headingFilterConstant) + speed_y*(headingFilterConstant);
        speed_z = new_speed_z*(1-headingFilterConstant) + speed_z*(headingFilterConstant);
      }
    }
    //Get Vx and Vy - ARCHAIC CODE. CONSIDER REMOVING
    //vx = speed * cos(heading*M_PI/180.0);
    //vy = speed * sin(heading*M_PI/180.0);
  }
  //Reset the lat/lon values
  latitude_prev = latitude;
  longitude_prev = longitude;    
  //Compute altitude and vertical speed — guard against dt==0 to prevent NaN (fix for issue #59)
  if (altitude_prev != -99 && dt > 0) {
    double dalt = -altitude + altitude_prev; //negative because z is down
    if (vertical_speedFLAG == 0) {
      vertical_speedFLAG = 1;
      vertical_speed = dalt/dt;
    } else {
      vertical_speed = dalt/dt*(1-headingFilterConstant) + vertical_speed*(headingFilterConstant);
    }
  }
  //Reset altitude
  altitude_prev = altitude;

  //RESET XYZ Values
  X_prev = X;
  Y_prev = Y;
  Z_prev = Z;

  //Reset time
  prev_time = current_time;
}

//////////////////////////////////////////////////////////////////
////////////////////DO NOT USE THE FUNCTIONS BELOW////////////////
//////////////////////////////////////////////////////////////////

/*void GPS::computeCOG(double current_time) {
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

  //Guard against del_time==0 to prevent NaN (fix for issue #59)
  if (del_time == 0) {
    speed = 0;
  } else {
    speed = del_dist/del_time;
  }
  
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
*/