#include "environment.h"

//When you invoke this command the computer is going to search for
//these models. If it can't find them it will throw an error like this
//Cannot open /usr/local/share/GeographicLib/gravity/egm2008.egm
//Note it's possible to send the code a different path so we will fix this by putting another
//#ifndef. If you're on windows you need to download the model otherwise
//we will simply tell the computer to look in our user specified directory
//Notice that in satellite_physics.h we created a pointer to grav and mag
//In order to initialize them we need to use the "new" function much like
//we use the new function in main.cpp when we create a new Satellite
// Gravity_Flag = Gravity_IN; -- These have been moved to gravity_magnetic.cpp
// Magnetic_Flag = Magnetic_IN;
//Note. Since these are variables, it would be much nicer to put these in the
//Satellite class rather than these massive global variables. We can let it slide
//for now since it works but just a thought.

//constructor
environment::environment() {
}

void environment::setMass(double m) {
  //printf("m = %lf \n",m);
  mass = m;
  //PAUSE();
}

void environment::gravitymodel(MATLAB State) {
  FGRAVI.mult_eq(0); //zero out gravity
  gSun.mult_eq(0); //zero out sun gravity as well

  double gx=0, gy=0, gz=0;
  double x = State.get(1, 1);
  double y = State.get(2, 1);
  double z = State.get(3, 1);
  double mu = -GSPACE*MEARTH;
  double muSun = -GSPACE*MSUN;
  double rSat = sqrt(x*x + y*y + z*z);

  //Use the egm2008 model
  if (Gravity_Flag == 1)
    {
      if (rSat > REARTH) { //Make sure you're outside the earth otherwise this routine will return a nan
        egm2008->W(x, y, z, gx, gy, gz);
      } else {
      	gx = 0;
        gy = 0;
        gz = 0;
      }
    }
  //Use the point mass model
  else if ((Gravity_Flag == 0) || (Gravity_Flag == 2)) //This is the point mass model here
    {
      if (rSat > REARTH) { //Make sure you're outside the earth
        	gx = (mu / pow(rSat, 3))*x;
        	gy = (mu / pow(rSat, 3))*y;
        	gz = (mu / pow(rSat, 3))*z;
      } else {
      	//I don't really like this. I'm just going to kill the program if you land inside the Earth. I mean, at that point the simulation is invalid.
      	//Why continue?
      	//This environment.cpp is now part of FAST.git which means
      	//there is a ground contact model now
      	//printf("Gravity model is on and running but a part of your spacecraft is inside the Earth.....sooooo....\n");
      	//printf("the simulation is going to get killed. Hope you are having an ok day. \n");
      	//printf("X,Y,Z = %lf %lf %lf \n",x,y,z);
      	//printf("rSat = %lf REARTH = %lf delx = %lf \n",rSat,REARTH,rSat-REARTH);
      	//exit(1);
      	gx = 0;
      	gy = 0;
      	gz = 0;
      }
      if (Gravity_Flag == 2) {
      	//Need to add Sun gravity
      	//First get the location of the satellite
      	for (int i = 1;i<=3;i++) {
      	  //plus(rSun2EarthToday,XYZ_current);
      	  rSun2Sat.set(i,1,rSun2EarthToday.get(i,1)+State.get(i,1));
      	}
      	//Then compute gSun
      	gSun.overwrite(rSun2EarthToday);
      	gSun.mult_eq(muSun/pow(rSun2Sat.norm(),3));
      }
    }
      else if (Gravity_Flag == -1) { //Gravity is off
        gx = 0;
        gy = 0;
        gz = 0;
      }
      else if (Gravity_Flag == 3) { //Constant Gravity
        gx = 0;
        gy = 0;
        gz = GEARTH;
  }

  //printf("GX,GY,GZ = %lf %lf %lf \n",gx,gy,gz);
  //PAUSE();

  //Add Sun gravity
  FGRAVI.plus_eq(gSun);
  //Add Earth Gravity
  FGRAVI.plus_eq1(1,1,gx);
  FGRAVI.plus_eq1(2,1,gy);
  FGRAVI.plus_eq1(3,1,gz);
  //Multiply by Mass
  FGRAVI.mult_eq(mass);
  //FGRAVI.disp();
}

void environment::groundcontactmodel(MATLAB State,MATLAB k) {
  double x = State.get(1,1);
  double y = State.get(2,1);
  double z = State.get(3,1);
  double norm = sqrt(x*x + y*y + z*z);
  double xdot = k.get(1,1);
  double ydot = k.get(2,1);
  double zdot = k.get(3,1);
  double u = State.get(8,1);
  double v = State.get(9,1);
  double r = State.get(13,1);
  double N = mass*GRAVITYSI;

  //Check to see if we're inside Earth
  bool insideEarth = 0;
  //!Gravity Flag (-1=off,0=point mass, 1=egm2008,2=point mass + sun,3=flat earth)
  //Flat Earth Model. Z is down so anything positive is under the surface
  //printf("GRAVITY_FLAG = %d z = %lf \n",GRAVITY_FLAG,z);
  if ((Gravity_Flag == 3) & (z>0)) {
    insideEarth = 1;
  }
  //Globe Model
  if ((Gravity_Flag == 1) & (norm<REARTH)) {
    insideEarth = 1;
  }
  if (insideEarth) {
    //printf("INSIDE EARTH! \n");
    FGNDI.set(1,1,-N*GNDCOEFF*sat(xdot,0.1,1.0));
    FGNDI.set(2,1,-N*GNDCOEFF*sat(ydot,0.1,1.0));
    FGNDI.set(3,1,-z*GNDSTIFF-zdot*GNDDAMP);
    //if (abs(r)>0.01) {
    //  MGNDI.set(3,1,-0.0001*N*GNDSTIFF*sat(r,0.01,1.0));
    //} else {
    MGNDI.mult_eq(0);
    //} 
  } else {
    FGNDI.mult_eq(0);
    MGNDI.mult_eq(0);
  }
  //FGNDI.disp();
}

void environment::init(MATLAB in_simulation_matrix) {
  FGRAVI.zeros(3,1,"FORCE OF GRAVITY INERTIAL");
  FGNDI.zeros(3,1,"Ground Forces Inertial Frame");
  MGNDI.zeros(3,1,"Ground Moments Inertial Frame");
  BVECINE.zeros(3,1,"Inertial Frame Vectors of Magnetic Field");
  BVECSPH.zeros(3,1,"Speherical Frame Vectors of Magnetic Field");
  BVECB_Tesla.zeros(3,1,"Environment Magnetic Field Body Frame (Tesla)");

  //Magnet and Gravity Model Stuff
  Gravity_Flag = in_simulation_matrix.get(17,1);
  Magnetic_Flag = in_simulation_matrix.get(18,1);
  time_magnet_next = in_simulation_matrix.get(19,1);
  time_magnet = 0;
  julian_today = in_simulation_matrix.get(20,1);
  double julian_2000 = 2451545;
  if (julian_today < 2451545) {
    //Must be a year
    yr = julian_today;
    julian_today = julian_2000 + (yr-2000)*365.25;
  } else {
    //Otherwise compute year based off julian day
    yr = int((julian_today - julian_2000)/365.25) + 2000;
  }
  printf("Julian Day = %lf \n",julian_today);
  printf("Year = %lf \n",yr);

  //Get Distances from Sun to Earth
  rSun2Earth2000.zeros(3,1,"XYZ of Earth from Sun on Jan 1,2000");
  rSun2EarthToday.zeros(3,1,"XYZ of Earth from Sun Today");
  
  //Compute the location of the earth on Jan 1 2000 using the Julian Day of that time
  EarthEphemeris(rSun2Earth2000,julian_2000);
  //rSun2Earth2000.disp();
  //Now compute the location of Earth on the Julian Day supplied
  EarthEphemeris(rSun2EarthToday,julian_today);
  //rSun2EarthToday.disp();
  
  //Now compute the location of the Earth today relative to the Earth on Jan 1st, 2000
  rEarth20002EarthToday.zeros(3,1,"XYZ of Earth Today from Earth 1/1/2000");
  rEarth20002EarthToday.minus(rSun2EarthToday,rSun2Earth2000);
  //rEarth20002EarthToday.disp();
  rSun2Sat.zeros(3,1,"Sun 2 Satellite");
  gSun.zeros(3,1,"Sun Gravity Acceleration");

  #ifdef USEHILPATH
  sprintf(COEFFFILENAME,"%s","../../GeographicLib/EGM_EMM");
  #else
  sprintf(COEFFFILENAME,"%s","modeling/GeographicLib/EGM_EMM");
  #endif

  if (Gravity_Flag == 1) {
    #ifdef __linux__
    egm2008 = new GravityModel("egm2008",COEFFFILENAME); 
    #else
    egm2008 = new GravityModel("egm2008"); //Initializing gravity model
    #endif
    printf("Gravity Model Imported \n");
  }
  if (Magnetic_Flag == 1) {
    #ifdef __linux__
    emm2015 = new MagneticModel("emm2015",COEFFFILENAME); //Initializing magnetic model
    printf("Magnetic Model Imported Using %s \n",COEFFFILENAME);
    #else
    emm2015 = new MagneticModel("emm2015",COEFFFILENAME); 
    printf("Magnetic Model Imported Using %s \n",COEFFFILENAME);
    #endif
  }
  sph_coord.zeros(3,1,"Spherical Coordinate (Phi and Theta)");
  printf("Gravity and Magnet Models Imported but you might need to double check the .emm and .egm file \n");
}

void environment::getCurrentMagnetic(double simtime,MATLAB State) {
  
  //This routine will only update the magnetic field once
  if (time_magnet_next == -99 && BVECINE.get(1,1) != 0) { 
    time_magnet = simtime + 1e10;
  }

  //If it's time to update the magnetic field go ahead and
  //increment the timer and then proceed
  if (simtime >= time_magnet)  {
    time_magnet += time_magnet_next;
  } else {
    //otherwise return prematurely
    return;
  }
  
  if (Magnetic_Flag == 1)
    {
      double b_east, b_north, b_vertical;
      double bx,by,bz;
      double x = State.get(1, 1);
      double y = State.get(2, 1);
      double z = State.get(3, 1);
      //x = rho*sin(phi)*cos(theta);
      //y = rho*sin(phi)*sin(theta);
      // y/x = tan(theta)
      //z = rho*cos(phi);
      double rho = sqrt((pow(x, 2) + pow(y, 2) + pow(z, 2)));
      double phi;
      if (rho > 0) {
        phi = (acos(z / rho));
      } else {
        phi = 0;
      }
      double the = atan2(y , x);
      double lat = 90 - phi*(180 / PI);
      double lon = the*(180/PI);
      double h = rho-REARTH; //need to send the model the height above the earth's surface
      //printf("x,y,z,rho,H = %lf,%lf,%lf,%lf,%lf \n",x,y,z,rho,h);
      if (h > 0) {
      	/**
      	 * Evaluate the components of the geomagnetic field.
      	 *
      	 * @param[in] t the time (years).
      	 * @param[in] lat latitude of the point (degrees).
      	 * @param[in] lon longitude of the point (degrees).
      	 * @param[in] h the height of the point above the ellipsoid (meters).
      	 * @param[out] Bx the easterly component of the magnetic field (nanotesla).
      	 * @param[out] By the northerly component of the magnetic field (nanotesla).
      	 * @param[out] Bz the vertical (up) component of the magnetic field (nanotesla).
      	 **********************************************************************/
      	emm2015->operator()(yr, lat, lon, h, b_east, b_north, b_vertical);

      	double b_down = -b_vertical; //vertical must be inverted to match NED

      	//Our "spherical reference frame has the following coordinate system
      	bx = b_north;
      	by = b_east;
      	bz = b_down;
      } else {
      	bx = 0;
      	by = 0;
      	bz = 0;
      }

      BVECSPH.set(1, 1, bx); //Btdubs these are all in nT
      BVECSPH.set(2, 1, by);
      BVECSPH.set(3, 1, bz);

      //Our inertial frame is set such that x goes out the equator at
      //the prime meridian, y is orthogonal to x and z goes through
      //the north pole

      //In order to go from spherical to inertial we need to
      //understand that aircraft convention uses the 3-2-1 Euler angle
      //convention

      //Rotation about the z-axis (psiE) - 3
      //rotation about the y-axis (thetaE) - 2
      //rotation about the x-axis (phiE) - 1
      //However, these are not the same as phi and the from the
      //spherical reference frame. Longitude is measure to the right
      //but this is actually a positive rotation about z. Thus
      double psiE = the;
      //Furthermore, phi is a positive rotation about the y-axis but you need to add pi to make sure
      //the z-component points downwards
      double thetaE = phi+PI;
      //Finally, there is no rotation about the x-axis
      double phiE = 0;

      //With these "Euler" Angles defined we can convert the spherical
      //coordinates to inertial coordinates.
      sph2ine(phiE, thetaE, psiE);
    }
  else
    {
      //If the magnetic field model is off just set them to zero
      BVECSPH.mult_eq(0.0);
      BVECINE.mult_eq(0.0);
    }
  return;
}

void environment::sph2ine(double phi, double the, double psi)
{
  sph_coord.set(1,1,phi);
  sph_coord.set(2,1,the);
  sph_coord.set(3,1,psi);
  //sph_coord.disp();
  sph2ine32.L321(sph_coord,0); //0 for Euler Angles
  //sph2ine32.disp();
  //vecSPH.disp();
  sph2ine32.rotateBody2Inertial(BVECINE,BVECSPH);
  //vecI.disp();
}

double environment::getCurrentDensity() {
  return RHOSLSI;
}

void environment::EarthEphemeris(MATLAB Si) {
  //Overloaded function call. Use the day month and year to call Earth Ephemeris
  EarthEphemeris(Si,julian_today);
}

void environment::EarthEphemeris(MATLAB Si,double julian_day) {
  //First step to Figure this out is I need to now where the Sun is. Well that's easy. It's 0,0,0. 
  //Really when people say the Sun's ephemeris they mean the Earth's Ephemeris data
  //I've learned this in my Orbital Mechanics Text book but I may as well just look this up again.
  //I don't want to make this fancy with input files so I'm just going to hard code everything.
  //So JPL has this system called the HORIZONS
  //https://ssd.jpl.nasa.gov/?horizons
  //In order to access the HORIZONS System simply type the following command

  //$ telnet horizons.jpl.nasa.gov 6775

  //You can use crude models here
  //https://ssd.jpl.nasa.gov/txt/p_elem_t1.txt -- I've saved this txt to the Gitlab repo as well
  //So First step is to follow the PDF titled - aprx_pos_planets.pdf
  //An example Implementation of this code is in Fundamentals_Astrodynamics using the routine julian_day_orbit.py
  //Really you need to look at the Universe.py module. Anywho here is how that code works.

  //First we need to get some orbital elements of the Earth which are defined from Jan 1st 2000
  double a0 = 1.00000261; //Semi major axis
  double e0 = 0.01671123; //eccentricity (0 is a perfect circle)
  double i0 = -0.00001531; //inclination in degrees
  double L0 = 100.46457166; //mean longitude in degrees
  double wbar0 = 102.93768193; //longitude of the perihelion
  double OMEGA0 = 0.0; //longitude of the ascending node
  //In order to get these parameters at the particular date in time we need to compute the rate 
  double adot = 0.00000562; //these are all /century
  double edot = -0.00004392;
  double idot = -0.01294668;
  double Ldot = 35999.37244981;
  double wbardot = 0.32327364;
  double OMEGAdot = 0.0;
  //#compute the orbital Elements for this particular Julian Day (2451545 is Jan 1st 2000)
  double T = (julian_day - 2451545.0)/36525.0;
  double AU = 149597870700.0; //this is 1 Astronaumical Unit in meters - the distance from Earth to the Sun
  double a = (a0 + T*adot)*AU;
  double e = e0 + T*edot; //##This is in radians
  double i = i0 + T*idot; //this is in degrees
  double L = L0 + T*Ldot; //deg
  double wbar = wbar0 + T*wbardot; //deg
  double OMEGA = OMEGA0 + T*OMEGAdot; //deg
  //If you're trying to find the orbital elements of any planet past Mars you need to add correction factors but 
  //Earth doesn't need any
  //#Compute estar
  double estar = e*180.0/PI;
  //#Argument of the perihelion
  double w = wbar - OMEGA;
  //#Mean Anomaly - No correction factors
  double M = L - wbar;
  //#Need to modulus M
  while (M > 180) { 
    M -= 360;
  }
  while (M < -180) {
    M += 360;
  }
  //#Solve for Eccentric Anomaly
  //#M = E - estar*sin(E)
  double E = M + estar*sin(M*PI/180.0);
  double dM = 1;
  double dE = 0;
  while (abs(dM) > 1e-6) {
    dM = M - (E - estar*sin(E*PI/180.0));
    dE = dM/(1.0-e*cos(E*PI/180.0));
    E += dE;
  }        
  //#Compute the semi latus rectum
  double p = a*(1-e*e);
  //#Compute coordinate of planet in ecliptic plane of the planet
  double xprime = a*(cos(E*PI/180.0)-e);
  double yprime = a*sqrt(1-e*e)*sin(E*PI/180.0);
  double zprime = 0.0;

  //#Convert certain parameters to radians
  w *= PI/180.0;
  OMEGA *= PI/180.0;
  i *= PI/180.0;

  //#Compute coordinate of planet in the J2000 frame or the ecliptic plane of the sun
  double x0 = (cos(w)*cos(OMEGA) - sin(w)*sin(OMEGA)*cos(i))*xprime + (-sin(w)*cos(OMEGA)-cos(w)*sin(OMEGA)*cos(i))*yprime;
  double y0 = (cos(w)*sin(OMEGA) + sin(w)*cos(OMEGA)*cos(i))*xprime + (-sin(w)*sin(OMEGA)+cos(w)*cos(OMEGA)*cos(i))*yprime;
  double z0 = (sin(w)*sin(i))*xprime + (cos(w)*sin(i))*yprime;
  //If you did it right you should get the following coordinates for julian_day = 2458485 (jan 1st 2019)
  //in the J2000 frame 
  //xyz = [-26831520440.7 144634231633.0 -6248426.35536]

  Si.set(1,1,x0);
  Si.set(2,1,y0);
  Si.set(3,1,z0);
}

