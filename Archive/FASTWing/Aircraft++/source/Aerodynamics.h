#ifndef AERO_H
#define AERO_H

#include "MATLAB.h"
#include "Rotation3.h"
#include <fstream>
#include "mathp.h"
#include "timer.h"
#include <Common/FASTPWM.h>
#include "Controller.h"

///APPRENTICE AERO PACK - AFTER PITCH ROLL YAW OSCILLATIONS
//#Initial = [-0.03284800659907888, -0.06906055250375191, -0.17115641081019545, 0.08831709693307076, -0.216]
//#Final = [-0.78826431744966985, -0.33411253656820367, -0.014305373822254552, 0.27846397495417952, 0.061169396698422564]
struct Aero_Pack {
  double CLzero = 0.3441029736813229;
  double CLalpha = 5.185317204810547;
  double CLq = 5.9369349224934975;
  double CLdele = 0.41181051485504044;
  double CDzero = 0.01727851900786801;
  double CDalpha = 1.0080305157535787;
  double Cybeta = -0.24095762862664397;
  double Cydelr = 0.0688450367504697;
  double Cyp = -0.027733042384004744;
  double Cyr = 0.17663419386614151;
  double CLbeta = -0.01633611041536569;
  double CLp =  -0.50160400189302223;
  double CLr = 0.09800094300447591;
  double CLdela = 0.28052396489578457;
  double CLdelr = 0.004667460118675911;
  double Cmzero = -0.07636788929236879;
  double Cmalpha= -2.1859941783855077;
  double Cmq = -24.450947269394277;
  double Cmdele = -1.1530526838451003;
  double Cnp = -0.78826431744966985;
  double Cnbeta = 0.27846397495417952;
  double Cnr = -0.33411253656820367;
  double Cndela = 0.061169396698422564;
  double Cndelr = -0.014305373822254552; //May need to run a sysID flight on rudder to actually get this one
  double cbar=0.2286;// #Aerodynamic chord length: m
  double bws=1.4859;  //#Wing span: m
  double S = cbar*bws;
  double rho = 1.225;
  double W = 12.1; //This has got to be Newtons - Maxwell says yes since the a/c is like 2.72 lbf = 2.72*4.44 ~= 12.0
  double Tmax = 0;//these are set in the setup() routine
  double Kt = 0;
  double max_speed = 15; //this is just from anectodal evidence and looking at plots
};
/*
struct Aero_Pack_ROLL_PITCHSYSID {
  double CLzero = 0.3441029736813229;
  double CLalpha = 5.185317204810547;
  double CLq = 5.9369349224934975;
  double CLdele = 0.41181051485504044;
  double CDzero = 0.01727851900786801;
  double CDalpha = 1.0080305157535787;
  double Cybeta = -0.24095762862664397;
  double Cydelr = 0.0688450367504697;
  double Cyp = -0.027733042384004744;
  double Cyr = 0.17663419386614151;
  double CLbeta = -0.01633611041536569;
  double CLp =  -0.50160400189302223;
  double CLr = 0.09800094300447591;
  double CLdela = 0.28052396489578457;
  double CLdelr = 0.004667460118675911;
  double Cmzero = -0.07636788929236879;
  double Cmalpha= -2.1859941783855077;
  double Cmq = -24.450947269394277;
  double Cmdele = -1.1530526838451003;
  double Cnp = -0.03284800659907888;
  double Cnbeta = 0.08831709693307076;
  double Cnr = -0.06906055250375191;
  double Cndela = -0.216; //Flipped
  double Cndelr = -0.17115641081019545;
  double cbar=0.2286;// #Aerodynamic chord length: m
  double bws=1.4859;  //#Wing span: m
  double S = cbar*bws;
  double rho = 1.225;
  double W = 12.1;
  double Tmax = 0;//these are set in the setup() routine
  double Kt = 0;
  double max_speed = 15; //this is just from anectodal evidence and looking at plots
};
*/
/* 
struct Aero_Pack_PITCHSYSID {
  double CLzero = 0.3441029736813229;
  double CLalpha = 5.185317204810547;
  double CLq = 5.9369349224934975;
  double CLdele = 0.41181051485504044;
  double CDzero = 0.01727851900786801;
  double CDalpha = 1.0080305157535787;
  double Cybeta = -0.24095762862664397;
  double Cydelr = 0.0688450367504697;
  double Cyp = -0.027733042384004744;
  double Cyr = 0.17663419386614151;
  double CLbeta = -0.01633611041536569;
  double CLp = -0.6583142767075709;
  double CLr = 0.09800094300447591;
  double CLdela = 0.19085572159332118; //The sign on all dela's have been flipped 
  double CLdelr = 0.004667460118675911;
  double Cmzero = -0.07636788929236879;
  double Cmalpha= -2.1859941783855077;
  double Cmq = -24.450947269394277;
  double Cmdele = -1.1530526838451003;
  double Cnp = -0.03284800659907888;
  double Cnbeta = 0.08831709693307076;
  double Cnr = -0.06906055250375191;
  double Cndela = -0.216; //Flipped
  double Cndelr = -0.17115641081019545;
  double cbar=0.2286;// #Aerodynamic chord length: m
  double bws=1.4859;  //#Wing span: m
  double S = cbar*bws;
  double rho = 1.225;
  double W = 12.1;
  double Tmax = 0;//these are set in the setup() routine
  double Kt = 0;
  double max_speed = 15; //this is just from anectodal evidence and looking at plots
};
*/
/* 
struct Aero_Pack_PRESYSID { ///The data below was fit using get_aircraft_coefficients.py
  double CLzero = 0.3441029736813229;
  double CLalpha = 5.185317204810547;
  double CLq = 5.9369349224934975;
  double CLdele = 0.41181051485504044;
  double CDzero = 0.01727851900786801;
  double CDalpha = 1.0080305157535787;
  double Cybeta = -0.24095762862664397;
  double Cydelr = 0.0688450367504697;
  double Cyp = -0.027733042384004744;
  double Cyr = 0.17663419386614151;
  double CLbeta = -0.01633611041536569;
  double CLp = -0.6583142767075709;
  double CLr = 0.09800094300447591;
  double CLdela = 0.19085572159332118; //The sign on all dela's have been flipped 
  double CLdelr = 0.004667460118675911;
  double Cmzero = -0.03632475219322319;
  double Cmalpha= -2.5363576941792045;
  double Cmq = -17.11816235985625;
  double Cmdele = -1.0238066966535029;
  double Cnp = -0.03284800659907888;
  double Cnbeta = 0.08831709693307076;
  double Cnr = -0.06906055250375191;
  double Cndela = -0.216; //Flipped
  double Cndelr = -0.17115641081019545;
  double cbar=0.2286;// #Aerodynamic chord length: m
  double bws=1.4859;  //#Wing span: m
  double S = cbar*bws;
  double rho = 1.225;
  double W = 12.1;
  double Tmax = 0;//these are set in the setup() routine
  double Kt = 0;
  double max_speed = 15; //this is just from anectodal evidence and looking at plots
};
*/

class Aerodynamics {
 public:
  MATLAB Faero,Maero;
  void ForceMomentAero(double,MATLAB,MATLAB);
  Aero_Pack aeropack;
  void print_to_file(FILE*);
  Controller ctl;
  double dt,T;
  double computeThrust(double,double);
  int AEROTYPE=0;
  void setup(MATLAB);

  //Constructor
  Aerodynamics();
};

#endif
