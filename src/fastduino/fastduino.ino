//Note that you can't define #defines in this ino script to make everything Arduino specific
//You do have access to ARDUINO so just use #ifdef ARDUINO if you want to do something ARDUINO specific
//Furthermore, you can't import text files on an Arduino so everything
//in Simulation.txt and Config.txt must be placed in here

//Config.txt
#define PRINTRATE 0.1    //!Standard Out Print Rate // seconds (set to negative)
#define LOGRATE 1.0     //!Data logging rate // seconds (numbers if you want)
#define RCRATE 0.1  //!RC Rate // seconds (to run as fast as possible)
#define TELEMRATE 1.0      //!Telemetry Rate (seconds)
// --- GPS ON THE ARDUINO NEEDS TO RUN AS FAST AS POSSIBLE #define GPSRATE 1.0      //!GPS Rate (seconds)
// ---- IMU RUNS AS FAST AS POSSIBLE ON EVERY PLATFORM -99      !IMU Rate (seconds) 
// ---- ANALOG RATE IS NOT NEEDED IN FASTDUINO 0.1      !Analog Rate (seconds)
#define PTHTYPE 4        //!PTHTYPE (0=off,1=MS,2=MPL,3=BME,4=LPS22)
#define IMUTYPE 2        //!IMUTYPE (0=MPU,1=LSM,2=BNO)
//0        !Filter Constant //0 for no filtering and 1.0 for overfiltering
#define CONTROLLERTYPE 12 //!Control System (-X=controlled by PIC,0=ACRO,1=Inner,2=yawrate,1X=turns on guidance)
//2.0  !Mass (kg) // 1U - 2 // 2U - 4 // 6U - 12
//0.00333  !Ixx (kg-m^2) // 1U - 0.00333 //2U,upright - 0.0167 // 2U,sideways - 0.0067 // 6U - 0.13
//0.00333  !Iyy (kg-m^2) // 1U - 0.00333 //2U,upright - 0.0167 // 2U,sideways - 0.0167 // 6U - 0.10
//0.00333  !Izz (kg-m^2) // 1U - 0.00333 //2U,upright - 0.0067 // 2U,sideways - 0.0167 // 6U - 0.05
//0.000    !Ixy (kg-m^2)
//0.000    !Ixz (kg-m^2)
//0.000    !Iyz (kg-m^2)
int NUMTELEMETRY = 4; ///Number of telemetry variables to be sent. 

///DEBUG FLAGS
#define USETIMER
//#define USEGPS
#define USEIMU
//#define TELEMETRY
#define RCSIGNALS
#define LOGDATA
#define USEPTH
//#define GUIDANCE

//Note when setting this up for the first time. 
//Install the Due board by going to the board manager
//Go to preferences and change the location of the libraries to ~/FASTCASST

//Timer.h for realtime clock - Tested on 2/28/2023 and it compiles and runs
#ifdef USETIMER
#include "timer.h"
TIMER watch;
#endif

//Include the hardware environment
//#include "hardware.h"

///DEBUG HEADER FILES
//#include "mathp.h" - Tested and compiles 3/9/2023

//Need MATLAB.h for Matrices 
//#include "MATLAB.h" - Tested and compiles 3/9/2023

//Datalogger is inside hardware.h
#ifdef LOGDATA
#include "Datalogger.h" //Compiles but does not work yet - Keep getting SD card init failed
Datalogger logger; //Remember that you need to have a data/ folder on the SD card
#endif

//RCIO is inside hardware.h 
//#include "RCIO.h"
//RCIO rc;

//RCInput is inside RCIO.h - Compiles on 2/28/2023 and Runs on 2/28/2023
#ifdef RCSIGNALS
#include "RCInput.h"
RCInput rin;

//RCOutput is inside RCIO.h - Compiles and runs 2/28/2023
#include "RCOutput.h"
RCOutput rout;
#endif

//PWMSIGNALS.h is inside RCIO.h - Compiles and runs 2/28/2023
//#include "PWMSIGNALS.h"

//Hardware also has the Comms class for wired and wireless communication - Compiled and tested on 3/17/2023
#ifdef TELEMETRY
#include "Comms.h"
Comms serTelem;
MATLAB telemetry_matrix;
#endif

//Hardware has sensors.h
//#include "sensors.h"
//For now we will just create a sense matrix - This is only needed for GUIDANCE
#ifdef GUIDANCE
MATLAB sense_matrix;
#endif

//Sensors has a lot of sensors. We're going to need to add them in one at a time
//Let's start with GPS
#ifdef USEGPS
#include "GPS.h" // Compiles, runs and gets a GPS fix - 3/9/2023
GPS satellites; 
#endif

//Now let's get the barometer working
//#include "BaroTemp.h" //Compiles, runs and gets a valid pressure reading - 3/9/2023
//BaroTemp atm;
//Switched to new PTH class - also compiles runs and gets a valid reading 4/4/2023
#ifdef USEPTH
#include "PTH.h"
PTH atm;
#endif

//And finally it's time for the IMU -- Compiles 3/20/2023 - Does it work? - It does now. Whew that was hard 4/4/2023
//Note that I haven't checked the orientation so you might get the wrong angles
#ifdef USEIMU
#include "IMU.h"
IMU orientation;
#endif

//The other part we need is the quadcopter autopilot class
#ifdef GUIDANCE
#include "quadcopter_controller.h"
controller control;
#endif

//Create Loop Variables
double lastPRINTtime = 0;
double lastLOGtime = 0;
double lastTELEMtime = 0;
double lastRCtime = 0;
double lastGPStime = 0;

void setup() {
  //Setup Serial Std Out
  Serial.begin(115200);

  ///Print dummy version number
  Serial.print("FASTKit Software Version 42.0 \n");
  
  ///Initialize the timer
  #ifdef USETIMER
  Serial.print("Initializing Timer...\n");
  watch.init(0);
  Serial.print("Timer Initialized \n");
  #endif
  
  //Hardware init

  //DEBUGGING
  //Initialize Datalogger
  //Time plus the receiver and PWM out signals
  //3 LLH signals, 3 barometer values
  //3 Euler Angles and 3 Euler rates
  #ifdef LOGDATA
  logger.init("data/",1+RECV_N_CHANNEL*2+3+3+6); 
  //Remember the SD card on the arduino needs to have a data/ folder
  //Append the headers
  logger.appendheader("Time (sec)");
  char** rinnames = (char**)malloc((RECV_N_CHANNEL)*sizeof(char*));
  for (int i = 0;i<RECV_N_CHANNEL;i++) {
    rinnames[i] = (char*)malloc((18)*sizeof(char));
    sprintf(rinnames[i],"%s%d%s","RXIN ",i," (us)");
    logger.appendheader(rinnames[i]);
  }
  char** routnames = (char**)malloc((RECV_N_CHANNEL)*sizeof(char*));
  for (int i = 0;i<RECV_N_CHANNEL;i++) {
    routnames[i] = (char*)malloc((18)*sizeof(char));
    sprintf(routnames[i],"%s%d%s","PWMOUT ",i," (us)");
    logger.appendheader(routnames[i]);
  }
  logger.appendheader("Latitude (deg)");
  logger.appendheader("Longitude (deg)");
  logger.appendheader("Altitude (m)");  
  logger.appendheader("Pressure (Pa)"); 
  logger.appendheader("Temperature (C)");
  logger.appendheader("Pressure Altitude (m)"); 
  logger.appendheader("Roll (deg)");
  logger.appendheader("Pitch (deg)");
  logger.appendheader("Yaw (deg)");
  logger.appendheader("Roll Rate (rad/s)");
  logger.appendheader("Pitch Rate (rad/s)");
  logger.appendheader("Yaw Rate (rad/s)");
  logger.printheaders();
  #endif

  //Initialize Telemetry
  #ifdef TELEMETRY
  Serial.print("Initializing Telemetry \n");
  telemetry_matrix.zeros(NUMTELEMETRY,1,"Telemetry Matrix");
  serTelem.TelemInit(NUMTELEMETRY);
  #endif

  //Initialize the IMU
  #ifdef USEIMU
  orientation.init(IMUTYPE);
  #endif
  
  #ifdef RCSIGNALS
  //rc.outInit(RECV_N_CHANNEL);
  //Initialize RCInput
  rin.initialize();
  //Initialize RCOutputr
  rout.initialize(RECV_N_CHANNEL);
  #endif

  #ifdef USEGPS
  //initialize GPS
  satellites.init();
  #endif

  #ifdef USEPTH
  //Initialize Barometer
  atm.init(PTHTYPE);
  #endif

  #ifdef GUIDANCE
  //Initialize the controller
  control.init(CONTROLLERTYPE);
  //Initialize Sense Matrix
  sense_matrix.zeros(29,1,"Full State From Sensors");
  #endif
}

#ifdef GUIDANCE
void populate() {
  //Initialize everything to -99
  sense_matrix.mult_eq(0);
  sense_matrix.plus_eq(-99);
  //sense_matrix_dot.mult_eq(0);
  //sense_matrix_dot.plus_eq(-99);

  ///XYZ
  #ifdef USEGPS
  sense_matrix.set(1,1,satellites.X);
  sense_matrix.set(2,1,satellites.Y);
  #endif
  #ifdef USEPTH
  //sense_matrix.set(3,1,(satellites.Z-atm.altitude)/2.0);  //Average GPS and BARO?
  sense_matrix.set(3,1,-atm.altitude); //Let's use Barometer as altitude sensor
  //sense_matrix.set(3,1,-atm.altitude);
  #endif

  //Roll pitch Yaw
  #ifdef USEIMU
  sense_matrix.set(4,1,orientation.roll);
  sense_matrix.set(5,1,orientation.pitch);

  ///Yaw Angle IS A COMBINATION OF IMU AND GPS
  //getCompassHeading();
  sense_matrix.set(6,1,orientation.yaw);
  #endif

  //UWV
  #ifdef USEGPS
  //Assume that the vehicle is traveling straight so V and W are zero
  sense_matrix.set(7,1,satellites.speed);
  #endif
  sense_matrix.set(8,1,0);
  sense_matrix.set(9,1,0);

  //PQR
  #ifdef USEIMU
  sense_matrix.set(10,1,orientation.roll_rate);
  sense_matrix.set(11,1,orientation.pitch_rate);
  sense_matrix.set(12,1,orientation.yaw_rate);

  //MXYZ
  sense_matrix.set(13,1,orientation.mx);
  sense_matrix.set(14,1,orientation.my);
  sense_matrix.set(15,1,orientation.mz);
  #endif
  //sense_matrix.set(15,1,Heading_Mag);

  //GPS
  #ifdef USEGPS
  sense_matrix.set(16,1,satellites.latitude);
  sense_matrix.set(17,1,satellites.longitude);
  sense_matrix.set(18,1,satellites.altitude);
  sense_matrix.set(19,1,satellites.heading); //THe GPS Heading
  #endif

  //IMU
  #ifdef USEIMU
  sense_matrix.set(20,1,orientation.yaw); //IMU Heading
  #endif

  //Analog
  //sense_matrix.vecset(21,26,analog.results,1);

  //Barometera
  #ifdef USEPTH
  sense_matrix.set(27,1,atm.pressure);
  sense_matrix.set(28,1,atm.altitude);
  sense_matrix.set(29,1,atm.temperature);
  #endif

}
#endif

void loop() {

  #ifdef USETIMER
  //Update Timer
  watch.updateTime();
  #endif
  
  #ifdef RCSIGNALS  
  //Update RC Signals
  //rc.read();
  //Read the receiver
  if (lastRCtime <= watch.currentTime) {
    rin.readRCstate();  
  }
  #endif
  
  #ifdef USEGPS
  //Poll GPS
  satellites.poll(watch.currentTime);
  #endif

  #ifdef USEPTH
  //Poll Barometer
  if (PTHTYPE) {
    atm.poll(watch.currentTime);
  }
  #endif

  #ifdef USEIMU
  //Poll IMU
  orientation.loop(watch.elapsedTime);
  #endif

  #ifdef GUIDANCE
  //Populate Sense Matrix
  populate();
  //Run the Controller.
  control.loop(watch.currentTime,rin.rx_array,sense_matrix);
  //I then need to populate the pwm_array with the control signals as quickly as possible
  for (int i = 0;i<control.NUMSIGNALS;i++) {
    rout.pwm_array[i] = int(control.control_matrix.get(i+1,1));
  }
  #endif
  
  #ifdef RCSIGNALS
  //Send signals to PWM channels
  rout.write();
  #endif
  
  //Print Everything
  if (lastPRINTtime <= watch.currentTime) {
      lastPRINTtime+=PRINTRATE;
      #ifdef USETIMER
      Serial.print("T = ");
      Serial.print(watch.currentTime);
      Serial.print(" ");
      Serial.print(watch.elapsedTime);
      #endif
      #ifdef RCSIGNALS
      Serial.print(" RX = ");
      rin.printRCstate(-6);
      //rc.in.printRCstate(-5);
      Serial.print(" PWM = ");
      rout.print();
      //rc.out.print();
      #endif
      #ifdef USEGPS
      Serial.print(" LLH = ");
      satellites.printLLH();
      #endif
      #ifdef USEPTH
      Serial.print(" PTHAlt = ");
      atm.print();
      #endif
      #ifdef USEIMU
      Serial.print(" Euler = ");
      orientation.printALL();      
      #endif
      Serial.print("\n");
  }

  //Send data via telemetry
  #ifdef TELEMETRY
  if (lastTELEMtime <= watch.currentTime) {
    Serial.print("Sending Telemetry \n");
    #ifdef USETIMER
    telemetry_matrix.set(1,1,watch.currentTime);
    #endif
    #ifdef USEPTH
    telemetry_matrix.set(2,1,atm.pressure);
    telemetry_matrix.set(3,1,atm.temperature);
    telemetry_matrix.set(4,1,atm.altitude);
    #endif
    serTelem.sendTelemetry(telemetry_matrix,0);
    lastTELEMtime+=TELEMRATE;
  }
  #endif

  //Log Everything
  #ifdef LOGDATA
  if (lastLOGtime <= watch.currentTime) {
    lastLOGtime+=LOGRATE;
    #ifdef USETIMER
    logger.printvar(watch.currentTime);
    logger.writecomma();
    #endif
    #ifdef RCSIGNALS
    logger.printarray(rin.rx_array,RECV_N_CHANNEL);
    logger.writecomma();
    logger.printarray(rout.pwm_array,RECV_N_CHANNEL);
    logger.writecomma();
    #endif
    #ifdef USEGPS
    logger.printvar(satellites.latitude);
    logger.writecomma();
    logger.printvar(satellites.longitude);
    logger.writecomma();
    logger.printvar(satellites.altitude);
    logger.writecomma();
    #endif
    #ifdef USEPTH
    logger.printvar(atm.pressure);
    logger.writecomma();
    logger.printvar(atm.temperature);
    logger.writecomma();
    logger.printvar(atm.altitude);
    logger.writecomma();
    #endif
    #ifdef USEIMU
    logger.printvar(orientation.roll);
    logger.writecomma();
    logger.printvar(orientation.pitch);
    logger.writecomma();
    logger.printvar(orientation.yaw);
    logger.writecomma();
    logger.printvar(orientation.roll_rate);
    logger.writecomma();
    logger.printvar(orientation.pitch_rate);
    logger.writecomma();
    logger.printvar(orientation.yaw_rate);
    #endif
    logger.writenewline();
  }
  #endif
  //cross_sleep(0.5);
}
