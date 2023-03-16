//Note that you can't define #defines in this ino script to make everything Arduino specific
//You do have access to ARDUINO so just use #ifdef ARDUINO if you want to do something ARDUINO specific
//Furthermore, you can't import text files on an Arduino so everything
//in Simulation.txt and Config.txt must be placed in here

//Config.txt
#define PRINTRATE 1.0    //!Standard Out Print Rate // seconds (set to negative)
#define LOGRATE 1.0     //!Data logging rate // seconds (numbers if you want)
//0.1  !RC Rate // seconds (to run as fast as possible)
#define TELEMRATE 1.0      //!Telemetry Rate (seconds)
//1.0      !GPS Rate (seconds)
//-99      !IMU Rate (seconds) (does not work at the moment)
//0.1      !Analog Rate (seconds)
//1        !Poll Barometer (0=off,1=on)
//0        !IMUTYPE (0=MPU,1=LSM)
//0        !Filter Constant //0 for no filtering and 1.0 for overfiltering
//2 !Control System (3=two-stage,2=FL,1=PID,0=off)
//2.0  !Mass (kg) // 1U - 2 // 2U - 4 // 6U - 12
//0.00333  !Ixx (kg-m^2) // 1U - 0.00333 //2U,upright - 0.0167 // 2U,sideways - 0.0067 // 6U - 0.13
//0.00333  !Iyy (kg-m^2) // 1U - 0.00333 //2U,upright - 0.0167 // 2U,sideways - 0.0167 // 6U - 0.10
//0.00333  !Izz (kg-m^2) // 1U - 0.00333 //2U,upright - 0.0067 // 2U,sideways - 0.0167 // 6U - 0.05
//0.000    !Ixy (kg-m^2)
//0.000    !Ixz (kg-m^2)
//0.000    !Iyz (kg-m^2)
int NUMTELEMETRY = 4; ///Number of telemetry variables to be sent. 

//Note when setting this up for the first time. 
//Install the Due board by going to the board manager
//Go to preferences and change the location of the libraries to ~/FASTCASST

//Timer.h for realtime clock - Tested on 2/28/2023 and it compiles and runs
#include "timer.h"
TIMER watch;

//Include the hardware environment
//#include "hardware.h"

///DEBUG HEADER FILES
//#include "mathp.h" - Tested and compiles 3/9/2023

//Need MATLAB.h for Matrices 
//#include "MATLAB.h" - Tested and compiles 3/9/2023

//Datalogger is inside hardware.h
#include "Datalogger.h" //Compiles but does not work yet - Keep getting SD card init failed
Datalogger logger; //Remember that you need to have a data/ folder on the SD card

//RCIO is inside hardware.h 
//#include "RCIO.h"
//RCIO rc;

//RCInput is inside RCIO.h - Compiles on 2/28/2023 and Runs on 2/28/2023
#include "RCInput.h"
RCInput rin;

//RCOutput is inside RCIO.h - Compiles and runs 2/28/2023
#include "RCOutput.h"
RCOutput rout;

//PWMSIGNALS.h is inside RCIO.h - Compiles and runs 2/28/2023
//#include "PWMSIGNALS.h"

//Hardware also has the Comms class for wired and wireless communication
#include "Comms.h"
Comms serTelem;
MATLAB telemetry_matrix;

//Hardware has sensors.h
//#include "sensors.h"

//Sensors has a lot of sensors. We're going to need to add them in one at a time
//Let's start with GPS
#include "GPS.h" // Compiles, runs and gets a GPS fix - 3/9/2023
GPS satellites; 

//Now let's get the barometer working
//#include "BaroTemp.h" //Compiles, runs and gets a valid pressure reading - 3/9/2023
//BaroTemp atm;

//Create Loop Variables
double lastPRINTtime = 0;
double lastLOGtime = 0;
double lastTELEMtime = 0;

void setup() {
  //Setup Serial Std Out
  Serial.begin(115200);

  ///Print dummy version number
  Serial.print("FASTKit Software Version 42.0 \n");
  
  ///Initialize the timer
  Serial.print("Initializing Timer...\n");
  watch.init(0);
  Serial.print("Timer Initialized \n");
  
  //Hardware init

  //DEBUGGING
  //Initialize Datalogger
  logger.init("data/",1+RECV_N_CHANNEL*2+3); //Time plus the receiver signals and the PWM out signal and 3 LLH signals
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
  logger.printheaders();

  //Initialize Telemetry
  telemetry_matrix.zeros(NUMTELEMETRY,1,"Telemetry Matrix");
  serTelem.TelemInit(NUMTELEMETRY);
  
  //rc.outInit(RECV_N_CHANNEL);
  //Initialize RCInput
  rin.initialize();
  //Initialize RCOutputr
  rout.initialize(RECV_N_CHANNEL);
  //initialize GPS
  satellites.init();
  //Initialize Barometer
  //atm.init();
  
}

void loop() {
  //Update Timer
  watch.updateTime();
  //Update RC Signals
  //rc.read();

  //DEBUGGING
  rin.readRCstate();
  //Copy rin.rx_array to rout.pwm_array
  for (int idx = 0;idx<RECV_N_CHANNEL;idx++) {
    rout.pwm_array[idx] = rin.rx_array[idx];
  }
  //Send signals to PWM channels
  rout.write();
  
  //Poll GPS
  satellites.poll(watch.currentTime);

  //Poll Barometer
  //atm.poll(watch.currentTime);
  
  //Print Everything
  if (lastPRINTtime <= watch.currentTime) {
      lastPRINTtime+=PRINTRATE;
      Serial.print("T = ");
      Serial.print(watch.currentTime);
      Serial.print(" ");
      Serial.print(watch.elapsedTime);
      Serial.print(" RX = ");
      rin.printRCstate(-6);
      //rc.in.printRCstate(-5);
      Serial.print(" PWM = ");
      rout.print();
      //rc.out.print();
      Serial.print(" LLH = ");
      satellites.printLLH();
      //Serial.print(" PTAlt = ");
      //atm.print();
      Serial.print("\n");
  }

  //Send data via telemetry
  if (lastTELEMtime <= watch.currentTime) {
    telemetry_matrix.set(1,1,watch.currentTime);
    telemetry_matrix.set(2,1,satellites.latitude);
    telemetry_matrix.set(3,1,satellites.longitude);
    telemetry_matrix.set(4,1,satellites.altitude);
    serTelem.sendTelemetry(telemetry_matrix,0);
    lastTELEMtime+=TELEMRATE;
  }

  //Log Everything
  if (lastLOGtime <= watch.currentTime) {
    lastLOGtime+=LOGRATE;
    logger.printvar(watch.currentTime);
    logger.writecomma();
    logger.printarray(rin.rx_array,RECV_N_CHANNEL);
    logger.writecomma();
    logger.printarray(rout.pwm_array,RECV_N_CHANNEL);
    logger.writecomma();
    logger.printvar(satellites.latitude);
    logger.writecomma();
    logger.printvar(satellites.longitude);
    logger.writecomma();
    logger.printvar(satellites.altitude);
    logger.writenewline();
  }
  //cross_sleep(0.1);
}
