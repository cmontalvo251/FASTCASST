//This is used on Nghia Huynh's autopilot
// FASTSensor sensor package includes GPS, 9DOF (10DOF Footprint. This isn't the Bosch one), Accelerometer, SD card, and reads servo signals.

//IMU//
#include <Adafruit_10DOF.h> //THE REASON THIS CODE IS CALLED 9DOF is Because it's using the Adafruit_10DOF footprint
//but this one does not have a pressure sensor to make it 10DOF. The 10DOF sensor has been discontinued. When I think of 9DOF
//I think of the 9DOF sensor made by Bosch but this one is different.

//The 9DOF sensor is plugged into I2C and all soldered onto one chip. It's pretty nice.
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
//Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001); //This is why the pressure stuff is commented out. I've left it here just in case
//Probably need to delete it in later revisions.
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
sensors_event_t event; //Creates an event instance to get 9DOF data.

typedef struct vector
{
  float x;
  float y;
  float z;
};

vector gyro_data;
vector accel_data;
vector mag_data;
#define PLOTRAW 0

//Arduino Time
float lastPrint = 0;
float last9DOF = 0;
float lastAUTO = 0;
    
// Added complementary filter stuff here    
float sigma = 0.15;
float roll_prev = 0, pitch_prev = 0 , yaw_prev = 0;
float roll_filter, pitch_filter, yaw_filter;
    
//GPS//
#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Serial1);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//Servo//
#include <Servo.h>
Servo AIL1,AIL2,ELE,RUD; //aileron, elevator, RUDD servos.
//unsigned long current_time,timerA,timerE; //This is for reading but we don't need to read
//byte last_channelA,last_channelE; //any receiver signals
//int pulse_timeA,pulse_timeE;

//Autopilot
float aileronSignal, aileronSignal2, elevatorSignal, rudderSignal;
float roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate,yaw_init;
  
//SD//
#include <SPI.h> // apparently I need this too
#include <SD.h>
File myFile;

/// ALL THIS HERE FOR REFERENCE
//void displaySensorDetails(void)
//{
//  sensor_t sensor;
//  
//  accel.getSensor(&sensor);
//  Serial.println(F("----------- ACCELEROMETER ----------"));
//  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
//  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
//  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
//  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
//  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
//  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
//  Serial.println(F("------------------------------------"));
//  Serial.println(F(""));
//
//  gyro.getSensor(&sensor);
//  Serial.println(F("------------- GYROSCOPE -----------"));
//  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
//  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
//  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
//  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
//  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
//  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
//  Serial.println(F("------------------------------------"));
//  Serial.println(F(""));
//  
//  mag.getSensor(&sensor);
//  Serial.println(F("----------- MAGNETOMETER -----------"));
//  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
//  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
//  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
//  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
//  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
//  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));  
//  Serial.println(F("------------------------------------"));
//  Serial.println(F(""));
//
//  bmp.getSensor(&sensor);
//  Serial.println(F("-------- PRESSURE/ALTITUDE ---------"));
//  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
//  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
//  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
//  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" hPa"));
//  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" hPa"));
//  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" hPa"));  
//  Serial.println(F("------------------------------------"));
//  Serial.println(F(""));
  
//}


///////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
  // SD //
  Serial.begin(115200);
  Serial.print("Initializing SD card...");
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (53 on the Mega) must be left as an output or the SD library functions will not work. 
  pinMode(53, OUTPUT);
  
  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    while(1){};
  }
  Serial.println("initialization done.");
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  char filename[15];
  strcpy(filename, "FAST000.TXT");
  for (uint8_t i = 0; i < 1000; i++) {
    filename[4] = '0' + i/100;
    filename[5] = '0' + (i/10)%10;
    filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  char lastfile[15];
  strcpy(lastfile,"FAST999.TXT");
  if (SD.exists(lastfile)){
    Serial.print("Sorry SD card has reached its naming limit. Suggest wiping SD card");
    while(1){};
  }
  
  myFile = SD.open(filename, FILE_WRITE);
 
  // if the file opened okay, write to it:
  if (!myFile) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    while(1){};
  }
  Serial.print("Writing to "); 
  Serial.println(filename);
  
  // Servo //
  // Writting to pins 8 9 and 10
  AIL1.attach(8);
  AIL2.attach(35);
  ELE.attach(41);
  RUD.attach(9);
  
  // 10DOF//
  Serial.println(F("Adafruit 10DOF Tester")); Serial.println("");

  //It looks like the LSM303 has an accelerometer and magnetometer on board. 
  //The L3GD20 is the gyroscope
  
  /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 Accel Module ... check your connections */
    Serial.println(F("Ooops, no LSM303 Accel Module detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 Magnetometer Module detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  // GPS //
  GPS.begin(9600);
  
  Serial.print("GPS Class Initialized \n");
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //turn on RMC(recommended minimum)
  Serial.print("NMEA OUTPUT Set \n");
  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);  // 1 or 10 Hz update rate. what should i use??
  //FASTPilot is set at 10HZ right now but I wonder if that's what's causing the jitter
  //issue. Could be one possible source for error.
  
  Serial.print("Update Rate Set \n");
  
  useInterrupt(false);
  
  Serial.print("Delaying for  1 second \n");
  delay(1000);

  Serial.print("Calibrating Yaw Angle -- DO NOT MOVE!!!!! \n");
  int NUMCYCLES = 100;
  for (int idx = 0;idx < NUMCYCLES;idx++) {
    accel.getEvent(&event); //Need to move this routine to a separate millis timer.
    accel_data.x = event.acceleration.x;
    accel_data.y = event.acceleration.y;
    accel_data.z = event.acceleration.z;
    mag.getEvent(&event);
    mag_data.x = event.magnetic.x;
    mag_data.y = event.magnetic.y;
    mag_data.z = event.magnetic.z;
    //Conversion to Euler Angles event.acceleration (m/s^2) event.gyro (rad/s) event.magnetic (uT - Micro Teslas - 10^-6)
    roll = atan2(accel_data.y,accel_data.z);
    float sroll = sin(roll);
    float croll = cos(roll);
    pitch = atan2(-accel_data.x,accel_data.y*sroll + accel_data.z*croll);
    float spitch = sin(pitch);
    float cpitch = cos(pitch);
    yaw = atan2(mag_data.z*sroll-mag_data.y*croll,mag_data.x*cpitch+mag_data.y*spitch*sroll+mag_data.z*spitch*croll);
    yaw_init += yaw;
  }
  yaw_init /= float(NUMCYCLES);
  
  Serial.print("Ok go ahead \n");
  
  lastPrint = millis()/1000.0;
  last9DOF = millis()/1000.0;
  lastAUTO = millis()/1000.0;
}


/////////////////////////////////////////////////////////////////////////////////////
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMERO_COMPA_vect) {
  char c = GPS.read();
  if (GPSECHO)
    if (c) UDR0 = c; // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
    }
}

uint32_t timer = millis();
///////////////////////////////////////////////////////////////////////////////////////
void loop() {
  
   // GPS //
  if (! usingInterrupt) {
     // read data from the GPS in the 'main loop'
     char c = GPS.read();
     // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
      //if (c) UDR0 = c; //writing direct to UDR0 is much faster than Serial.print
      //but only one character can be written at a time.
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
    return;
  }

  //Separate Loop for 9DOF sensor
  if (millis()/1000.0 - last9DOF > 0.01) { //Setting this to 0 means it will run every loop. Need to check for jitter. May need to throttle this down
    last9DOF = millis()/1000.0;
    
    
    accel.getEvent(&event); //Need to move this routine to a separate millis timer.
    accel_data.x = event.acceleration.x;
    accel_data.y = event.acceleration.y;
    accel_data.z = event.acceleration.z;
    mag.getEvent(&event);
    mag_data.x = event.magnetic.x;
    mag_data.y = event.magnetic.y;
    mag_data.z = event.magnetic.z;
    gyro.getEvent(&event);
    gyro_data.x = event.gyro.x;
    gyro_data.y = event.gyro.y;
    gyro_data.z = event.gyro.z;
    //Compute Roll, Pitch, Yaw Euler Angles and Rates using the information from above.
    roll_rate = gyro_data.x;
    pitch_rate = gyro_data.y;
    yaw_rate = gyro_data.z;
    //Conversion to Euler Angles event.acceleration (m/s^2) event.gyro (rad/s) event.magnetic (uT - Micro Teslas - 10^-6)
    roll = atan2(accel_data.y,accel_data.z);
    
    roll_filter = roll*(sigma) + (1-sigma)*(roll_prev);
    
    roll_prev = roll_filter;
    
    float sroll = sin(roll_filter);
    float croll = cos(roll_filter);
    pitch = atan2(-accel_data.x,accel_data.y*sroll + accel_data.z*croll);
    
    roll_filter *= pow(cos(pitch),2); // To avoid singularity
    
    pitch_filter = pitch*(sigma) + (1-sigma)*(pitch_prev);
    pitch_prev = pitch_filter;
    
    float spitch = sin(pitch_filter);
    float cpitch = cos(pitch_filter);
    yaw = atan2(mag_data.z*sroll-mag_data.y*croll,mag_data.x*cpitch+mag_data.y*spitch*sroll+mag_data.z*spitch*croll);
    //Yaw offset
    yaw -= yaw_init;
    yaw *= pow(cos(pitch),2); // To avoid singularity
    
    yaw_filter   = yaw*(sigma) + (1-sigma)*(yaw_prev);
    yaw_prev = yaw;
  }

  //Autopilot - May need to throttle this down depending on any jitter.
  if (millis()/1000.0 - lastAUTO > 0.02) {
    lastAUTO = millis()/1000.0;
    float roll_command = 0;
    float pitch_command = 0;
    float kp_roll = 57.2958,kp_pitch = 57.2958,kp_yaw = 0;
    float kd_roll = 0,kd_pitch = 0,kd_yaw = 6.5;
    elevatorSignal = kp_roll*(roll_filter - roll_command) + kd_roll*roll_rate + 90.0;
    aileronSignal = kp_pitch*(pitch_filter - pitch_command) + kd_pitch*pitch_rate + 90.0;
    rudderSignal = kd_yaw*yaw_rate + 90.0; //The hope is that yaw stability will take over and if we just dampen the rate we should be ok

    // Limit the control surface signals here
//    if (aileronSignal > 30.0 + 90.0){
//      aileronSignal = 30.0;
//      }
//      else if (aileronSignal < -30.0 - 90.0){
//      aileronSignal = -30.0;        
//      }
//    if (elevatorSignal > 30.0 + 90.0){
//      elevatorSignal = 30.0 + 90.0;
//      }
//      else if (elevatorSignal < -30.0 - 90.0){
//      elevatorSignal = -30.0 -90.0;        
//      }
//    if (rudderSignal > 30.0 + 90.0){
//      rudderSignal = 30.0 + 90.0;
//      }
//      else if (rudderSignal < -30.0 -90.0){
//      rudderSignal = -30.0 - 90.0;
//      }
     
  }

  // Send Signals to servos - I think this should happen every loop but I might be wrong.
  // Reverse aileron and elevator signal due to the orientation of the mounting sensor
  AIL1.write(aileronSignal);
  AIL2.write(aileronSignal);
  ELE.write(elevatorSignal);
  RUD.write(rudderSignal);

  //printing if clause and writing to SD card
  if (millis()/1000.0 - lastPrint  > 0.1) { ///0.2 second so that would be 5 Hz. 
    lastPrint = millis()/1000.0; // reset timer
    //Serial.print(GPS.year); Serial.print(" ");
    //Serial.print(GPS.month); Serial.print(" ");
    //Serial.print(GPS.day); Serial.print(" ");
    Serial.print(GPS.hour, DEC); Serial.print(' ');
    Serial.print(GPS.minute, DEC); Serial.print(' ');
    Serial.print(GPS.seconds, DEC); Serial.print(' ');
    Serial.print(lastPrint); Serial.print(" ");
    //Serial.print((int)GPS.fix); Serial.print(" ");
    Serial.print(GPS.latitudeDegrees, 8); Serial.print(" ");
    Serial.print(GPS.longitudeDegrees, 8); Serial.print(" "); 
    Serial.print(GPS.speed); Serial.print(" "); //This is speed in knots
    Serial.print(GPS.angle); Serial.print(" "); //According to this website
    Serial.print(GPS.altitude); Serial.print(" "); //http://www.gpsinformation.org/dale/nmea.htm

    //Euler Angles and Rates
    Serial.print(roll);Serial.print(" ");
//    Serial.print(roll_filter);Serial.print(" ");
    Serial.print(pitch);Serial.print(" ");
    Serial.print(yaw);Serial.print(" ");
    Serial.print(roll_rate);Serial.print(" ");
    Serial.print(pitch_rate);Serial.print(" ");
    Serial.print(yaw_rate);Serial.print(" ");

    #if PLOTRAW
    Serial.print(accel_data.x);Serial.print(" ");
    Serial.print(accel_data.y);Serial.print(" ");
    Serial.print(accel_data.z);Serial.print(" ");
    Serial.print(mag_data.x);Serial.print(" ");
    Serial.print(mag_data.y);Serial.print(" ");
    Serial.print(mag_data.z);Serial.print(" ");
    Serial.print(gyro_data.x);Serial.print(" ");
    Serial.print(gyro_data.y);Serial.print(" ");
    Serial.print(gyro_data.z);Serial.print(" ");
    #endif
    //Servo Signals
    Serial.print(aileronSignal); Serial.print(" ");
    Serial.print(elevatorSignal); Serial.print(" ");
    Serial.print(rudderSignal); Serial.print(" ");

    Serial.print("\n");
    
    //Ok now everything from above is copied except Serial is replaced with myFile and myFile.flush() is at the bottom.
    //All 9DOF.getEvents are also missing to avoid duplication.
    //myFile.print(GPS.year); myFile.print(" ");
    //myFile.print(GPS.month); myFile.print(" ");
    //myFile.print(GPS.day); myFile.print(" ");
//    myFile.print(GPS.hour, DEC); myFile.print(' ');
//    myFile.print(GPS.minute, DEC); myFile.print(' ');
//    myFile.print(GPS.seconds, DEC); myFile.print(' ');
    myFile.print(lastPrint); myFile.print(" ");
    myFile.print((int)GPS.fix); myFile.print(" ");
    myFile.print(GPS.latitudeDegrees, 8); myFile.print(" ");
    myFile.print(GPS.longitudeDegrees, 8); myFile.print(" "); 
    myFile.print(GPS.speed); myFile.print(" "); //This is speed in knots
    myFile.print(GPS.angle); myFile.print(" "); //According to this website
    myFile.print(GPS.altitude); myFile.print(" "); //http://www.gpsinformation.org/dale/nmea.html

    //Euler Angles and Rates
//    myFile.print(roll);myFile.print(" ");
    myFile.print(pitch_filter);myFile.print(" "); // This is actually the roll angle bc of the way the FASTsensor is mounted
//    myFile.print(pitch);myFile.print(" ");
    myFile.print(roll_filter);myFile.print(" "); // This is actually the pitch angle bc of the way the FASTsensor is mounted
//    myFile.print(yaw);myFile.print(" ");
    myFile.print(yaw_filter);myFile.print(" ");
    myFile.print(roll_rate);myFile.print(" ");
    myFile.print(pitch_rate);myFile.print(" ");
    myFile.print(yaw_rate);myFile.print(" ");

    //Servo Signals
    myFile.print(aileronSignal); myFile.print(" ");
    myFile.print(elevatorSignal); myFile.print(" ");
    myFile.print(rudderSignal); myFile.print(" ");

    myFile.print("\n");

    myFile.flush();
  }
}


// THIS IS ALL FOR READING. NO NEED FOR THIS RIGHT NOW

//////////////////////////////////////////////////////////////////////////////////////
//ISR(PCINT0_vect) { //for PORTB0-7
//
//  current_time = micros();
//
//  //Reading elevator from pin 12
//  if (PINB & B01000000) { 
//  
//    if(last_channelE == 0)
//    {
//      last_channelE = 1;
//      timerE = current_time;
//    }
//  }
//  else if(last_channelE == 1){
//      last_channelE = 0;
//      pulse_timeE = current_time - timerE;
//  }
//
// //Reading aileron from pin 11
// if (PINB & B00100000) { 
//  
//    if(last_channelA == 0)
//    {
//      last_channelA = 1;
//      timerA = current_time;
//    }
//  }
//  else if(last_channelA == 1){
//      last_channelA = 0;
//      pulse_timeA = current_time - timerA;
//  }
//
//}
