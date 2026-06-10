//#include <SPI.h>

//#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>

//If you want to use the ISR() functions you need to use the FASTGPS.h and FASTSerial.h libraries

#include <FASTGPS.h>
#include <FASTSerial.h>
//#include <SD.h>
#include <avr/sleep.h>

//Get PWMServo Library - Make sure to get version 2.
#include <PWMServo.h>

// Ladyada's logger modified by Bill Greiman to use the SdFat library
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS Shield
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada
// Fllybob added 10 sec logging option
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true
/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
#define LOG_FIXONLY false  

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// Set the pins used
#define chipSelect 10
#define ledPin 13
#define pulsePin_thrust 3
#define pulsePin_steer 5
#define servo0 540
#define servo180 2390
#define steerRight 1975
#define steerLeft 1010
#define steerMiddle 1560
#define thrustForwad 1900
#define thrustRest 1460
#define thrust_half 1680
#define thrustBrake 1100
#define GPSVAL 111119.999999921
#define ORIGINLAT 30.69139862
#define ORIGINLON -88.17546844
#define waypoint_lat 30.69139862
#define waypoint_lon  -88.17613983
//File logfile;

int pulseMap_steer = steerMiddle, pulseMap_thrust = thrustRest, pulseRead_steer, pulseRead_steer_new, pulseRead_thrust, auto_steer;
int press_count = 0,autopilot_count = 0;

boolean autopilot = false;

boolean PRESS = false;

float latitudeDegrees=0,longitudeDegrees=0,speedGPS=0;
uint8_t hour=0,minute=0,seconds=0;

boolean GPSUpdate = false;
float xy_prev[2];
float xy[2];
float psi;
float psic = 0;
float delpsi = 0;
float angle_gps;
int auto_steer_final = 0;

float waypoint_x, waypoint_y;
//float u; //To save space we will compute u offline. 
//int lastUpdate; // a variable that will be updated when we get a new GPS update
int count = 0;
float lastTime = 0;
float arduinoTime;

//Declaring Variables
byte last_channel_steer, last_channel_thrust; 
unsigned long timer_steer, timer_thrust, current_time;

//Servos
PWMServo servoSteer;

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A')+10;
}

void setup() {
  // for Leonardos, if you want to debug SD issues, uncomment this line
  // to see serial output
  //while (!Serial);

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Ultimate GPSlogger Shield");
  pinMode(ledPin, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  waypoint_x = (waypoint_lat-ORIGINLAT)*GPSVAL;
  waypoint_y = (waypoint_lon-ORIGINLON)*GPSVAL*cos(ORIGINLAT*PI/180);
  // see if the card is present and can be initialized:
  //if (!SD.begin(chipSelect, 11, 12, 13)) {
//  if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
//    Serial.println("Card init. failed!");
//    while(true) {};
//  }
//  char filename[15];
//  strcpy(filename, "GPSLOG00.TXT");
//  for (uint8_t i = 0; i < 100; i++) {
//    filename[6] = '0' + i/10;
//    filename[7] = '0' + i%10;
//    // create if does not exist, do not open existing, write, sync after write
//    if (! SD.exists(filename)) {
//      break;
//    }
//  }
//
//  logfile = SD.open(filename, FILE_WRITE);
//  if( ! logfile ) {
//    Serial.print("Couldnt create "); 
//    Serial.println(filename);
//    while(true){};
//  }
//  Serial.print("Writing to "); 
//  Serial.println(filename);

  // connect to the GPS at the desired rate
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  
  useInterrupt(true); //what happens if we set this to false?

  Serial.println("Ready!");

  //Setup servos
  //thrust_servo.attach(9);
  servoSteer.attach(9);

  //Send steerMiddle to Servo
  servoSteer.write(map(steerMiddle,servo0,servo180,0,180));

   // put your setup code here, to run once:
   PCICR |= (1 << PCIE2); //PCIE2 is for pins 0-7
   PCMSK2 |= (1 << PCINT19); //This is for pin 3 
   PCMSK2 |= (1 << PCINT21); //This is for pin 5

   xy_prev[0] = 0;
   xy_prev[1] = 0;

   Serial.print("OK Waiting 5 Seconds ! \n");
   delay(5000);

   lastTime = millis();
   //lastUpdate = millis();
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  #ifdef UDR0
      if (GPSECHO)
        if (c) UDR0 = c;  
      // writing direct to UDR0 is much much faster than Serial.print 
      // but only one character can be written at a time. 
  #endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (GPSUpdate == true) { //this variable will change from 0 to 1 when we get new GPS coordinates.

       

       
      xy[0] = (latitudeDegrees-ORIGINLAT)*GPSVAL;
      xy[1] = (longitudeDegrees-ORIGINLON)*GPSVAL*cos(ORIGINLAT*PI/180);
 
      //Use the new value and the previous of xy to compute psi
      //psi = atan2(xy[1]-xy_prev[1],xy[0]-xy_prev[0]);
      if(psi < 0)
      {
        psi = 2*PI + psi;
      }
      //Use new and old value to get u - This has been commented out to save space. We can compute this offline
      //u = (sqrt(pow(xy[1],2) + pow(xy[0],2))-sqrt(pow(xy_prev[1],2) + pow(xy_prev[0],2)))/(millis()-lastUpdate);
      
      //Use a moving average filter to filter out GPS noise
      xy_prev[0] = xy[0];
      xy_prev[1] = xy[1];

      GPSUpdate = false;
      //lastUpdate = millis();
   }

  if (pulseRead_thrust < thrustBrake) {
    pulseRead_thrust = thrustBrake;
  }
  if (pulseRead_steer_new > steerRight) {
    pulseRead_steer_new = pulseRead_steer;
  }
  if (pulseRead_steer_new < steerLeft) { 
    pulseRead_steer_new = pulseRead_steer;  
  }
  pulseRead_steer = pulseRead_steer_new;
  pulseMap_thrust = map(pulseRead_thrust,servo0,servo180,0,180);
  pulseMap_steer = map(pulseRead_steer,servo0,servo180,0,180);
  
  if(autopilot == false){
     servoSteer.write(pulseMap_steer);
  }
  else{
    //Then we need to apply control based on our current xy coordinate and where we want to go
    //Now we need to get an angle command to direct us to our waypoint
//  //This routine takes care of the wrapping nature of atan2
    psic = atan2(waypoint_y-xy[1],waypoint_x-xy[0]);
    if(psic < 0)
    {
      psic = 2*PI + psic;
    }
    if(abs(psic - psi) > PI)
    {
      if(psic > psi)
      {
        psic = psic - 2*PI;
      }
      else
      {
        psi = psi - 2*PI;
      }
    }
    delpsi = psic - psi;
    //Now we can feed psic into the nose wheel command
    float auto_steer = 8*delpsi; //Need to change 0.1 to something else and tune this
    //auto_steer = map(steerRight,servo0,servo180,0,180);
    auto_steer_final = round(auto_steer) + 94;
    servoSteer.write(auto_steer_final);
  }
  
  if(pulseMap_thrust < 60){
        press_count = press_count + 1;
  }
  if(pulseMap_thrust >= 60){
      press_count = 0;
      PRESS = false;
  }
  if(press_count >= 3000 && PRESS == false){
    autopilot_count = autopilot_count + 1;
    press_count = 0;PRESS = true;
    
  }
  if(autopilot_count%2 == 0){
    autopilot = false;
  }
  else{
    autopilot = true;
  }
  
  arduinoTime = millis();

  if (millis() > 100 + lastTime) {
    Serial.print(latitudeDegrees,8); Serial.print(" "); 
    Serial.print(longitudeDegrees,8);Serial.print(" "); 
    Serial.print(hour);Serial.print(" "); 
    Serial.print(minute);Serial.print(" "); 
    Serial.print(seconds); Serial.print(" "); 
    Serial.print(speedGPS);Serial.print(" "); 
    //Serial.print(arduinoTime);Serial.print(" "); 
    Serial.print(xy[0]);    Serial.print(" ");
    Serial.print(xy[1]);    Serial.print(" ");
//    Serial.print(u);    Serial.print(" ");
    Serial.print(pulseRead_thrust);Serial.print(" "); 
    Serial.print(pulseRead_steer);Serial.print(" "); 
    Serial.print(pulseMap_thrust);Serial.print(" "); 
    Serial.print(auto_steer);    Serial.print(" ");
    Serial.print(pulseMap_steer);Serial.print(" ");
    Serial.print(waypoint_x);    Serial.print(" ");
    Serial.print(waypoint_y);    Serial.print(" ");
    Serial.print(auto_steer); Serial.print(" ");    
    Serial.print(auto_steer_final); Serial.print(" ");
    Serial.print(angle_gps);    Serial.print(" ");
    Serial.print(autopilot); Serial.print(" ");
    Serial.print(psic);    Serial.print(" ");
    Serial.print(delpsi);    Serial.print("\n");
    
  
//    logfile.print(latitudeDegrees); logfile.print(" "); 
//    logfile.print(longitudeDegrees);logfile.print(" "); 
//    logfile.print(hour);logfile.print(" "); 
//    logfile.print(minute);logfile.print(" "); 
//    logfile.print(seconds); logfile.print(" "); 
//    logfile.print(speedGPS);logfile.print(" "); 
//    logfile.print(arduinoTime);logfile.print(" "); 
//    logfile.print(psi);    logfile.print(" ");
//    logfile.print(xy[0]);    logfile.print(" ");
//    logfile.print(xy[1]);    logfile.print(" ");
////    logfile.print(u);    logfile.print(" ");
//    logfile.print(pulseRead_thrust);logfile.print(" "); 
//    logfile.print(pulseRead_steer);logfile.print(" "); 
//    logfile.print(pulseMap_thrust);logfile.print(" "); 
//    logfile.print(auto_steer);    logfile.print(" ");
//    logfile.print(pulseMap_steer);logfile.print(" ");
//    logfile.print(autopilot); logfile.print("\n"); 
   lastTime = millis();
  }
  

    if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    
    // Don't call lastNMEA more than once between parse calls!  Calling lastNMEA 
    // will clear the received flag and can cause very subtle race conditions if
    // new data comes in before parse is called again.
    char *stringptr = GPS.lastNMEA();
    
    if (!GPS.parse(stringptr))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another

    latitudeDegrees = GPS.latitudeDegrees;
    longitudeDegrees = GPS.longitudeDegrees;
    speedGPS = GPS.speed;
    hour = GPS.hour;
    minute = GPS.minute;
    seconds = GPS.seconds;
    GPSUpdate = true;
    psi = GPS.angle*PI/180;
    // Sentence parsed! 
    Serial.println("OK");
    if (LOG_FIXONLY && !GPS.fix) {
      Serial.print("No Fix");
      return;
    }

    // Rad. lets log it!
//    Serial.println("Log");
//
//    uint8_t stringsize = strlen(stringptr);
//    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))    //write the string to the SD file
//        while(true){};
//    if (strstr(stringptr, "RMC") || strstr(stringptr, "GGA"))   //logfile.flush();
//    Serial.println();
  }  
 
}

//This routine below is hijacked by the SoftwareSerial Library. In order to get this to work you need to use the
//FASTSerial library which essentially comments out those lines of code and puts it in here.

ISR(PCINT2_vect){ //This is for pins 3 and 5

  SoftwareSerial::handle_interrupt(); //This is taken from the SoftwareSerial library. Turns out the Software SerialLibrary hijacks these routines
  //So if you want to Use ISR() you need to comment out those lines of code and put this in here.
  
//ISR(PCINT0_vect){ //This is for pins 8 and 9
  current_time = micros();
  if(PIND & B00001000){ //This is pin 3
  //if (PINB & B00000001) { //This is pin 8
    if(last_channel_thrust == 0)
    {
      last_channel_thrust = 1;
      timer_thrust = current_time;
    }
  }
  else if(last_channel_thrust == 1){
      last_channel_thrust = 0;
      pulseRead_thrust = current_time - timer_thrust;
  }
  if(PIND & B00100000){ //This is pin 5
  //if (PINB & B00000010){  //This is pin 9
    if(last_channel_steer == 0)
    {
      last_channel_steer = 1;
      timer_steer = current_time;
    }
  }
  else if(last_channel_steer == 1){
      last_channel_steer = 0;
      pulseRead_steer_new = current_time - timer_steer;
  }  
}

///* End code */
