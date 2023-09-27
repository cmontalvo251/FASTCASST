#include <SPI.h>

//#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>

//If you want to use the ISR() functions you need to use the FASTGPS.h and FASTSerial.h libraries

#include <FASTGPS.h>
#include <FASTSerial.h>

#include <SD.h>
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

File logfile;

int pulsePin_thrust = 3, pulsePin_steer = 5;
int pulseMap_steer, pulseMap_thrust, pulseRead_steer, pulseRead_steer_new,pulseRead_thrust;

int servo0 = 540, servo180 = 2390, steerRight = 1975, steerLeft = 1010, steerMiddle = 1560;
int thrustForward = 1900, thrustRest = 1460, thrust_half = (thrustForward+thrustRest)/2;
int thrustBrake = 1100;

float latitudeDegrees=0,longitudeDegrees=0,speedGPS=0;
uint8_t hour=0,minute=0,seconds=0;

int arduinoTime;

float lastTime = 0;

//Declaring Variables
byte last_channel_steer, last_channel_thrust; 
unsigned long zero_timer, timer_steer, timer_thrust, current_time;

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

// blink out an error code
void error(uint8_t errno) {
  /*
  if (SD.errorCode()) {
   putstring("SD error: ");
   Serial.print(card.errorCode(), HEX);
   Serial.print(',');
   Serial.println(card.errorData(), HEX);
   }
   */
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

void setup() {
  // for Leonardos, if you want to debug SD issues, uncomment this line
  // to see serial output
  //while (!Serial);

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("\r\nUltimate GPSlogger Shield");
  pinMode(ledPin, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  //if (!SD.begin(chipSelect, 11, 12, 13)) {
  if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
    Serial.println("Card init. failed!");
    error(2);
  }
  char filename[15];
  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error(3);
  }
  Serial.print("Writing to "); 
  Serial.println(filename);

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

  int init_steer = map(steerMiddle,servo0,servo180,0,180);
  servoSteer.write(init_steer);

  pulseRead_steer = steerMiddle;
  pulseRead_thrust = thrustRest;

   // put your setup code here, to run once:
   PCICR |= (1 << PCIE2); //PCIE2 is for pins 0-7
   PCMSK2 |= (1 << PCINT19); //This is for pin 3 
   PCMSK2 |= (1 << PCINT21); //This is for pin 5

   Serial.print("OK Waiting 5 Seconds ! \n");
   delay(5000);

   zero_timer = micros();
   lastTime = millis();
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
  if (pulseRead_thrust < thrustRest) {
    pulseRead_thrust = thrustRest;
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
  
  servoSteer.write(pulseMap_steer);
  
  arduinoTime = millis();

    /* Serial.print("Latitude: "); */
    /* Serial.print("Longitude: "); */
    /* Serial.print("Hour: "); */
    /* Serial.print("Minute: "); */
    /* Serial.print("Second: "); */
    /* Serial.print("Speed (m/s): "); */
    /* Serial.print("Milliseconds: "); */
    /* Serial.print("PWM Thrust: "); */
    /* Serial.print("PWM Steer: "); */
    /* Serial.print("Thrust Servo Degrees: "); */
    /* Serial.println("Steer Servo Degrees: "); */
  
    /* Serial.print(latitudeDegrees); Serial.print(" ");  */
    /* Serial.print(longitudeDegrees);Serial.print(" ");  */
    /* Serial.print(hour);Serial.print(" ");  */
    /* Serial.print(minute);Serial.print(" ");  */
    /* Serial.print(seconds); Serial.print(" ");  */
    /* Serial.print(speedGPS);Serial.print(" ");  */
  if (millis() > 100 + lastTime) {

    Serial.print(arduinoTime);Serial.print(" "); 
    Serial.print(pulseRead_thrust);Serial.print(" "); 
    Serial.print(pulseRead_steer);Serial.print(" "); 
    Serial.print(pulseMap_thrust);Serial.print(" "); 
    Serial.print(pulseMap_steer);Serial.print("\n"); 

    logfile.print(latitudeDegrees); logfile.print(" "); 
    logfile.print(longitudeDegrees);logfile.print(" "); 
    logfile.print(hour);logfile.print(" "); 
    logfile.print(minute);logfile.print(" "); 
    logfile.print(seconds); logfile.print(" "); 
    logfile.print(speedGPS);logfile.print(" "); 
    logfile.print(arduinoTime);logfile.print(" "); 
    logfile.print(pulseRead_thrust);logfile.print(" "); 
    logfile.print(pulseRead_steer);logfile.print(" "); 
    logfile.print(pulseMap_thrust);logfile.print(" "); 
    logfile.print(pulseMap_steer);logfile.print("\n"); 
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

    // Sentence parsed! 
    Serial.println("OK");
    if (LOG_FIXONLY && !GPS.fix) {
      Serial.print("No Fix");
      return;
    }

    // Rad. lets log it!
    Serial.println("Log");

    uint8_t stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))    //write the string to the SD file
        error(4);
    if (strstr(stringptr, "RMC") || strstr(stringptr, "GGA"))   logfile.flush();
    Serial.println();
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

