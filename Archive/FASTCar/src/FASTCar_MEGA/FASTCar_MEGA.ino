//If you want to use the ISR() functions you need to use the FASTGPS.h and FASTSerial.h libraries
#include <FASTSerial.h>
#include <FASTGPS.h>
#include <SD.h>
#include <avr/sleep.h>

//Get PWMServo Library - Make sure to get version 2.
#include <PWMServo.h>

//SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&Serial1);
#define GPSECHO  false
#define LOG_FIXONLY false  
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// Set the pins used
#define chipSelect 53
#define ledPin 13
#define pulsePin_steer 5

#define servo0 540
#define servo180 2390

#define steerLeft 1010
#define steerMiddle 1560
#define steerRight 1975
File logfile;

int pulseMap_steer = steerMiddle,pulseRead_steer = steerMiddle, pulseRead_steer_new;

boolean autopilot = false;
int auto_steer_final = steerLeft;
float lastTime = 0;
//Declaring Variables
byte last_channel_steer;
unsigned long timer_steer, current_time;

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
  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
    Serial.println("Card init. failed!");
    while(true) {};
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
    while(true){};
  }
  Serial.print("Writing to "); 
  Serial.println(filename);

  // connect to the GPS at the desired rate
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  
  useInterrupt(false); //what happens if we set this to false?

  Serial.println("Ready!");

  //Setup servos
  servoSteer.attach(12);

  //Send steerMiddle to Servo
  servoSteer.write(map(steerMiddle,servo0,servo180,0,180));

   // put your setup code here, to run once:
   PCICR |= (1 << PCIE0); //PCIE0 is for PORTB 0-7 which includes pin 10-13
   PCMSK0 |= (1 << PCINT4); //This is for pin 10

   Serial.print("OK Waiting 1 Seconds ! \n");
   delay(1000);

   lastTime = millis()/1000.0;
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
  if (pulseRead_steer_new > steerRight) {
    pulseRead_steer_new = pulseRead_steer;
  }
  if (pulseRead_steer_new < steerLeft) { 
    pulseRead_steer_new = pulseRead_steer;  
  }
  pulseRead_steer = pulseRead_steer_new;
  pulseMap_steer = map(pulseRead_steer,servo0,servo180,0,180);
  
  if(autopilot == false){
     servoSteer.write(pulseMap_steer);
  }
  else{
    if (auto_steer_final > steerRight) {
      auto_steer_final = steerLeft;
    }
    servoSteer.write(map(auto_steer_final,servo0,servo180,0,180));
  }

  if (millis()/1000.0 > 0.1 + lastTime) {
    auto_steer_final = auto_steer_final+10;
    lastTime = millis()/1000.0;
    //SERIAL PRINTS
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print(':');
    Serial.print(lastTime); Serial.print(" ");
    Serial.print(autopilot); Serial.print(" ");
    Serial.print(pulseMap_steer); Serial.print(" ");
    Serial.print(pulseRead_steer); Serial.print(" ");
    Serial.print(pulseRead_steer_new); Serial.print(" ");
    Serial.print("\n");
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
  }  
 
}

//This routine below is hijacked by the SoftwareSerial Library. In order to get this to work you need to use the
//FASTSerial library which essentially comments out those lines of code and puts it in here.
//Remember if you change PCINT0_vect to PCINT2_vect you need
//to change FASTSerial.cpp to only have 1 blocked out line of code.
ISR(PCINT0_vect){ //This is for pins 3 and 5

  //SoftwareSerial::handle_interrupt(); //This is taken from the SoftwareSerial library. Turns out the Software SerialLibrary hijacks these routines
  //So if you want to Use ISR() you need to comment out those lines of code and put this in here.
  
  current_time = micros();
  if (PINB & B00010000) { //PORTB6 is Digital Pin 10 MEGA
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
