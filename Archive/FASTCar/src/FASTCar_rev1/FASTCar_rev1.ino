#include <SPI.h>

//#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>

//If you want to use the ISR() functions you need to use the FASTGPS.h and FASTSerial.h libraries

#include <FASTGPS.h>
#include <FASTSerial.h>

#include <SD.h>
#include <avr/sleep.h>

//Get PWMServo Library - Make sure to get version 2.
//#include <PWMServo.h>
#include <Servo.h>

//Declaring Variables
byte last_channel_steer, last_channel_thrust, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, start;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_steer, timer_thrust, timer_3, timer_4, current_time;
int pulse_time_steer, pulse_time_thrust;

//Servos

Servo thrust_servo;
Servo steer_servo;

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
  
  //useInterrupt(false); //what happens if we set this to false?

  Serial.println("Ready!");

  //Setup servos
  thrust_servo.attach(9);
  steer_servo.attach(6);

   // put your setup code here, to run once:
   PCICR |= (1 << PCIE2); //PCIE2 is for pins 0-7
   PCMSK2 |= (1 << PCINT19); //This is for pin 3 
   PCMSK2 |= (1 << PCINT21); //This is for pin 5
   zero_timer = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //Looks like the pulses are working just fine. Let's convert them to degrees
  float thrust_deg = map(pulse_time_thrust,540,2400,0,180);
  float steer_deg = map(pulse_time_steer,540,2400,0,180);

  //Ok now let's load in the old PWMServo Library and send the signals.
  steer_servo.write(steer_deg);
  thrust_servo.write(thrust_deg);
  
  Serial.print(pulse_time_thrust);
  Serial.print(" ");
  Serial.print(thrust_deg);
  Serial.print(" ");
  Serial.print(pulse_time_steer);
  Serial.print(" ");
  Serial.print(steer_deg);
  Serial.print("\n");  
  
 
}

//This routine below is hijacked by the SoftwareSerial Library. In order to get this to work you need to use the
//FASTSerial library which essentially comments out those lines of code and puts it in here.

ISR(PCINT2_vect){ //This is for pins 3 and 5

  SoftwareSerial::handle_interrupt(); //This is taken from the SoftwareSerial library. Turns out the Software SerialLibrary hijacks these routines
  //So if you want to Use ISR() you need to comment out those lines of code and put this in here.
  
//ISR(PCINT0_vect){ //This is for pins 8 and 9
  current_time = micros();
  if(PIND & B00100000){ //This is pin 5
  //if (PINB & B00000001) { //This is pin 8
    if(last_channel_thrust == 0)
    {
      last_channel_thrust = 1;
      timer_thrust = current_time;
    }
  }
  else if(last_channel_thrust == 1){
      last_channel_thrust = 0;
      pulse_time_thrust = current_time - timer_thrust;
  }
  if(PIND & B00001000){ //This is pin 3
  //if (PINB & B00000010){  //This is pin 9
    if(last_channel_steer == 0)
    {
      last_channel_steer = 1;
      timer_steer = current_time;
    }
  }
  else if(last_channel_steer == 1){
      last_channel_steer = 0;
      pulse_time_steer = current_time - timer_steer;
    
  }  
}

///* End code */

