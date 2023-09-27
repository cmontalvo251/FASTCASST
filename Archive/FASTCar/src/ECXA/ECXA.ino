// EXCA SENSOR CODE 
 
// SD Card
// MTK GPS module using the Adafruit GPS module
// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
 
//hook wires directly to the shields TX and RX solder pads
//TX on GPS shield to RX1 (pin 19)
//RX on GPS shield to TX1 (pin 18)
//5v on shield to 5v
//GND on shield to GND
 
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
 
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada
 
#include <FASTGPS.h>
 
// GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
 
//   Hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3
 
// If using hardware serial (e.g. Arduino Mega) enable this line instead
// (you can change the Serial number to match your wiring):
// HardwareSerial mySerial = Serial1; //Turns out this doesn't work and
// Adafruit_GPS GPS(&mySerial); //Adafruit must have never tested it.
 
Adafruit_GPS GPS(&Serial1);
 
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
 
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
 
float lastPrint = 0;
int WPcounter = 0;
float kint = 0;
float umeasured;
float x=0,y=0,psic,dpsi,xcommand,ycommand;

//Compute which waypoint we need to travel to
float WPx[4],WPy[4]; 
 
//SD Card Stuff
#include <SD.h>
File myFile;

/// Servo Stuff
#include <Servo.h>

Servo myservo, mythrottle;

#define THROTTLE_MIN 88
#define THROTTLE_MAX 140
//Intramural Fields
//#define Ox 30.695691
//#define Oy -88.192942
//Municipal Park
#define Ox 30.706606
#define Oy -88.159402
#define OFFSET 20.0 //FUCKING DOG WALKER HAD TO PARK THEIR CAR IN THE MIDDLE OF THE FUCKING PARKING LOT
#define NM2FT 6076.115485560000
#define FT2M 0.3048
#define GPS2CART 111119.999999921
#define TRACK_DISTANCE 30
#define STEER_STRAIGHT 103 //Might need to fix this everytime
#define STEER_LEFT 50
#define STEER_RIGHT 142
 
unsigned long current_time,timer1, timer2, timer3;
byte last_channel=0, last_channel2=0, last_channel3=0;
int receiver_pulse_servo = 1500;
int receiver_pulse_throttle = 1500; 
int trainer_switch = 1500;
float servoA,deltaT;

void setup() 
{

  // put your setup code here, to run once:

  //Setup waypoints
  WPx[0] = TRACK_DISTANCE;    WPy[0] = 0.0+OFFSET; 
  WPx[1] = 0.0;               WPy[1] = TRACK_DISTANCE+OFFSET;
  WPx[2] = TRACK_DISTANCE;    WPy[2] = TRACK_DISTANCE+OFFSET;
  WPx[3] = 0.0;               WPy[3] = 0.0+OFFSET;
  //In this context the waypoints are
  // (x,y) - Assuming TRACK_DISTANCE = 30 and OFFSET = 20
  // (30,20)
  // (0,50)
  // (30,50)
  // (0,20)
  // The reason for the spiral pattern is to make it easy for the 
  // pilot to see if the autopilot is working while the car is driving in
  // autopilot mode

   ///Enable interrupts for PORTB
  PCICR |= (1 << PCIE0); //PORTB 0-7  //PortB Handles pins 10-13 so if you want to read arduino signals you need to choose 10-13
  
  ///Run an interrupt function when pin 10 and 11 changes
  //PCMSK0 |= (1 << PCINT4 | 1 << PCINT5); //PORTB4 - Digital Pin 10 & 11

  //Before I added the auto pilot switch we have servos on 9(throttle),12(servo) and receivers on 10 and 11. But not I want servos on 9,10 and receivers on 11,12,13
  PCMSK0 |= (1 << PCINT5 | 1 << PCINT6 | 1 << PCINT7); //So this is PORTB - Digital pins 11,12,13 or PORTB5,6,7

  //Ok for reading receiver signals you'be got to read the servo signal which I'm putting in pin 11. You can double check in the 
  //interuppt sequence at the bottom of this script. So take the aileron signal from the receiver and throw it in pin 11. It is currently
  //the brown wire. The yellow wire is coming from the throttle signal which needs to be read using pin 12. Finally take GEAR and put it
  //in pin 13. Again you can double check all of this using the interuppt sequence at the bottom of this script.
  
  //Attach Servos
  //myservo.attach(12); 
  //mythrottle.attach(9); 
  
  //Since PORTB handles 10-13 we don't want the servos attached to anything between 10-13 
  //In this case we have two servos so we will attach two servos to 9 and 10
  myservo.attach(10); //If you're looking at the ECXA - the white wire going underneath the solderable breadboard is the throttle
  mythrottle.attach(9); //The blue wire is for the servo. So right now I have the white wire(throttle) in pin 9 and the blue wire(servo)
  // in pin 10.
 
  //Begin Serial
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);

  // also spit it out
  Serial.println("ECXA Sensor");
 
  //Initialize the SD Card
  pinMode(53, OUTPUT);
  if (!SD.begin(53)) {
    Serial.println("SD Card initialization failed!");
    while(1){};
  }
  Serial.println("initialization done.");
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another
  char filename[15];
  strcpy(filename, "ECXA000.TXT");
  for (uint8_t i = 0; i < 1000; i++) {
    filename[4] = '0' + i/100;
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  char lastfile[15];
  strcpy(lastfile,"EXCXA999.TXT");
  if (SD.exists(lastfile)) {
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
 
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
 
  Serial.print("GPS Class Initialized \n");
 
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
 
  Serial.print("NMEA OUTPUT Set \n");
 
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 or 10 Hz update rate
  // For the parsing code to work nicely and have time to sort through the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  Serial.print("Update Rate Set \n");

  //Request updates on antenna status
  //GPS.sendCommand(PGCMD_ANTENNA);
  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(false);
 
  Serial.print("Delaying for 1 second to ensure all sensors are onboard. \n");

  delay(1000);
  // Ask for firmware version
  // mySerial.println(PMTK_Q_RELEASE);
 
  lastPrint = millis()/1000.0;

} 
 
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  //if (GPSECHO) - Currently GPSECHO is set to false so I just commented this out. I'm not really sure if this block
  //of code is even running since I have useInterrupt set to false
    //if (c) UDR0 = c; // writing direct to UDR0 is much much faster than Serial.print
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

void loop(){                     // run over and over again
 
  // put your main code here, to run repeatedly:
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(

  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    //if (GPSECHO) - Currently GPSECHO is set to false so I'll just comment this out
      //if (c) Serial.print(c);
      //if (c) UDR0 = c; //writing direct to UDR0 is much faster than Serial.print
      //but only one character can be written at a time.
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    // Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  //Check and see if autopilot switch is on or off
  //The Dx5 will default Ch5 to the Trainer Switch and it will call it GEAR.
  //Right now the receiver for GEAR is plugged into pin 13. Ch5 on a Dx5 is a 2-pin switch.
  //Unfortunately if you use a Dx6, GEAR will default to Switch A which is a 2-pin switch.
  //So you need to go into System_Setup->Channel_Assign->Next and set GEAR to switch D which
  //is the default trainer switch on a Dx6
  //Unfortunately using the pulseIn command mucks with the interrupts at the end of this script so this has been commented out.
  
  //int trainer_switch = pulseIn(13,HIGH); 
  //int trainer_switch = 2000;
  
  //Set default to 0 in case something freaks out
  int autopilot = 0;

  //If you're using the Dx5 it means you have pin 13 plugged into GEAR on the receiver which the receiver defaults to the
  //trainer switch. When CH5 (TRAINER) on the transmitter is set to 0 the trainer_switch PWM signal is about 1912. When set to
  //1 on the transmitter the trainer_switch variable is about 1112 so this is pretty simple logic.

  if (trainer_switch > 1500) {
    autopilot = 0;
  }
  else {
    autopilot = 1;
  }

  
  //This is legacy code when I was using my Dx6 and a 3-pin switch.
  //if (trainer_switch > 1800) { //1896
  //  autopilot = 0;
  //}
  //else if (trainer_switch > 1400) { //1468
  //  autopilot = 1;
  //}
  //else {
  //  autopilot = 2;  //1077
  //}

  //This is legacy code when the trainer switch wasn't using the interuppts properly
  //if (deltaT < 70) {
  //  autopilot = 1;
  //}

  //Convert GPS.speed to umeasured - Do this all the time so we have this in the Serial.print for debugging
  umeasured = GPS.speed*0.51444;

  //Convert Lat,Lon to X,Y
  x = (GPS.latitudeDegrees - Ox)*GPS2CART; //North direction - Xf , meters
  y = (GPS.longitudeDegrees - Oy)*GPS2CART*cos(Ox*PI/180.0);

  //Write to each servo
  if (autopilot == 0) {
    //Manaul Control
    
    //Reset integral value to make sure we don't get insane integrator windup
    kint = 0;
    //Convert Recevier Signals to degrees
    servoA = map(receiver_pulse_servo,528,2475,0,180);
    deltaT = map(receiver_pulse_throttle,528,2475,0,180);
  }
  else if (autopilot == 1) {
    //Full Autonomous Control
    
    //Speed Controller
    
    float err = 3.5 - umeasured; //So our command is 2.0 right now might need to change this
    kint += 0.01*err;
    //Kp is 10.0 right now. Might need to lower this.
    deltaT = 10.0*err + 0.1*kint + THROTTLE_MIN; // So after tuning the throttle controller I realized that high proportional 
    //gain is bad so I'm going to add some integral gain and lower kp.
    //The interesting thing about this controller is the fact that if err is negative deltaT is THROTTLE_MIN which is techinically zero due to the saturation block below
    //deltaT = THROTTLE_MIN; //DEBUG MODE.
    
    //Saturation Block
    if (deltaT > THROTTLE_MAX) {
      deltaT = THROTTLE_MAX;
    }
    else if (deltaT < THROTTLE_MIN) {
      deltaT = THROTTLE_MIN;
    }
        
    //Waypoint Control
    xcommand = WPx[WPcounter];
    ycommand = WPy[WPcounter];
    if (sqrt(pow(xcommand-x,2) + pow(ycommand-y,2)) < 8) {
      WPcounter++;
      if (WPcounter > 3) {
        WPcounter = 0;
      }
    }

    //Compute Desired Heading based on x_c,y_c
    float psi = GPS.angle*PI/180.0; //convert GPS.angle to radians
    //psic = atan2(ycommand-y,xcommand-x);

    //For debugging we can map the aileron command from the transmitter to 0 - 2*pi using the map function
    psic = 0.0 + (2.0*PI)/(2068.0-1072.0)*(float(receiver_pulse_servo)-1072.0);
    
    float spsi = sin(psi);
    float cpsi = cos(psi);
    float spsic = sin(psic);
    float cpsic = cos(psic);
    dpsi = atan2(spsi*cpsic-cpsi*spsic,cpsi*cpsic+spsi*spsic); //this entire dpsi crap is to make sure that our dpsi is between -180 and 180
    ///The gain is negative and initially set to a low value. Increase this number 
    //slowly.
    servoA = -8.0*dpsi + STEER_STRAIGHT; //We can only do proportional control because we have no way
    //of measuring the yaw rate r unless we install a 9DOF sensor.
    
    //Saturation Block
    if (servoA > 136) {
      servoA = 136;
    }
    else if (servoA < 44) {
      servoA = 44;
    }

    //servoA = STEER_STRAIGHT;
    //deltaT = 72;
  }

  //Send Signals to actual hardware
 
  myservo.write(servoA);
  mythrottle.write(deltaT);

  //Output Data

  if (millis()/1000.0 > 0.2 + lastPrint) {

     lastPrint = millis()/1000.0;
     
     //SERIAL PRINTS
     Serial.print(GPS.year); Serial.print(" ");
     Serial.print(GPS.month); Serial.print(" ");
     Serial.print(GPS.day); Serial.print(" ");
     Serial.print(GPS.hour, DEC); Serial.print(' ');
     Serial.print(GPS.minute, DEC); Serial.print(' ');
     Serial.print(GPS.seconds, DEC); Serial.print(' ');
     Serial.print(lastPrint); Serial.print(" ");
     Serial.print((int)GPS.fix); Serial.print("  ");   
     //GPS Sensor & Servos
     Serial.print(GPS.latitudeDegrees, 8); Serial.print(" ");
     Serial.print(GPS.longitudeDegrees, 8); Serial.print(" ");
     Serial.print(trainer_switch); Serial.print(" ");
     Serial.print(autopilot); Serial.print(" ");
     Serial.print(umeasured); Serial.print(" ");
     Serial.print(receiver_pulse_throttle); Serial.print(" ");
     Serial.print(deltaT); Serial.print(" ");
     Serial.print(x); Serial.print(" ");
     Serial.print(y); Serial.print(" ");
     Serial.print(xcommand); Serial.print(" ");
     Serial.print(ycommand); Serial.print(" ");
     Serial.print(GPS.angle); Serial.print(" ");
     Serial.print(psic*180.0/PI); Serial.print(" ");
     Serial.print(dpsi*180.0/PI); Serial.print(" ");
     Serial.print(receiver_pulse_servo); Serial.print(" ");
     Serial.print(servoA); Serial.print(" ");
     Serial.println(" "); //This creates a newline

     //SD CARD LOG PRINTS
     myFile.print(GPS.year); myFile.print(" ");
     myFile.print(GPS.month); myFile.print(" ");
     myFile.print(GPS.day); myFile.print(" ");
     myFile.print(GPS.hour, DEC); myFile.print(' ');
     myFile.print(GPS.minute, DEC); myFile.print(' ');
     myFile.print(GPS.seconds, DEC); myFile.print(' ');
     myFile.print(lastPrint); myFile.print(" ");
     myFile.print((int)GPS.fix); myFile.print("  ");   
     //GPS Sensor & Servos
     myFile.print(GPS.latitudeDegrees, 8); myFile.print(" ");
     myFile.print(GPS.longitudeDegrees, 8); myFile.print(" ");
     myFile.print(trainer_switch); myFile.print(" ");
     myFile.print(autopilot); myFile.print(" ");
     myFile.print(umeasured); myFile.print(" ");
     myFile.print(receiver_pulse_throttle); myFile.print(" ");
     myFile.print(deltaT); myFile.print(" ");
     myFile.print(x); myFile.print(" ");
     myFile.print(y); myFile.print(" ");
     myFile.print(xcommand); myFile.print(" ");
     myFile.print(ycommand); myFile.print(" ");
     myFile.print(GPS.angle); myFile.print(" ");
     myFile.print(psic*180.0/PI); myFile.print(" ");
     myFile.print(dpsi*180.0/PI); myFile.print(" ");
     myFile.print(receiver_pulse_servo); myFile.print(" ");
     myFile.print(servoA); myFile.print(" ");
     myFile.println(" "); //This creates a newline
   
     //Flush the file 
     myFile.flush();

  }

}

//// This Part causes an error when compiling/uploading ////
//pin change interrupt for PORTB
//Before I added the auto pilot switch we have servos on 9(throttle),12(servo) and receivers on 10 and 11. 
//But now I want servos on 9(throttle),10(servo) and receivers on 11,12,13
//PCMSK0 |= (1 << PCINT5 | 1 << PCINT6 | 1 << PCINT7); //So this is PORTB - Digital pins 11,12,13 or PORTB5,6,7
ISR(PCINT0_vect) {
 
  current_time = micros();

  //This is the block for the servo which is on pin 11
  if (PINB & B00100000) { // PORTB5 is Digital Pin 11 MEGA
    if(last_channel == 0)
      {
	      last_channel = 1;
	      timer1 = current_time;
      }
  }
  else if(last_channel == 1){
    last_channel = 0;
    receiver_pulse_servo = current_time - timer1;
  }

  //This block is for the throttle which is on pin 12
  if (PINB & B01000000) { // PORTB6 is Digital Pin 12 MEGA
    if(last_channel2 == 0)
      {
	      last_channel2 = 1;
	      timer2 = current_time;
      }
  }
  else if(last_channel2 == 1){
    last_channel2 = 0;
    receiver_pulse_throttle = current_time - timer2;
  }

  //Now we need one more block for pin 13 which will be the trainer switch. Pray the MEGA can handle this much insanity. 
  if (PINB & B10000000) { //PORTB7 is Digital Pin 13 MEGA
    if (last_channel3 == 0)
    {
      last_channel3 = 1;
      timer3 = current_time;
    }
  }
  else if (last_channel3 == 1){
      last_channel3 = 0;
      trainer_switch = current_time - timer3;
  }  
  
}
