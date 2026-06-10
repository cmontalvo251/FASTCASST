// FASTSensor sensor package includes GPS, 10DOF, Accelerometer, SD card, and reads servo signals.

//IMU//
#include <Adafruit_10DOF.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

float temperature;
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

//Arduino Time
float lastPrint = 0;

//GPS//
#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Serial3);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//Servo//
//#include <Servo.h>
//Servo AILE,ELEV,THRO; //aileron, elevator, throttle servos.. just reading for now
//unsigned long current_time,timerA,timerE;
//byte last_channelA,last_channelE;
//int pulse_timeA,pulse_timeE;
  
//SD//
#include <SPI.h> // apparently I need this too
#include <SD.h>
File myFile;

// Accelerometer
int xpin = 2;
int ypin = 1;
int zpin = 0;

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
    filename[5] = '0' + i/10;
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
  // I just need to read for now
  // and the writting will need autonoumus flight
  //AILE.attach(8);
  //ELEV.attach(9);
  //THRO.attach(10);
  
  // Reading from pins 11 12. Dont need to read throttle because it will be constant (max)
//  PCICR |= (1 << PCIE0); //PORTB 0-7 
//  PCMSK0 |= (1 << PCINT5); //PORTB5 - Digital Pin 11
//  PCMSK0 |= (1 << PCINT6); //PORTB6 - Digital Pin 12
  
  // 10DOF//
  Serial.begin(115200);
  Serial.println(F("Adafruit 10DOF Tester")); Serial.println("");
  
  /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
//  displaySensorDetails();
 
 
  // GPS //
  GPS.begin(9600);
  
  Serial.print("GPS Class Initialized \n");
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //turn on RMC(recommended minimum)
  Serial.print("NMEA OUTPUT Set \n");
  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 or 10 Hz update rate. what should i use??
  
  Serial.print("Update Rate Set \n");
  
  useInterrupt(false);
  
  Serial.print("Delaying for  1 second \n");
  delay(1000);
  
  lastPrint = millis()/1000.0;
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

  
  // PITOT //
//  float pitotValue = analogRead(14); //Pitot in analog 14
//  float x_accel = analogRead(xpin);
//  float y_accel = analogRead(ypin);
//  float z_accel = analogRead(zpin);
//
    if (timer > millis()){ 
        timer = millis();
      }
  //printing if clause
    if (millis() - timer  > 1000) {
    
      timer = millis(); // reset timer

    
//  
//    SERIAL PRINTS
//    GPS Sensor
      Serial.print("\nTime: ");
      Serial.print(GPS.hour, DEC); Serial.print(':');
      Serial.print(GPS.minute, DEC); Serial.print(':');
      Serial.print(GPS.seconds, DEC); Serial.print(':');
      Serial.println(GPS.milliseconds);;

      Serial.print("Fix: "); Serial.println((int)GPS.fix);
        if (GPS.fix) {
        
      
         Serial.print(GPS.latitudeDegrees, 8); Serial.print(" ");
         Serial.println(GPS.longitudeDegrees, 8); Serial.print(" "); 
    
         Serial.print("Speed (knots): ");Serial.println(GPS.speed); Serial.print(" ");
    
         Serial.print("Angle: "); Serial.println(GPS.angle); Serial.print(" ");
    
         Serial.print("Altitude: "); Serial.println(GPS.altitude);
        }
      }
}
//
//        //Accelerometer
//      Serial.print(x_accel); Serial.print(" ");
//      Serial.print(y_accel); Serial.print(" ");
//      Serial.print(z_accel); Serial.print("\n");
//
//    //10DOF Data
//  /* Get a new sensor event */
//      sensors_event_t event;
//   
//  /* Display the results (acceleration is measured in m/s^2) */
//      accel.getEvent(&event);
//      Serial.print(F("ACCEL "));
//      Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
//      Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
//      Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
//
//  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
//      mag.getEvent(&event);
//      Serial.print(F("MAG   "));
//      Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
//      Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
//      Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
//
//  /* Display the results (gyrocope values in rad/s) */
//      gyro.getEvent(&event);
//      Serial.print(F("GYRO  "));
//      Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
//      Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
//      Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s ");  
//
//  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
//      bmp.getEvent(&event);
//        if (event.pressure)
//        {
//         /* Display atmospheric pressure in hPa */
//         Serial.print(F("PRESS "));
//         Serial.print(event.pressure);
//         Serial.print(F(" hPa, "));
//         /* Display ambient temperature in C */
//         bmp.getTemperature(&temperature);
//         Serial.print(temperature);
//         Serial.print(F(" C, "));
//         /* Then convert the atmospheric pressure, SLP and temp to altitude    */
//         /* Update this next line with the current SLP for better results      */
//         Serial.print(bmp.pressureToAltitude(seaLevelPressure,
//                                        event.pressure,
//                                        temperature)); 
//         Serial.println(F(" m"));
//        }
//  
//      Serial.println(F(""));
//   
//
//   
//    //Servos
////    Serial.print(pulse_timeA); Serial.print(" ");
////    Serial.print(pulse_timeE); Serial.print("\n");
//  
//  
//   /* Write data to file */
//   //GPS Sensor
//      myFile.print(GPS.hour, DEC); myFile.print(' ');
//      myFile.print(GPS.minute, DEC); myFile.print(' ');
//      myFile.print(GPS.seconds, DEC); myFile.print(' ');
//      myFile.print(lastPrint); myFile.print(" ");
//      myFile.print((int)GPS.fix); myFile.print(" ");
//      myFile.print(GPS.latitudeDegrees, 8); myFile.print(" ");
//      myFile.print(GPS.longitudeDegrees, 8); myFile.print(" "); 
//      myFile.print(GPS.speed); myFile.print(" ");
//      myFile.print(GPS.angle); myFile.print(" ");
//      myFile.print(GPS.altitude); myFile.println(" ");
//
//      //Accel
//      myFile.print(x_accel); myFile.print(" ");
//      myFile.print(y_accel); myFile.print(" ");
//      myFile.print(z_accel); myFile.println(" ");
//  
//   //10 DOF Data
//
//
//      mag.getEvent(&event);
//      myFile.print(event.magnetic.x); myFile.print(" ");
//      myFile.print(event.magnetic.y); myFile.print(" ");
//      myFile.print(event.magnetic.z); myFile.println(" ");
//
//
//      gyro.getEvent(&event);
//      myFile.print(event.gyro.x); myFile.print(" ");
//      myFile.print(event.gyro.y); myFile.print(" ");
//      myFile.print(event.gyro.z); myFile.println(" ");
//
//
//      accel.getEvent(&event);
//      myFile.print(event.acceleration.x); myFile.print(" ");
//      myFile.print(event.acceleration.y); myFile.print(" ");
//      myFile.print(event.acceleration.z); myFile.println(" ");
//
//
//      bmp.getEvent(&event);
//      myFile.print(temperature); myFile.print(" ");
//      myFile.print(event.pressure); myFile.print(" ");
//      myFile.print(bmp.pressureToAltitude(seaLevelPressure,
//                                        event.pressure,
//                                        temperature)); myFile.println(" ");
//   
//
//   
////   //Servos
////    myFile.print(pulse_timeA); myFile.print(" ");
////    myFile.print(pulse_timeE); myFile.print("\n");
//   
//    //Flush the file
//      myFile.flush();
//  }
//}


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
