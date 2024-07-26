
//Datalogger 9DOF Code


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MPU9250.h>
#include <Adafruit_10DOF.h>
#include <SD.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <SPI.h>
#include <math.h>


unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 1000;

Adafruit_BNO055 bno = Adafruit_BNO055(55); 
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_10DOF                dof   = Adafruit_10DOF();
File myFile;
char filename[15];

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");


if(!bno.begin())
{
  Serial.print("Ooops, no BNO055 detected . . . Check your wiring or I2C ADDR!");
  while(1);
}

//delay(1000);

//SD Card
  Serial.begin(9600);
  Serial.print("Initializing SD Card...");
  pinMode(53, OUTPUT);
  if (!SD.begin(53)) {
    Serial.println("initialization Failed!");
    Serial.println("");
    return;
  }

  Serial.println("initialization done.");
  Serial.println("");
  if (!accel.begin()) {
    Serial.println("10 DOF initialization failed!");
    Serial.println("");
    return;
  }

  SD.begin(53);
  Serial.print("Datalogger Initialization Done.");
  Serial.println("");
  strcpy(filename, "SData_00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    if (!SD.exists(filename)) {
      Serial.println(filename);
      Serial.println("");
      break;
    }
  }

  myFile = SD.open(filename, FILE_WRITE);
  if(myFile) {
      Serial.println("Writing to file");
      myFile.close();
    } else {
      Serial.println("Error opening file");
      myFile.close();
    }
  Serial.println("");
  Serial.println("Setup Complete");
  Serial.println("");
  delay(1000);
  

bno.setExtCrystalUse(true);
}

void loop(void) 
{

currentMillis = millis();

  sensors_event_t event;
  bno.getEvent(&event);

  
  sensors_vec_t   orientation;
  accel.getEvent(&event);
  
  /*
  Serial.print("X9: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY9: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ9: ");
  Serial.print(event.orientation.z, 4);

 
  myFile.print("X9: ");
  myFile.print(event.orientation.x, 4);
  myFile.print("\tY9: ");
  myFile.print(event.orientation.y, 4);
  myFile.print("\tZ9: ");
  myFile.print(event.orientation.z, 4);

  */

  myFile = SD.open(filename, FILE_WRITE);
  
  //Time
  
  Serial.print(millis());

  myFile.print(millis());
  
  //9DOF
  
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  Serial.print("");
  Serial.print(accel.x());
  Serial.print("");
  Serial.print(accel.y());
  Serial.print("");
  Serial.print(accel.z());
  //Serial.println("");


  
  myFile.print("");
  myFile.print(accel.x());
  myFile.print("");
  myFile.print(accel.y());
  myFile.print("");
  myFile.print(accel.z());
  //myFile.println("");


  //Serial.println("Test1");
  //myFile.println("Test1");
  //myFile.close();
  
  //delay(100);

  

  //10DOF
  
  
  if (dof.accelGetOrientation(&event, &orientation))
  {
    /*
    Serial.print("\tX10: ");
    Serial.print(orientation.x, 4);
    Serial.print("\tY10: ");
    Serial.print(orientation.y, 4);
    Serial.print("\tZ10: ");
    Serial.print(orientation.z, 8);


    
    myFile.print("\tX10: "); 
    myFile.print(orientation.x, 4); 
    myFile.print("\tY10: "); 
    myFile.print(orientation.y, 4); 
    myFile.print("\tZ10: "); 
    myFile.print(orientation.z, 8);  
     
    */

    Serial.print(""); 
    Serial.print(event.acceleration.x, 4); 
    Serial.print(""); 
    Serial.print(event.acceleration.y, 4); 
    Serial.print(""); 
    Serial.print(event.acceleration.z, 4); 
    Serial.println("");
    
    
    myFile.print(""); 
    myFile.print(event.acceleration.x, 4); 
    myFile.print(""); 
    myFile.print(event.acceleration.y, 4); 
    myFile.print(""); 
    myFile.print(event.acceleration.z, 4);  
    myFile.println("");
    myFile.close();
  }
    delay(100);

}
