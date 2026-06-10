// Arduino Library for Microchip MCP9808 Temperature Sensor.
// Example sketch.
// Prints the initial register values, then changes the limit registers,
// then prints the register values every two seconds.
//
// Jack Christensen Jun 2015
// Tested with Arduino 1.8.5 and an Arduino Uno.

#include <MCP9808.h>      // http://github.com/JChristensen/MCP9808
#include <Streaming.h>    // http://arduiniana.org/libraries/streaming/

MCP9808 mySensor;

void setup()
{
    Serial.begin(115200);
    Serial << F( "\n" __FILE__ " " __DATE__ " " __TIME__ "\n" );

    delay(1000);                        // time for the sensor to perform an initial temperature conversion
    uint8_t status = mySensor.begin();  // initialize the hardware
    if ( status != 0 )
    {
        Serial << "Error reading sensor, status=" << status << endl;
        Serial.flush();
        while (1);                      // loop until reset
    }
    displayData();                      // print the initial sensor data
    mySensor.tUpper = 25 * 4;           // 25C (77F)
    mySensor.tLower = -10 * 4;          // -10C (14F)
    mySensor.tCritical = 30 * 4;        // 30C (86F)
    mySensor.write();                   // write the changed values
}

void loop()
{
    delay(2000);
    displayData();
}

// print the data from the sensor on the serial monitor
void displayData()
{
    uint8_t status;

    if ( (status = mySensor.read()) == 0 )
    {
        float C = mySensor.tUpper / 16.0;
        float F = C * 9.0 / 5.0 + 32.0;
        Serial << F("\nUpper Limit\t") << C << F("C\t") << F << F("F\tAlert=") << mySensor.alertUpper << endl;
        C = mySensor.tLower / 16.0;
        F = C * 9.0 / 5.0 + 32.0;
        Serial << F("Lower Limit\t") << C << F("C\t") << F << F("F\tAlert=") << mySensor.alertLower << endl;
        C = mySensor.tCritical / 16.0;
        F = C * 9.0 / 5.0 + 32.0;
        Serial << F("Critical Limit\t") << C << F("C\t") << F << F("F\tAlert=") << mySensor.alertCritical << endl;
        C = mySensor.tAmbient / 16.0;
        F = C * 9.0 / 5.0 + 32.0;
        Serial << F("Ambient\t\t") << C << F("C\t") << F << F("F\n");
        Serial << F("Config 0x") << _HEX(mySensor.config) << F(" Resolution 0x") << _HEX(mySensor.resolution) << endl;
        Serial << F("Mfr ID 0x") << _HEX(mySensor.mfrID) << F( " Device ID 0x") << _HEX(mySensor.deviceID) << F( " Device Rev 0x") << _HEX(mySensor.deviceRev) << endl;
    }
    else
    {
        Serial << "Error reading sensor, status=" << status << endl;
    }
}
