# Arduino Library for Microchip MCP9808 Maximum Accuracy Digital Temperature Sensor
https://github.com/JChristensen/MCP9808  
ReadMe file  
Jack Christensen Jun 2015

![CC BY-SA](http://mirrors.creativecommons.org/presskit/buttons/80x15/png/by-sa.png)

## Introduction
**Arduino Library for Microchip MCP9808 Maximum Accuracy Digital Temperature Sensor**

A lightweight implementation that exposes all functionality of the [Microchip MCP9808 sensor](https://www.microchip.com/wwwproducts/en/mcp9808). Temperatures are handled in the integer domain to avoid the code size and runtime overhead associated with floating-point. Should the user wish to work in floating-point format, conversion is easily done in the application code.

The ambient temperature is read as °C\*16. If a lower resolution is configured, the applicable low-order bits are returned as zeroes.

The Upper, Lower and Critical temperature limit registers are read and written as °C\*4.

Bit masks for the config register are provided in the **MCP9808.h** file.  

"Arduino Library for Microchip MCP9808 Temperature Sensor" by Jack Christensen is licensed under [CC BY-SA 4.0](http://creativecommons.org/licenses/by-sa/4.0/).

## Examples
The following example sketch is included with the **MCP9808** library:

- **MCP9808_EX1:** Demonstrates reading data from the sensor and changing the temperature limit registers. Converts the ambient temperature reading to floating point in °C and °F.

## Enumeration

### twiClockFreq_t
##### Description
Symbolic names for I2C bus speeds.
##### Values
- **MCP9808::twiClock100kHz** -- 100kHz bus speed.  
- **MCP9808::twiClock400kHz** -- 400kHz bus speed.  

## Constructor

### MCP9808(uint8_t LS_ADDR_BITS)
##### Description
Instantiates an MCP9808 sensor object.
##### Syntax
```c++
MCP9800 mySensor(0);
// or
MCP9800 mySensor;
```
##### Parameters
**LS_ADDR_BITS:** An unsigned 8-bit integer *(uint8_t or byte)* representing the least-significant three bits of the I2C device address (i.e. a value between 0 and 7). Defaults to zero if omitted.

## Methods
### begin(twiClockFreq_t twiFreq)
##### Description
Initializes the library. Calling `begin()` calls `read()` to initially populate the sensor data into local storage. Call this method once in the setup code.
##### Syntax
`mySensor.begin(twiClockFreq_t freq);`
##### Parameters
**twiFreq:** An optional parameter specifying the speed for the I2C bus. If omitted, it defaults to 100kHz *(MCP9808::twiClockFreq_t)*
##### Returns
I2C status (zero if successful) *(uint8_t)*.
##### Example
```c++
MCP9808 mySensor;
mySensor.begin(MCP9808::twiClock400kHz);
```
### read()
##### Description
Reads data from the sensor into local storage. See Properties below for a description of the data.
##### Syntax
`mySensor.read();`
##### Parameters
None.
##### Returns
I2C status (zero if successful) *(uint8_t)*.
##### Example
```c++
MCP9808 mySensor;
mySensor.begin();
uint8_t e = mySensor.read();
if ( e != 0 )
{
	//there was an error on the bus
}
else
{
	//all is well
}
```
### write()
##### Description
Writes data from local storage to the sensor. Note that only the three critical temperature registers, the configuration register and the resolution register are written; other registers are read-only.
##### Syntax
`mySensor.write();`
##### Parameters
None.
##### Returns
I2C status (zero if successful) *(uint8_t)*.
##### Example
```c++
MCP9808 mySensor;
mySensor.begin();
mySensor.tUpper = 100;    //set upper limit to 25C (77F)
mySensor.write();
```

## Properties
**tUpper:** Upper temperature limit, in units of °C\*4. _(int16\_t)_  
**tLower:** Lower temperature limit, in units of °C\*4. _(int16\_t)_  
**tCritical:** Critical temperature limit, in units of °C\*4. *(int16\_t)*  
**tAmbient:** Ambient temperature, in units of °C\*16. Read-only. *(int16\_t)*  
**config:** Configuration register, see datasheet for details. *(uint16\_t)*  
**resolution:** Temperature resolution for the ambient temperature. An integer between 0 and 3, where 0 is ±0.5°C, 1 is ±0.25°C, 2 is ±0.125°C, and 3 is ±0.0625°C. Higher resolutions require longer conversion times, see the datasheet. ±0.0625°C is the power-on default. *(uint8\_t)*  
**alertCritical:** If tAmbient ≥ tCritical, then *true*, else *false*. Read-only. *(bool)*  
**alertUpper:** If tAmbient > tUpper, then *true*, else *false*. Read-only. *(bool)*  
**alertLower:** If tAmbient < tLower, then *true*, else *false*. Read-only. *(bool)*  
**mfrID:** Manufacturer ID. Read-only. *(uint16\_t)*  
**deviceID:** Device ID. Read-only. *(uint8\_t)*  
**deviceRev:** Device revision. Read-only. *(uint8\_t)*  
