// "Arduino Library for Microchip MCP9808 Temperature Sensor"
// by Jack Christensen is licensed under CC BY-SA 4.0,
// http://creativecommons.org/licenses/by-sa/4.0/

#include <MCP9808.h>

// constructor
// instantiate a temperature sensor object given the least three significant
// bits (A2:0) of its I2C address (0-7)
MCP9808::MCP9808(uint8_t LS_ADDR_BITS)
{
    m_devAddr = MCP9808_BASE_ADDR | (LS_ADDR_BITS & 7);
}

// initialize
uint8_t MCP9808::begin(twiClockFreq_t twiFreq)
{
    Wire.begin();
    Wire.setClock(twiFreq);
    return read();
}

// read all registers from the sensor.
// device does not support sequential access, so must read one register at a time :-|
// returns the I2C status (zero if successful). 
uint8_t MCP9808::read()
{
    uint8_t regs[SIZEOF_REGS];  // temporary storage for register data
    uint8_t* rp = regs;
    uint8_t e = 0;

    // first seven registers are two bytes each
    for (uint8_t r = CONFIG_REG; r < N_REGS; ++r) {
        Wire.beginTransmission(m_devAddr);
        Wire.write(r);
        if ( (e = Wire.endTransmission(true)) != 0 ) return e;
        Wire.requestFrom(m_devAddr, (uint8_t)2);
        *rp++ = Wire.read();
        *rp++ = Wire.read();
    }

    // last register is one byte
    Wire.beginTransmission(m_devAddr);
    Wire.write(RESOLUTION_REG);
    if ( (e = Wire.endTransmission(true)) != 0 ) return e;
    Wire.requestFrom(m_devAddr, (uint8_t)1);
    *rp = Wire.read();
    
    // assign to class members
    config = ( regs[0] << 8 ) + regs[1];
    tUpper    = (((int16_t)regs[2] << 11) + ((int16_t)regs[3] << 3)) >> 3;
    tLower    = (((int16_t)regs[4] << 11) + ((int16_t)regs[5] << 3)) >> 3;
    tCritical = (((int16_t)regs[6] << 11) + ((int16_t)regs[7] << 3)) >> 3;
    tAmbient  = (((int16_t)regs[8] << 11) + ((int16_t)regs[9] << 3)) >> 3;
    alertCritical = regs[8] & ALT_CRIT;
    alertUpper    = regs[8] & ALT_UPPER;
    alertLower    = regs[8] & ALT_LOWER;
    mfrID = ( regs[10] << 8 ) + regs[11];
    deviceID = regs[12];
    deviceRev = regs[13];
    resolution = regs[14];

    return e;
}

// write config, temperature and resolution registers (others are read-only)
// device does not support sequential access, so must write one register at a time :-|
// returns the I2C status (zero if successful). 
uint8_t MCP9808::write()
{
    uint8_t e = 0;

    Wire.beginTransmission(m_devAddr);       // configuration register
    Wire.write( CONFIG_REG );
    Wire.write( config >> 8 );
    Wire.write( config & 0xFF );
    if ( (e = Wire.endTransmission(true)) != 0 ) return e;

    Wire.beginTransmission(m_devAddr);       // upper limit
    Wire.write( tUPPER_REG );
    Wire.write( tUpper >> 6 );
    Wire.write( tUpper << 2 );
    if ( (e = Wire.endTransmission(true)) != 0 ) return e;

    Wire.beginTransmission(m_devAddr);       // lower limit
    Wire.write( tLOWER_REG );
    Wire.write( tLower >> 6 );
    Wire.write( tLower << 2 );
    if ( (e = Wire.endTransmission(true)) != 0 ) return e;

    Wire.beginTransmission(m_devAddr);       // critical limit
    Wire.write( tCRITICAL_REG );
    Wire.write( tCritical >> 6 );
    Wire.write( tCritical << 2 );
    if ( (e = Wire.endTransmission(true)) != 0 ) return e;
    
    Wire.beginTransmission(m_devAddr);       // resolution register
    Wire.write( RESOLUTION_REG );
    Wire.write( resolution & 0x03 );
    return Wire.endTransmission(true);
}
