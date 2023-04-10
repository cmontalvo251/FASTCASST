/*-----------------------------------------------------------------------------*
 * Arduino Library for Microchip MCP9808 Maximum Accuracy Digital              *
 * Temperature Sensor                                                          *
 *                                                                             *
 * A lightweight implementation that exposes all functionality of the MCP9808  *
 * sensor.  Temperatures are handled in the integer domain to avoid            *
 * the code size and runtime overhead associated with floating-point.          *
 * Should the user wish to work in floating-point format, conversion           *
 * is easily done in the application code.                                     *
 *                                                                             *
 * The ambient temperature is read as °C*16. If a lower resolution             *
 * is configured, the applicable low-order bits are returned as zeroes.        *
 *                                                                             *
 * The Upper, Lower and Critical temperature limit registers are read and      *
 * written as °C*4.                                                            *
 *                                                                             *
 * Bit masks for the config register are provided in the MCP9808.h file.       *
 *                                                                             *
 * Jack Christensen 15Apr2015 v1.0                                             *
 *                                                                             *
 * "Arduino Library for Microchip MCP9808 Temperature Sensor"                  *
 * by Jack Christensen is licensed under CC BY-SA 4.0,                         *
 * http://creativecommons.org/licenses/by-sa/4.0/                              *
 *-----------------------------------------------------------------------------*/

#ifndef MCP9808_H_INCLUDED
#define MCP9808_H_INCLUDED

#include <Arduino.h>
#include <Wire.h>

// device i2c base address
const int MCP9808_BASE_ADDR(0x18);

// register addresses
const uint8_t
        CONFIG_REG(0x01),
        tUPPER_REG(0x02),
        tLOWER_REG(0x03),
     tCRITICAL_REG(0x04),
      tAMBIENT_REG(0x05),
        MFR_ID_REG(0x06),
        DVC_ID_REG(0x07),
    RESOLUTION_REG(0x08);

// device has 8 registers, 15 bytes total (RESOLUTION_REG is one byte)
const uint8_t
          N_REGS(8),
    SIZEOF_REGS(15);

// bit definitions for config register
const uint16_t
    HYSTERESIS(0x0600),
          SHDN(0x0100),
       CRT_LOC(0x0080),
       WIN_LOC(0x0040),
       INT_CLR(0x0020),
      ALT_STAT(0x0010),
       ALT_CNT(0x0080),
       ALT_SEL(0x0040),
       ALT_POL(0x0020),
       ALT_MOD(0x0010);

// bit definitions for ambient temperature register MSB
const uint8_t
     ALT_CRIT(0x80),
    ALT_UPPER(0x40),
    ALT_LOWER(0x20);

class MCP9808
{
    public:
        // I2C clock frequencies
        enum twiClockFreq_t { twiClock100kHz = 100000, twiClock400kHz = 400000 };

        MCP9808(uint8_t LS_ADDR_BITS = 0);

        uint8_t
            begin(twiClockFreq_t twiFreq = twiClock100kHz),
            read(),         // read all device data
            write();        // write all device registers that are not read-only
        
        int16_t
            tUpper,         // upper temperature limit, in units of 1/4 °C
            tLower,         // lower temperature limit, in units of 1/4 °C
            tCritical,      // critical temperature limit, in units of 1/4 °C
            tAmbient;       // ambient temperature, in units of 1/16 °C

        uint16_t
            config,         // configuration register
            mfrID;          // manufacturer ID
            
        uint8_t
            deviceID,       // device ID
            deviceRev,      // device revision
            resolution;     // resolution for tAmbient
            
        bool
            alertCritical,  // tAmbient >= tCritical
            alertUpper,     // tAmbient > tUpper
            alertLower;     // tAmbient < tLower

    private:
        uint8_t
            m_devAddr;
};
#endif
