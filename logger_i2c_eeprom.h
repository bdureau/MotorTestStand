#ifndef _LOGGER_I2C_EEPROM_H
#define _LOGGER_I2C_EEPROM_H

#include <Wire.h>
#include "config.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "Wstring.h"
#include "Wiring.h"
#endif
// TWI buffer needs max 2 bytes for eeprom address
// 1 byte for eeprom register address is available in txbuffer
#define I2C_TWIBUFFERSIZE  30


struct ThrustCurveDataStruct {
  long diffTime;
  long thrust;
  #if defined TESTSTANDSTM32V2 || defined TESTSTANDESP32 || defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3
  long casing_pressure;
  #endif

  #if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3
  long casing_pressure2;
  long thrust_filtered;
  #endif
};


struct ThrustCurveConfigStruct {
  long ThrustCurve_start;    
  long ThrustCurve_stop; 
};

#define LOGGER_I2C_EEPROM_VERSION "1.0.0"

// The DEFAULT page size. This is overriden if you use the second constructor.
// I2C_EEPROM_PAGESIZE must be multiple of 2 e.g. 16, 32 or 64
// 24LC256 -> 64 bytes
#define LOGGER_I2C_EEPROM_PAGESIZE 128 //64
#define THRUSTCURVE_LIST_START 0
#define THRUSTCURVE_DATA_START 200
class logger_I2C_eeprom
{
public:
    /**
     * Initializes the EEPROM with a default pagesize of I2C_EEPROM_PAGESIZE.
     */
    logger_I2C_eeprom(uint8_t deviceAddress);
    //logger_I2C_eeprom(uint8_t deviceAddress, const unsigned int deviceSize);
    uint8_t _deviceAddress;
    void begin();
    void clearThrustCurveList();
    void write_byte( unsigned long eeaddress, uint8_t data );
    uint8_t read_byte(  unsigned long eeaddress );
    unsigned long readThrustCurve(unsigned long eeaddress);
    int readThrustCurveList();
    int writeThrustCurveList();
    int getLastThrustCurveNbr();
    bool eraseLastThrustCurve();
    int printThrustCurveList();
    void setThrustCurveStartAddress(int ThrustCurveNbr, long startAddress);
    void setThrustCurveEndAddress(int ThrustCurveNbr, long endAddress);
    void setThrustCurveTimeData( long difftime);
    long getThrustCurveTimeData();
    void setThrustCurveData( long thrust);
    long getThrustCurveData();
    #if defined TESTSTANDSTM32V2 || defined TESTSTANDESP32 || defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3
    long getPressureCurveData();
    void setPressureCurveData( long pressure);
    #endif
    #if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3
    long getPressureCurveData2();
    void setPressureCurveData2( long pressure);
    void setThrustCurveDataFiltered( long thrust);
    long getThrustCurveDataFiltered();
    #endif
    long getThrustCurveStart(int ThrustCurveNbr);
    long getThrustCurveStop(int ThrustCurveNbr);
    void printThrustCurveData(int ThrustCurveNbr);
    long checkMemoryErrors(long memoryLastAddress);
    int checkMemorySize();
    bool checkWrite(long address);
    boolean CanRecord();
    unsigned long writeFastThrustCurve(unsigned long eeaddress);
    long getSizeOfThrustCurveData();
    long getLastThrustCurveEndAddress();   
    
private: 
    ThrustCurveConfigStruct _ThrustCurveConfig[25];
    ThrustCurveDataStruct _ThrustCurveData;
    uint8_t _pageSize;
};

#endif
// END OF FILE
