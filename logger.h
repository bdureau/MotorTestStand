#ifndef _LOGGER_H
#define _LOGGER_H

// config.h first: it holds the board #define tested below
#include "config.h"
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
#include <LittleFS.h>
/*
 * Description: This is used only by the ESP32 V4 version of the teststand that uses the
 * ESP32 filesystem (LittleFS) to store the thrust curves rather than the external I2C eeprom.
 *
 * Storage format: one binary file per thrust curve, "/thrustCurve<N>.bin", containing a
 * packed array of ThrustCurveDataStruct (16 bytes per sample). The number of samples is
 * file.size() / sizeof(ThrustCurveDataStruct). No JSON, so no document-capacity limit.
 */
#define FORMAT_LITTLEFS_IF_FAILED true
#define THRUSTCURVE_FILE_PREFIX   "thrustCurve"
#define THRUSTCURVE_FILE_EXT      ".bin"
#define THRUSTCURVE_ALLOC_CHUNK   64      // samples added to the RAM buffer per realloc
//#define DEBUG

struct ThrustCurveDataStruct {
  long diffTime;
  long thrust;
  long pressure;
  long pressure2;
};

struct ThrustCurveMinAndMaxStruct {
  long minThrust;
  long maxThrust;
  long minPressure;
  long maxPressure;
  long minPressure2;
  long maxPressure2;
  long thrustCurveDuration;
};

class logger {
  public:
    logger();
    bool initFileSystem();
    bool isReady();
    bool clearThrustCurveList();
    bool eraseLastThrustCurve();
    long getLastThrustCurveNbr();
    bool writeThrustCurve(long thrustCurveNbr);
    bool writeFastThrustCurve();
    bool readThrustCurve(long thrustCurveNbr);
    bool addToCurrentThrustCurve();
    bool initThrustCurve();
    ThrustCurveDataStruct* getThrustCurveData();
    void setThrustCurveTimeData(long diffTime);
    void setThrustCurveData(long thrust);
    void setPressureCurveData(long pressure);
    void setPressureCurveData2(long pressure);

    void getThrustCurveMinAndMax(long thrustCurveNbr);
    long getMaxPressure();
    long getMaxPressure2();
    long getMinThrust();
    long getMaxThrust();
    long getThrustCurveDuration();
    long getThrustCurveSize();
    int  getUsedPercent();
    void printThrustCurveData(int thrustCurveNbr);

  private:
    ThrustCurveDataStruct* thrustCurveData;
    ThrustCurveDataStruct currentRecord;
    long dataPos;        // number of valid samples in thrustCurveData
    long dataCapacity;   // number of samples allocated in thrustCurveData
    bool fsReady;
    void buildFileName(char *buf, size_t len, long thrustCurveNbr);
    long parseThrustCurveNbr(const char *fileName);
    unsigned int msgChk( char * buffer, long length );
    ThrustCurveMinAndMaxStruct _ThrustCurveMinAndMax;
};

#endif
#endif
