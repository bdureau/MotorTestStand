#ifndef _CONFIG_H
#define _CONFIG_H
/*
 * 
 * This is where you should do all the configuration
 * 
 * First you need to make sure that you have all the require Arduino libraries to compile it
 * 
 * required libraries:
 * 
 */

/////////////// config changes start here ///////////
// here choose one of the board that you want to use
// note that you will need to compile using the Arduino Uno or SMT32 board
//#define TESTSTAND


// if you have the STM32 shield then define TESTSTANDSTM32
//#define TESTSTANDSTM32

#define TESTSTANDSTM32V2

#ifdef TESTSTANDSTM32
#define BOARD_FIRMWARE "TestStandSTM32"
#endif
#ifdef TESTSTANDSTM32V2
#define BOARD_FIRMWARE "TestStandSTM32V2"
#endif
#ifdef TESTSTAND
#define BOARD_FIRMWARE "TestStand"
#endif

// If you want to have additionnal debugging uncomment it
//#define SERIAL_DEBUG
#undef SERIAL_DEBUG

#define BAT_MIN_VOLTAGE 7.0
//Voltage divider
#define R1 4.7
#define R2 10

#define VOLT_DIVIDER 10*(R1/(R1+R2))
#define VOLT_DIVIDER_PRESSURE (R2/(R2+R1))

////////////// config changes end here /////////////
//////////// do not change anything after unless you know what you are doing /////////////////////

#define MAJOR_VERSION 1
#define MINOR_VERSION 2
#define BUILD 1
#define CONFIG_START 32

#ifdef TESTSTANDSTM32
#include <itoa.h>
#endif

#ifdef TESTSTANDSTM32V2
#include <itoa.h>
#endif


#ifdef TESTSTANDSTM32 
#define SerialCom Serial1
#endif
#ifdef TESTSTANDSTM32V2
#define SerialCom Serial1
#endif
#ifdef TESTSTAND
#define SerialCom Serial
#endif



#include "Arduino.h"
//used for writing in the microcontroler internal eeprom
#include <EEPROM.h>


struct ConfigStruct {
  int unit;             // 0 = kg 1 = pound 
  long connectionSpeed; //by default 38400 bauds
  int endRecordTime;  // stop recording when minimum thrust
  int standResolution;  // number of sample per second
  int eepromSize;
  int startRecordThrust; //start recording when minimum thrust
  int batteryType; // 0= Unknown, 1= "2S (7.4 Volts)", 2 = "9 Volts",3 = "3S (11.1 Volts)
  long calibration_factor;
  long current_offset;
  int pressure_sensor_type; //0 = none 
  int cksum;  
};
extern ConfigStruct config;

extern void defaultConfig();
extern bool readTestStandConfig();
extern int getOutPin(int );
//extern bool writeTestStandConfig( char * );
extern bool writeTestStandConfigV2( char * );
extern void printTestStandConfig();
extern void writeConfigStruc();
extern bool CheckValideBaudRate(long);
extern unsigned int CheckSumConf( ConfigStruct );
extern unsigned int msgChk( char * buffer, long length );
#endif
