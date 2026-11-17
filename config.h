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
//#define TESTSTANDSTM32V2
//#define TESTSTANDSTM32V3 //use the 2 analog inputs to measure pressure

// if you have the esp32 version
//#define TESTSTANDESP32
//#define TESTSTANDESP32V3 //use the 2 analog inputs to measure pressure
#define TESTSTANDESP32V4 //save thrust curve on the ESP32 chip


#ifdef TESTSTANDSTM32
#define BOARD_FIRMWARE "TestStandSTM32"
#endif
#ifdef TESTSTANDSTM32V2
#define BOARD_FIRMWARE "TestStandSTM32V2"
#endif
#ifdef TESTSTANDSTM32V3
#define BOARD_FIRMWARE "TestStandSTM32V3"
#endif
#ifdef TESTSTAND
#define BOARD_FIRMWARE "TestStand"
#endif
#ifdef TESTSTANDESP32
#define BOARD_FIRMWARE "TestStandESP32"
#endif
#ifdef TESTSTANDESP32V3
#define BOARD_FIRMWARE "TestStandESP32V3"
#endif
#ifdef TESTSTANDESP32V4
#define BOARD_FIRMWARE "TestStandESP32V4"
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

// ESP32 only: number of back-to-back ADC samples averaged for each analog measurement.
// analogReadMilliVolts() takes a few tens of microseconds, so 16 samples cost well under 1 ms
// (the 1.9 version used 40 samples with a 1 ms delay each = ~40 ms per measurement).
#define ESP32_ADC_OVERSAMPLING 16
// ESP32 only: minimum interval between two battery voltage readings when not recording (ms).
// The battery is never read inside the recording loop, the cached value is used instead.
#define ESP32_BATTERY_READ_INTERVAL 1000

////////////// config changes end here /////////////
//////////// do not change anything after unless you know what you are doing /////////////////////

#define MAJOR_VERSION 1
#define MINOR_VERSION 10
#define BUILD 1
#define CONFIG_START 32

#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
#include <itoa.h>
#define SerialCom Serial1
#endif


#ifdef TESTSTAND
#define SerialCom Serial
#endif

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
#include "BluetoothSerial.h"
extern BluetoothSerial SerialBT;
#define SerialCom SerialBT
#define BUFFER_LENGTH I2C_BUFFER_LENGTH
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
  int telemetryType;
  #if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
  int pressure_sensor_type2; //0 = none
  #endif
  int cksum;  
};
extern ConfigStruct config;

extern void defaultConfig();
extern bool readTestStandConfig();
extern int getOutPin(int );
extern bool writeTestStandConfigV2( char * );
extern void printTestStandConfig(char *);
extern void writeConfigStruc();
extern bool CheckValideBaudRate(long);
extern unsigned int CheckSumConf( ConfigStruct );
extern unsigned int msgChk( char * buffer, long length );
#endif
