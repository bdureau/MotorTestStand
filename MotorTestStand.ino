/*
  Rocket Motor test stand ver 1.10
  Copyright Boris du Reau 2012-2026

  The following is a datalogger for logging rocket motor thrustcurves.

  The code works with different boards, you need to adjust the config.h file to reflect your board.

  This is using a scale load cell as the sensor, a HX711 as the cell amplifier and an Atmega 328 or an STM32
  The following record the motor thrustcurve and saves it in an EEPROM

  For the EEPROM 24LC512-I/P - SERIAL EEPROM 512K,24LC512, DIP8

  Number Name ConnectTo     Note
  1       a0     GND
  2       a1     GND
  3       a2     GND
  4       GND    GND
  5       SDA    SDA         2kOhm PullUp
  6       SCL    SCL
  7       WP     GND
  8       VCC    (+3v3 ... +5V0)

  The time to write in the EEPROM is 10ms

  Major changes on version 1.0
  Initial version. This is based on my datalogger code and uses the same principles
  Major Changes on version 1.1
  Added connection test
  Major Changes on version 1.2
  Added casing pressure
  Major Changes on version 1.3
  Telemetry fixes
  Major Changes on version 1.4
  Ported to ESP32
  Major Changes on version 1.5
  Use internal HX711 lib
  Changed calibration routines
  Fixed issues with ESP32
  Major Changes on version 1.6
  Record second analog input when available
  Major Changes on version 1.7
  Fixes to the calibration routines
  Major Changes on version 1.8
  Added ability to rename the bluetooth for ESP32
  Major Changes on version 1.9
  Added ability to save the flight on the ESP32 board rather than the eeprom
  Major Changes on version 1.10
  ESP32 sampling rate improvements (all ESP32 versions):
  - Analog inputs are now read with analogReadMilliVolts() (factory calibrated ADC)
    using a fast oversampling without delay, replacing the 40 x delay(1) polynomial
    correction that was taking ~40 ms per measurement
  - The battery voltage is no longer read inside the recording loop, a cached value
    is sent in the telemetry
  - Fixed ReadPressure2() that was using the first pressure sensor type
*/

//Test Stand configuration lib
#include "config.h"
#include <Wire.h> //I2C library
//#include "kalman.h"
#include "beepfunc.h"

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
#include "logger_i2c_eeprom.h"
#endif

#include "BHX711.h"


#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
BluetoothSerial SerialBT;
#endif


// HX711 circuit wiring
#ifdef TESTSTAND
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;
#endif
#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
const int LOADCELL_DOUT_PIN = PB15;
const int LOADCELL_SCK_PIN = PB14;
#endif

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 19;
#endif

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
#include <Preferences.h>
Preferences preferences;
#endif

#if defined TESTSTANDESP32V4
#include "logger.h"
#endif

//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////

BHX711 scale;

#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3 || defined TESTSTANDESP32 || defined TESTSTANDESP32V3
//EEProm address
logger_I2C_eeprom logger(0x50) ;
// End address of the 512 eeprom
long endAddress = 65536;
// EEPROM start address for the thrust curve. Anything before that is the flight index
long currentMemaddress = 200;
#endif

#if defined TESTSTANDESP32V4
logger logger;
// % of the LittleFS partition used, refreshed outside of recording (telemetry field)
int fsUsedPercent = 0;
#endif
//stop recording a maximum of 20 seconds after the motor has fired
long recordingTimeOut = 20000;
boolean canRecord = true;
boolean exitRecording = true;
long currentThrustCurveNbr;

long currPressure = 0;

#if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
long currPressure2 = 0;
#endif
long currThrust = 0;

boolean motorStarted = false;
unsigned long initialTime;
boolean FastReading = false;
#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
const int startPin =  PA1;
#endif

#if defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
const int pressurePin = PA7;
#endif

#ifdef TESTSTANDSTM32V3
const int pressurePin2 = PA8;
#endif

#ifdef TESTSTAND
const int startPin =  10;
#endif

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
const int startPin =  23;
const int pressurePin = 33;
#endif

#if defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
const int pressurePin2 = 32;
#endif

int startState = HIGH;
//telemetry
boolean telemetryEnable = false;
long lastTelemetry = 0;
long lastBattWarning = 0;
boolean recording = false;

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
// cached battery voltage (Volts), refreshed outside of the recording loop only
float batteryVoltage = 0.0;
unsigned long lastBatteryRead = 0;
#endif


void MainMenu();
/*
   Float replacement of the map function
*/
float map_tofloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
  ResetGlobalVar()
*/
void ResetGlobalVar() {
  exitRecording = false;
  motorStarted = false;
}

/*
   initTestStand()
*/
void initTestStand() {

  ResetGlobalVar();

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  delay(1000);
  if (config.calibration_factor != 0)
    scale.set_scale((float)config.calibration_factor);

  if (config.current_offset != 0)
    scale.set_offset( config.current_offset);
  scale.tare(100);

  SerialCom.println("Scale tared");
}

/*
   ReadThrust()
*/
long ReadThrust() {
  long thrust = 0;

  if (config.standResolution == 3)
    thrust = (long) ((scale.get_units(1)) * 1000);
  else if (config.standResolution == 2)
    thrust = (long) ((scale.get_units(2)) * 1000);
  else if (config.standResolution == 1)
    thrust = (long) ((scale.get_units(4)) * 1000);
  else if (config.standResolution == 0)
    thrust = (long) ((scale.get_units(8)) * 1000);
  return thrust;
}



#if defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
long ReadPressure() {

  float sum = 0;
  for (int i = 0; i < 5; i++) {
    int pressure = analogRead(pressurePin);

    sum += (float)map_tofloat( ((float)(pressure * 3300) / (float)4096000 / VOLT_DIVIDER_PRESSURE),
                               0.5,
                               4.5,
                               0.0,
                               (float)pressureSensorTypeToMaxValue(config.pressure_sensor_type));
    delay(1);
  }
  return (long)  (sum / 5.0);
}
#endif

#if defined TESTSTANDSTM32V3
long ReadPressure2() {

  float sum = 0;
  for (int i = 0; i < 5; i++) {
    int pressure = analogRead(pressurePin2);

    sum += (float)map_tofloat( ((float)(pressure * 3300) / (float)4096000 / VOLT_DIVIDER_PRESSURE),
                               0.5,
                               4.5,
                               0.0,
                               (float)pressureSensorTypeToMaxValue(config.pressure_sensor_type2));
    delay(1);
  }
  return (long)  (sum / 5.0);
}
#endif

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
/*
   analogReadMilliVoltsAvg(pin)
   Fast oversampled analog read for the ESP32.
   analogReadMilliVolts() uses the ADC factory calibration (eFuse) so no polynomial
   correction is needed anymore. Samples are taken back to back without any delay
   so that a measurement takes well under 1 ms.
   Returns the voltage at the pin in millivolts.
*/
float analogReadMilliVoltsAvg(uint8_t pinNumber) {
  uint32_t sum = 0;
  for (int i = 0; i < ESP32_ADC_OVERSAMPLING; i++) {
    sum += analogReadMilliVolts(pinNumber);
  }
  return (float)sum / (float)ESP32_ADC_OVERSAMPLING;
}

/*
   pressureFromMilliVolts(milliVolts, sensorType)
   Convert the voltage measured at the pin into a pressure (PSI).
   The 0.5V-4.5V sensor output goes through the voltage divider before the ADC.
*/
long pressureFromMilliVolts(float milliVolts, int sensorType) {
  return (long)map_tofloat( (milliVolts / 1000.0f / VOLT_DIVIDER_PRESSURE),
                            0.5,
                            4.5,
                            0.0,
                            (float)pressureSensorTypeToMaxValue(sensorType));
}

long ReadPressure() {
  return pressureFromMilliVolts(analogReadMilliVoltsAvg(pressurePin), config.pressure_sensor_type);
}

/*
   readBatteryVoltage()
   Read the battery voltage (Volts) and refresh the cached value.
   This must NOT be called in the recording loop, use batteryVoltage instead.
*/
float readBatteryVoltage() {
  batteryVoltage = VOLT_DIVIDER * (analogReadMilliVoltsAvg(2) / 1000.0f);
  lastBatteryRead = millis();
  return batteryVoltage;
}

/*
   updateBatteryVoltage()
   Refresh the cached battery voltage at most every ESP32_BATTERY_READ_INTERVAL ms.
   Does nothing while recording so that the sampling rate is not affected.
*/
void updateBatteryVoltage() {
  if (!recording && (millis() - lastBatteryRead) > ESP32_BATTERY_READ_INTERVAL)
    readBatteryVoltage();
}
#endif

#if defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
long ReadPressure2() {
  // use the second sensor type (version 1.9 was wrongly using pressure_sensor_type)
  return pressureFromMilliVolts(analogReadMilliVoltsAvg(pressurePin2), config.pressure_sensor_type2);
}
#endif

/*
   setup()
   Initialise the test Stand
*/
void setup()
{
  // initialise the connection
  Wire.begin();

  //soft configuration
  boolean softConfigValid = false;
  // Read test stand softcoded configuration
  softConfigValid = readTestStandConfig();

  // check if configuration is valid
  if (!softConfigValid)
  {
    //default values
    defaultConfig();
    //config.cksum = CheckSumConf(config);
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4|| defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
    writeConfigStruc();
#endif
  }

  // if the baud rate is invalid let's default it
  if (!CheckValideBaudRate(config.connectionSpeed))
  {
    config.connectionSpeed = 38400;
    config.cksum = CheckSumConf(config);
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
    writeConfigStruc();
#endif
  }

#ifdef TESTSTANDSTM32
  analogReadResolution(12); //// need to review !!!!
#endif
#if defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
  pinMode(PA7, INPUT_ANALOG);
  analogReadResolution(12); //// need to review !!!!
#endif

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
  // 12 bits ADC with 11dB attenuation (full range) on all the analog inputs.
  // analogReadMilliVolts() takes the attenuation into account.
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  // first battery reading so that the telemetry has a valid value
  readBatteryVoltage();
#endif

  //You can change the baud rate here
  //and change it to 57600, 115200 etc..
  config.connectionSpeed = 38400;

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
  Serial.begin(38400);
  char bluetoothName [21];
  preferences.begin("namespace", false);
  sprintf(bluetoothName, "%s", preferences.getString("Name", "ESP32Stand"));
  preferences.end();
  Serial.println(bluetoothName);
  SerialCom.begin(bluetoothName);
#else
  SerialCom.begin(38400);
#endif
  SerialCom.print("pressure_sensor_type" );
  SerialCom.println(config.pressure_sensor_type);
  SerialCom.print("connectionSpeed" );
  SerialCom.println(config.connectionSpeed);

#if defined TESTSTANDESP32V4
  if (!logger.initFileSystem()) {
    Serial.println(F("Failed to initialize file system"));
    SerialCom.println(F("Failed to initialize file system"));
  } else {
    Serial.println(F("initFileSystem Ok"));
  }
  logger.initThrustCurve();
#endif

#ifdef TESTSTAND
  //software pull up so that all bluetooth modules work!!! took me a good day to figure it out
  pinMode(PD0, INPUT_PULLUP);
#endif

  //software pull up so that all bluetooth modules work!!!
#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
  pinMode(PB11, INPUT_PULLUP);
#endif

#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3 || defined TESTSTAND || defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
  pinMode(startPin, INPUT_PULLUP);
#endif


  SerialCom.print(F("Start program\n"));
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
  Serial.print(F("Start program\n"));
#endif
  initTestStand();
  pinMode(pinSpeaker, OUTPUT);

  digitalWrite(pinSpeaker, LOW);

  //initialisation give the version of the testStand
  //One long beep per major number and One short beep per minor revision
  //For example version 1.2 would be one long beep and 2 short beep
  beepTestStandVersion(MAJOR_VERSION, MINOR_VERSION);
  SerialCom.print("before calman\n");

  SerialCom.print(F("before read list\n"));
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
  int v_ret;
  v_ret = logger.readThrustCurveList();

  long lastThrustCurveNbr = logger.getLastThrustCurveNbr();

  if (lastThrustCurveNbr < 0)
  {
    currentThrustCurveNbr = 0;
    currentMemaddress = 201;
  }
  else
  {
    currentMemaddress = logger.getThrustCurveStop(lastThrustCurveNbr) + 1;
    currentThrustCurveNbr = lastThrustCurveNbr + 1;
  }
#endif

#if defined TESTSTANDESP32V4
  long lastThrustCurveNbr = logger.getLastThrustCurveNbr();
  /*if (lastThrustCurveNbr <= 0)
  {
    currentThrustCurveNbr = 0;
  }
  else
  {
    currentThrustCurveNbr = lastThrustCurveNbr + 1;
    
  }*/
  currentThrustCurveNbr = lastThrustCurveNbr + 1;
  fsUsedPercent = logger.getUsedPercent();
  // no filesystem = nowhere to save the curve
  canRecord = logger.isReady();
  if (!canRecord) {
    SerialCom.println("Cannot record");
    Serial.println("Cannot record");
    beginBeepSeq();
  }
#endif


#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
  // check if eeprom is full
  canRecord = logger.CanRecord();
  if (!canRecord) {
    SerialCom.println("Cannot record");
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.println("Cannot record");
#endif
    beginBeepSeq();
  }
  SerialCom.println("End init");
#endif
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
  Serial.println("End init");
#endif
}



/*
   SendTelemetry(long sampleTime, int freq)
   Send telemetry so that we can plot the motor Thrust
*/
void SendTelemetry(long sampleTime, int freq) {
  char testStandTelem[150] = "";

  char temp[10] = "";
  if (telemetryEnable && (millis() - lastTelemetry) > freq) {
    lastTelemetry = millis();
    int val = 0;

    strcat(testStandTelem, "telemetry," );
    sprintf(temp, "%i,", currThrust);
    strcat(testStandTelem, temp);

    sprintf(temp, "%i,", sampleTime);
    strcat(testStandTelem, temp);

#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
    pinMode(PB1, INPUT_ANALOG);
    int batVoltage = analogRead(PB1);
    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
    dtostrf(bat, 4, 2, temp);
    strcat(testStandTelem, temp);
    strcat(testStandTelem, ",");
#endif

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    // cached value: reading the battery here was costing ~40 ms per frame while recording
    dtostrf(batteryVoltage, 4, 2, temp);
    strcat(testStandTelem, temp);
    strcat(testStandTelem, ",");
#endif
#ifdef TESTSTAND
    strcat(testStandTelem, "-1,");
#endif

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
    if (!recording) {
      sprintf(temp, "%i,", (int)(100 * ((float) logger.getLastThrustCurveEndAddress() / endAddress)) );
    }
    else {
      sprintf(temp, "%i,", (int)(100 * ((float) currentMemaddress / endAddress)) );
    }
    strcat(testStandTelem, temp);
#endif

#if defined TESTSTANDESP32V4
    sprintf(temp, "%i,", fsUsedPercent );
    strcat(testStandTelem, temp);
#endif
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
    sprintf(temp, "%i,", logger.getLastThrustCurveNbr() + 1 );
#endif
#if defined TESTSTANDESP32V4
    // cached value: avoids scanning the filesystem on every telemetry frame while recording
    sprintf(temp, "%ld,", currentThrustCurveNbr );
#endif
    strcat(testStandTelem, temp);


#if defined TESTSTANDSTM32V2 || defined TESTSTANDESP32 || defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    sprintf(temp, "%i,", currPressure );
    strcat(testStandTelem, temp);
#endif
#if defined TESTSTAND || defined TESTSTANDSTM32
    strcat(testStandTelem, "-1,");
#endif
#if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    sprintf(temp, "%i,", currPressure2 );
    strcat(testStandTelem, temp);
#endif
    unsigned int chk;
    chk = msgChk(testStandTelem, sizeof(testStandTelem));
    sprintf(temp, "%i", chk);
    strcat(testStandTelem, temp);
    strcat(testStandTelem, ";\n");
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print("$");
    Serial.print(testStandTelem);
#endif
    SerialCom.print("$");
    SerialCom.print(testStandTelem);
  }
}

//================================================================
// Main loop which call the menu
//================================================================
void loop()
{
  MainMenu();
}


//================================================================
// Function:  recordThrust()
// called for normal recording
//================================================================
void recordThrust()
{
  ResetGlobalVar();
  telemetryEnable = true;
  recordingTimeOut = config.endRecordTime * 1000;
  while (!exitRecording)
  {
    //read current thrust
    currThrust = ReadThrust();
#if defined TESTSTANDSTM32V2 || defined TESTSTANDESP32 || defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    currPressure = ReadPressure();
#endif
#if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    currPressure2 = ReadPressure2();
#endif
    //if (( currThrust > config.startRecordThrust) && !recording )
    if ( !recording )
    {
      recording = true;
      SendTelemetry(0, 200);
      // save the time
      initialTime = millis();

      //resetThrustCurve();
      if (canRecord)
      {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
        //Save start address
        logger.setThrustCurveStartAddress (currentThrustCurveNbr, currentMemaddress);
#endif
#if defined TESTSTANDESP32V4 
        logger.initThrustCurve();
#endif
        delay(10);
#ifdef SERIAL_DEBUG
        SerialCom.println(F("Save start address\n"));
        SerialCom.println(currentMemaddress);
        SerialCom.println(currentThrustCurveNbr);
#endif
      }

    }
    unsigned long prevTime = 0;

    // loop until for X seconds
    while (recording)
    {
      unsigned long currentTime;
      unsigned long diffTime;

      currThrust = ReadThrust() ;
      if (currThrust < 0)
        currThrust = 0;
#if defined TESTSTANDSTM32V2 || defined TESTSTANDESP32 || defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
      currPressure = ReadPressure();
      if (currPressure < 0)
        currPressure = 0;
#endif

#if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
      currPressure2 = ReadPressure2();
      if (currPressure2 < 0)
        currPressure2 = 0;
#endif

      currentTime = millis() - initialTime;

      SendTelemetry(currentTime, 200);
      diffTime = currentTime - prevTime;
      prevTime = currentTime;


      if (canRecord)
      {
        logger.setThrustCurveTimeData( diffTime);
        logger.setThrustCurveData(currThrust);
#if defined TESTSTANDSTM32V2 || defined TESTSTANDESP32 || defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
        logger.setPressureCurveData(currPressure);
#endif

#if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
        logger.setPressureCurveData2(currPressure2);
#endif
#if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3
        logger.setThrustCurveDataFiltered(/*(long) (scale.get_units_filtered() * 1000)*/0);
#endif

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
        if ( (currentMemaddress + logger.getSizeOfThrustCurveData())  > endAddress) {
          //memory is full let's save it
          //save end address
          logger.setThrustCurveEndAddress (currentThrustCurveNbr, currentMemaddress - 1);
          canRecord = false;
        } else {
          //SerialCom.println("Recording..");
          SendTelemetry(millis() - initialTime, 100 );
          currentMemaddress = logger.writeFastThrustCurve(currentMemaddress);
          currentMemaddress++;
        }
#endif

#if defined TESTSTANDESP32V4
        if (!logger.addToCurrentThrustCurve()) {
          // out of RAM: stop adding samples, what was captured is still saved at the end
          canRecord = false;
        }
#endif
        SendTelemetry(millis() - initialTime, 100 );
        //logger.addFlightToArray();
      }

      if ( ( (millis() - initialTime) > recordingTimeOut))
      {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
        //save end address
        logger.setThrustCurveEndAddress (currentThrustCurveNbr, currentMemaddress - 1);
        logger.writeThrustCurveList();
#endif

#if defined TESTSTANDESP32V4
        if (logger.writeFastThrustCurve()) {
          Serial.println(F("Thrust curve saved"));
        } else {
          Serial.println(F("Failed to save thrust curve"));
          SerialCom.println(F("Failed to save thrust curve"));
        }
#endif

        delay(10);

        exitRecording = true;
        SendTelemetry(millis() - initialTime, 100);
        recording = false;
        SendTelemetry(millis() - initialTime, 100);
        // we have no more thrust telemetry is not required anymore
        //telemetryEnable = false;
        startState = HIGH;
        resetThrustCurve();
      }

    } // end while (start recording)
  } //end while(recording)
}




//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  Serial.print("MainMenu");
  //SerialCom.println(F("in main"));
  char readVal = ' ';
  int i = 0;

  char commandbuffer[100];

  i = 0;
  readVal = ' ';
  while ( readVal != ';')
  {
    if (!FastReading)
    {
      currThrust = ReadThrust() ;
#if defined TESTSTANDSTM32V2 || defined TESTSTANDESP32 || defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
      currPressure = ReadPressure();
#endif
#if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
      currPressure2 = ReadPressure2();
#endif
      if (recording)
        SendTelemetry(millis() - initialTime, 200);

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
      // refresh the cached battery voltage (rate limited, never while recording)
      updateBatteryVoltage();
#endif
      startState = digitalRead(startPin);

      if (startState == HIGH)
      {
        //Serial.print("HIGH");
        SendTelemetry(0, 500);
        checkBatVoltage(BAT_MIN_VOLTAGE);
      }
      else
      {
        //Serial.println("LOW");
        exitRecording = false;
        recordThrust();
      }
    }

    while (SerialCom.available())
    {
      readVal = SerialCom.read();
      if (readVal != ';' )
      {
        if (readVal != '\n')
          commandbuffer[i++] = readVal;
      }
      else
      {
        commandbuffer[i++] = '\0';
        break;
      }
    }
  }
  interpretCommandBuffer(commandbuffer);

}


/*

   This interprets menu commands. This can be used in the command line or
   this is used by the Android console

   Commands are as folow:
   a  get all thrustcurve data
   b  get teststand config
   c  toggle continuity on and off
   d  reset teststand config
   e  erase all saved thrust curve
   f  FastReading on
   g  FastReading off
   h  hello. Does not do much
   i  unused
   j  tare the testStand
   k  unused
   l  list all thrust curves
   m  followed by a number turn main loop on/off. if number is 1 then
      main loop in on else turn it off
   n  Return the number of recorded thrustcurves in the EEprom
   r  followed by a number which is the recording number.
      This will retrieve all data for the specified flight
   s  write teststand config
   t  reset teststand config (why?)
   w  Start or stop recording
   x  delete last curve
   y  followed by a number turn telemetry on/off. if number is 1 then
      telemetry in on else turn it off
*/
void interpretCommandBuffer(char *commandbuffer) {
  SerialCom.println((char*)commandbuffer);
  //get all ThrustCurve data
  if (commandbuffer[0] == 'a')
  {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$start;\n"));
#endif
    SerialCom.print(F("$start;\n"));
    int i;
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32 || defined TESTSTANDESP32V3
    for (i = 0; i < logger.getLastThrustCurveNbr() + 1; i++)
    {
      logger.printThrustCurveData(i);
    }
#endif
#if defined TESTSTANDESP32V4
    for (i = 0; i < logger.getLastThrustCurveNbr()+1; i++)
    {
      logger.printThrustCurveData(i);
    }
#endif
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$end;\n"));
#endif
    SerialCom.print(F("$end;\n"));
  }
  //get Test Stand config
  else if (commandbuffer[0] == 'b')
  {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$start;\n"));
#endif
    SerialCom.print(F("$start;\n"));

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    unsigned char bluetoothName [21];
    preferences.begin("namespace", false);

    String btName = "";
    btName = preferences.getString("Name", "ESP32Stand");
    preferences.end();

    btName.getBytes(bluetoothName, btName.length() + 1);
    //Serial.println((char *)bluetoothName);
    printTestStandConfig((char *)bluetoothName);
#else
    printTestStandConfig("");
#endif
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$end;\n"));
#endif
    SerialCom.print(F("$end;\n"));
  }
  //prepare calibrate
  else if (commandbuffer[0] == 'k')
  {
    //remove weight
    scale.tare(100);
    //offset = scale.get_offset();
  }

  //calibrate
  else if (commandbuffer[0] == 'c')
  {
    char  temp[10];
    int i = 1;
    while (commandbuffer[i] != '\0') {
      temp[i - 1] = commandbuffer[i];
      i++;
    }
    temp[i] = '\0';

    SendCalibration(config.current_offset, (long)config.calibration_factor, "Init");
    scale.calibrate_scale((float)atof(temp), 20);
    config.current_offset = scale.get_offset();
    config.calibration_factor = scale.get_scale();

    SendCalibration(config.current_offset, (long)config.calibration_factor, "In progress");
    config.cksum = CheckSumConf(config);
    SendCalibration(config.current_offset, (long)config.calibration_factor, "Done");
    writeConfigStruc();
    SerialCom.print(F("$OK;\n"));
  }
  //reset test stand config this is equal to t why do I have 2 !!!!
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    preferences.begin("namespace", false);
    preferences.putString("Name", "ESP32Stand");
    preferences.end();
#endif
    initTestStand();
  }
  //this will erase all thrust curves
  else if (commandbuffer[0] == 'e')
  {
    SerialCom.println(F("Erase\n"));
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
    logger.clearThrustCurveList();
    logger.writeThrustCurveList();
    currentThrustCurveNbr = 0;
    currentMemaddress = 201;
#endif

#if defined TESTSTANDESP32V4
    logger.clearThrustCurveList();
    logger.initThrustCurve();
    currentThrustCurveNbr = 0;
    fsUsedPercent = logger.getUsedPercent();
    canRecord = logger.isReady();
#endif
  }
  //FastReading
  else if (commandbuffer[0] == 'f')
  {
    FastReading = true;
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  //FastReading off
  else if (commandbuffer[0] == 'g')
  {
    FastReading = false;
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  // unused
  else if (commandbuffer[0] == 'i')
  {
    //exit continuity mode
  }
  //tare testStand
  else if (commandbuffer[0] == 'j')
  {
    scale.tare(100);
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  //list all ThrustCurve
  else if (commandbuffer[0] == 'l')
  {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
    SerialCom.println(F("ThrustCurve List: \n"));
    logger.printThrustCurveList();
#endif
  }
  //mainloop on/off
  else if (commandbuffer[0] == 'm')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main Loop enabled\n"));
#endif
      //mainLoopEnable = true;
    }
    else {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main loop disabled\n"));
#endif
      //mainLoopEnable = false;
    }
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  //Number of ThrustCurve
  else if (commandbuffer[0] == 'n')
  {
    char thrustCurveData[30] = "";
    char temp[9] = "";
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$start;\n"));
#endif
    SerialCom.print(F("$start;\n"));
    strcat(thrustCurveData, "nbrOfThrustCurve,");
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V4
    sprintf(temp, "%i,", logger.getLastThrustCurveNbr() + 1 );
#endif

/*#if defined TESTSTANDESP32V4
  sprintf(temp, "%i,", logger.getLastThrustCurveNbr()+1 );
#endif*/

    strcat(thrustCurveData, temp);
    unsigned int chk = msgChk(thrustCurveData, sizeof(thrustCurveData));
    sprintf(temp, "%i", chk);
    strcat(thrustCurveData, temp);
    strcat(thrustCurveData, ";\n");
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print("$");
    Serial.print(thrustCurveData);
    Serial.print(F("$end;\n"));
#endif
    SerialCom.print("$");
    SerialCom.print(thrustCurveData);
    SerialCom.print(F("$end;\n"));
  }
  // send test tram
  else if (commandbuffer[0] == 'o')
  {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$start;\n"));
#endif
    SerialCom.print(F("$start;\n"));
    sendTestTram();
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$end;\n"));
#endif
    SerialCom.print(F("$end;\n"));
  }
  //test stand config param
  //write  config
  else if (commandbuffer[0] == 'p')
  {
    if (writeTestStandConfigV2(commandbuffer)) {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
      Serial.print(F("$OK;\n"));
#endif
      SerialCom.print(F("$OK;\n"));
    }
    else {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
      Serial.print(F("$KO;\n"));
#endif
      SerialCom.print(F("$KO;\n"));
    }
  }
  else if (commandbuffer[0] == 'q')
  {
    writeConfigStruc();
    readTestStandConfig();
    initTestStand();
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));

  }
  //this will read one Thrust curve
  else if (commandbuffer[0] == 'r')
  {
    char temp[3];
    temp[0] = commandbuffer[1];

    if (commandbuffer[2] != '\0')
    {
      temp[1] = commandbuffer[2];
      temp[2] = '\0';
    }
    else
      temp[1] = '\0';

    if (atol(temp) > -1)
    {
      SerialCom.print(F("$start;\n"));
      logger.printThrustCurveData(atoi(temp));
      SerialCom.print(F("$end;\n"));
    }
    else
      SerialCom.println(F("not a valid ThrustCurve"));
  }
  //write test stand config
  else if (commandbuffer[0] == 's')
  {
    /* if (writeTestStandConfig(commandbuffer)) {
       SerialCom.print(F("$OK;\n"));
       readTestStandConfig();
       initTestStand();
      }
      else {
       SerialCom.print(F("$KO;\n"));
      }*/
  }
  //reset config and set it to default
  else if (commandbuffer[0] == 't')
  {
    //reset config
    defaultConfig();
    writeConfigStruc();
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    preferences.begin("namespace", false);
    preferences.putString("Name", "ESP32Stand");
    preferences.end();
#endif
    initTestStand();
    SerialCom.print(F("config reseted\n"));
  }
  // check memory
  else if (commandbuffer[0] == 'u')
  {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
    int memSize = logger.checkMemorySize();
    SerialCom.print(F("Memory size: "));
    SerialCom.println(memSize);

    long errors = logger.checkMemoryErrors(65500);
    SerialCom.print(F("Nbr of errors: "));
    SerialCom.println(errors);
#endif
  }
  else if (commandbuffer[0] == 'v')
  {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    preferences.begin("namespace", false);
    SerialCom.println(preferences.getString("Name", "ESP32Stand"));
    preferences.end();
#endif
  }
  // Recording
  else if (commandbuffer[0] == 'w')
  {
    SerialCom.println(F("Recording \n"));
    recordThrust();
  }
  //delete last curve
  else if (commandbuffer[0] == 'x')
  {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
    logger.eraseLastThrustCurve();
    logger.readThrustCurveList();
    long lastThrustCurveNbr = logger.getLastThrustCurveNbr();
    if (lastThrustCurveNbr < 0)
    {
      currentThrustCurveNbr = 0;
      currentMemaddress = 201;
    }
    else
    {
      currentMemaddress = logger.getThrustCurveStop(lastThrustCurveNbr) + 1;
      currentThrustCurveNbr = lastThrustCurveNbr + 1;
    }
    canRecord = logger.CanRecord();
#endif

#if defined TESTSTANDESP32V4
    logger.eraseLastThrustCurve();
    long lastThrustCurveNbr = logger.getLastThrustCurveNbr();
    currentThrustCurveNbr = lastThrustCurveNbr + 1;
    fsUsedPercent = logger.getUsedPercent();
    canRecord = logger.isReady();
#endif

  }
  //telemetry on/off
  else if (commandbuffer[0] == 'y')
  {
    if (commandbuffer[1] == '1') {
      SerialCom.print(F("Telemetry enabled\n"));
      telemetryEnable = true;
    }
    else {
      SerialCom.print(F("Telemetry disabled\n"));
      telemetryEnable = false;
    }
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  else if (commandbuffer[0] == 'z')
  {
    updateTestStandName(commandbuffer);
#ifdef TELEMETRY_ESP32
    Serial.print(F("$OK;\n"));
#endif
    SerialCom.print(F("$OK;\n"));
  }
  // empty command
  else if (commandbuffer[0] == ' ')
  {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$KO;\n"));
#endif
    SerialCom.print(F("$K0;\n"));
  }
  else
  {
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
    Serial.print(F("$UNKNOWN;"));
    Serial.println(commandbuffer[0]);
#endif
    SerialCom.print(F("$UNKNOWN;"));
    SerialCom.println(commandbuffer[0]);
  }
}
/*

   re-nitialise all ThrustCurve related global variables

*/
void resetThrustCurve() {

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
  logger.readThrustCurveList();
  long lastThrustCurveNbr = logger.getLastThrustCurveNbr();
  if (lastThrustCurveNbr < 0)
  {
    currentThrustCurveNbr = 0;
    currentMemaddress = 201;
  }
  else
  {
    currentMemaddress = logger.getThrustCurveStop(lastThrustCurveNbr) + 1;
    currentThrustCurveNbr = lastThrustCurveNbr + 1;
  }
  canRecord = logger.CanRecord();
#endif

#if defined TESTSTANDESP32V4
  long lastThrustCurveNbr = logger.getLastThrustCurveNbr();
  currentThrustCurveNbr = lastThrustCurveNbr + 1;
  fsUsedPercent = logger.getUsedPercent();
  canRecord = logger.isReady();
#endif

}

/*
   Check if the battery voltage is OK.
   If not warn the user so that the battery does not get
   damaged by over discharging
*/
void checkBatVoltage(float minVolt) {
#if defined TESTSTANDSTM32
  if ((millis() - lastBattWarning) > 10000) {
    lastBattWarning = millis();
    pinMode(PB1, INPUT_ANALOG);
    int batVoltage = analogRead(PB1);

    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);

    if (bat < minVolt) {
      for (int i = 0; i < 10; i++)
      {
        tone(pinSpeaker, 1600, 1000);
        delay(50);
        noTone(pinSpeaker);
      }
      delay(1000);
    }
  }
#endif

#if defined TESTSTANDSTM32V2 || defined TESTSTANDSTM32V3
  if ((millis() - lastBattWarning) > 10000) {
    lastBattWarning = millis();
    pinMode(PB1, INPUT_ANALOG);
    int batVoltage = analogRead(PB1);

    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);

    if (bat < minVolt) {
      for (int i = 0; i < 10; i++)
      {
        tone(pinSpeaker, 1600, 1000);
        delay(50);
        noTone(pinSpeaker);
      }
      delay(1000);
    }
    // also check the pressure sensor
    // var if 0 PSI of if greater than 100
    if (currPressure == 0 || currPressure > 100) {
      for (int i = 0; i < 10; i++)
      {
        tone(pinSpeaker, 1000, 1000);
        delay(50);
        noTone(pinSpeaker);
      }
      delay(1000);
    }
  }
#endif
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
  if ((millis() - lastBattWarning) > 10000) {
    lastBattWarning = millis();

    // fresh reading, checkBatVoltage() is only called outside of the recording loop
    float bat = readBatteryVoltage();

    if (bat < minVolt) {
      for (int i = 0; i < 10; i++)
      {
        tone(pinSpeaker, 1600, 1000);
        noTone(pinSpeaker);
        delay(50);
      }
      delay(1000);
    }
    // also check the pressure sensor
    // var if 0 PSI of if greater than 100
    if (currPressure == 0 || currPressure > 100) {
      for (int i = 0; i < 10; i++)
      {
        tone(pinSpeaker, 1000, 1000);
        noTone(pinSpeaker);
        delay(50);
      }
      delay(1000);
    }
  }
#endif
}



void SendCalibration(long calibration_offset, long calibration_factor, char *flag) {
  char testStandCalibration[50] = "";

  char temp[10] = "";

  strcat(testStandCalibration, "calibration," );
  sprintf(temp, "%i,", calibration_offset);
  strcat(testStandCalibration, temp);
  sprintf(temp, "%i,", calibration_factor);
  strcat(testStandCalibration, temp);
  strcat(testStandCalibration, flag);
  strcat(testStandCalibration, ",");

  unsigned int chk;
  chk = msgChk(testStandCalibration, sizeof(testStandCalibration));
  sprintf(temp, "%i", chk);
  strcat(testStandCalibration, temp);
  strcat(testStandCalibration, ";\n");
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
  Serial.print("$");
  Serial.print(testStandCalibration);
#endif
  SerialCom.print("$");
  SerialCom.print(testStandCalibration);
}

/*
    Test tram
*/
void sendTestTram() {

  char altiTest[100] = "";
  char temp[10] = "";

  strcat(altiTest, "testTrame," );
  strcat(altiTest, "Bear altimeters are the best!!!!,");
  unsigned int chk;
  chk = msgChk(altiTest, sizeof(altiTest));
  sprintf(temp, "%i", chk);
  strcat(altiTest, temp);
  strcat(altiTest, ";\n");

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
  Serial.print("$");
  Serial.print(altiTest);
#endif
  SerialCom.print("$");
  SerialCom.print(altiTest);

}

int pressureSensorTypeToMaxValue( int type) {
  //"100 PSI", "150 PSI", "200 PSI", "300 PSI", "500 PSI", "1000 PSI", "1600 PSI"
  int maxValue = 100;
  switch (type)
  {
    case 0:
      maxValue = 100;
      break;
    case 1:
      maxValue = 100;
      break;
    case 2:
      maxValue = 150;
      break;
    case 3:
      maxValue = 200;
      break;
    case 4:
      maxValue = 300;
      break;
    case 5:
      maxValue = 500;
      break;
    case 6:
      maxValue = 1000;
      break;
    case 7:
      maxValue = 1600;
      break;
  }

  return maxValue;
}

/*int checkMemoryErrors(int memorySize) {
  int errors = 0;

  return errors;
  }

  int checkMemorySize() {
  int memorySize = 0;

  return memorySize;
  }*/

#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
void updateTestStandName(char *commandbuffer) {
  int i;
  char temp[25];
  char testStandName[21];
  strcpy(temp, commandbuffer);

  for (i = 2; i < strlen(temp); i++)
  {
    if (temp[i] == ',') {
      testStandName[i - 2] = '\0';
      break;
    }
    testStandName[i - 2] = temp[i];
  }

  preferences.begin("namespace", false);
  preferences.putString("Name", testStandName);
  preferences.end();
}
#endif
