/*
  Rocket Motor test stand ver 1.4
  Copyright Boris du Reau 2012-2024

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

*/

//Test Stand configuration lib
#include "config.h"
#include <Wire.h> //I2C library
#include "kalman.h"
#include "beepfunc.h"
#include "logger_i2c_eeprom.h"
#include "HX711.h"

#ifdef TESTSTANDESP32
BluetoothSerial SerialBT;
#endif


// HX711 circuit wiring
#ifdef TESTSTAND
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;
#endif
#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2
const int LOADCELL_DOUT_PIN = PB15;
const int LOADCELL_SCK_PIN = PB14;
#endif

/*#ifdef TESTSTANDSTM32V2
const int LOADCELL_DOUT_PIN = PB15;
const int LOADCELL_SCK_PIN = PB14;
#endif*/

#ifdef TESTSTANDESP32
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 19;
#endif

//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////

HX711 scale;

//EEProm address
logger_I2C_eeprom logger(0x50) ;
// End address of the 512 eeprom
long endAddress = 65536;
// current file number that you are recording
//int currentFileNbr = 0;
// EEPROM start address for the thrust curve. Anything before that is the flight index
long currentMemaddress = 200;
//stop recording a maximum of 20 seconds after the motor has fired
long recordingTimeOut = 20000;
boolean canRecord = true;
boolean exitRecording = true;
long currentThrustCurveNbr;
long currentThrust;
long currPressure=0;
long currThrust = 0;
long initialThrust;

//long thrustDetect;
boolean motorStarted = false;
unsigned long initialTime;
boolean FastReading = false;
#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2
const int startPin =  PA1;
#endif
/*#ifdef TESTSTANDSTM32V2
const int startPin =  PA1;
#endif*/
#ifdef TESTSTAND
const int startPin =  10;
#endif

#ifdef TESTSTANDESP32
const int startPin =  23;
#endif

int startState = HIGH;
//telemetry
boolean telemetryEnable = false;
long lastTelemetry = 0;
long lastBattWarning = 0;
boolean recording = false;


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
  //  calibration_factor= config.calibration_factor ;
  //  currentOffset = config.current_offset;
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  delay(1000);
  if (config.calibration_factor != 0)
    scale.set_scale((float)config.calibration_factor);
  
  if (config.current_offset != 0)
    scale.set_offset( config.current_offset);
  scale.tare();

  //SerialCom.println(config.calibration_factor);
  //SerialCom.println(config.current_offset);
  SerialCom.println("Scale tared");
}

/*
   ReadThrust()
*/
long ReadThrust() {
  //return  (long) KalmanCalc((abs(scale.get_units()) * 1000));
  //return  (long) (abs(scale.get_units(5)) * 1000);
  return  (long) ((scale.get_units(5)) * 1000);
   
}
//#ifdef TESTSTANDSTM32V2
#if defined(TESTSTANDSTM32) || defined(TESTSTANDSTM32V2)
long ReadPressure() {

  float sum = 0;
  for (int i = 0; i < 20; i++) {
    int pressure = analogRead(PA7);

    sum += (float)map_tofloat( ((float)(pressure * 3300) / (float)4096000 / VOLT_DIVIDER_PRESSURE),
                               0.5,
                               4.5,
                               0.0,
                               (float)pressureSensorTypeToMaxValue(config.pressure_sensor_type));
    delay(1);
  }
  return (long)  (sum / 20.0);
}
#endif
#ifdef TESTSTANDESP32
long ReadPressure() {

  float sum = 0;
  for (int i = 0; i < 20; i++) {
    //int pressure = analogRead(32);
    int pressure = analogReadAdjusted(32);

    sum += (float)map_tofloat( ((float)(pressure * 3300) / (float)4096000 / VOLT_DIVIDER_PRESSURE),
                               0.5,
                               4.5,
                               0.0,
                               (float)pressureSensorTypeToMaxValue(config.pressure_sensor_type));
    delay(1);
  }
  return (long)  (sum / 20.0);
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
    writeConfigStruc();
  }

  // if the baud rate is invalid let's default it
  if (!CheckValideBaudRate(config.connectionSpeed))
  {
    config.connectionSpeed = 38400;
    config.cksum = CheckSumConf(config);
    writeConfigStruc();
  }

#ifdef TESTSTANDSTM32
  analogReadResolution(12); //// need to review !!!!
#endif
#ifdef TESTSTANDSTM32V2
  pinMode(PA7, INPUT_ANALOG);
  analogReadResolution(12); //// need to review !!!!
#endif

#ifdef TESTSTANDESP32
  //pinMode(32, INPUT_ANALOG);
#endif
  // init Kalman filter
  KalmanInit();

  //You can change the baud rate here
  //and change it to 57600, 115200 etc..
  config.connectionSpeed = 38400;
  
#ifdef TESTSTANDESP32
  Serial.begin(38400);
  char standName [15];
  sprintf(standName, "ESP32Stand%i", /*(int)config.altiID*/ 0 );
  Serial.println(standName);
  SerialCom.begin(standName);
#else
 SerialCom.begin(38400);
#endif 
  SerialCom.print("pressure_sensor_type" );
  SerialCom.println(config.pressure_sensor_type);
  SerialCom.print("connectionSpeed" );
  SerialCom.println(config.connectionSpeed);
  
  //  pinMode(A0, INPUT);
#ifdef TESTSTAND
  //software pull up so that all bluetooth modules work!!! took me a good day to figure it out
  pinMode(PD0, INPUT_PULLUP);
#endif

  //software pull up so that all bluetooth modules work!!!
#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2
  pinMode(PB11, INPUT_PULLUP);
#endif
/*#ifdef TESTSTANDSTM32V2
  pinMode(PB11, INPUT_PULLUP);
#endif*/
  
  pinMode(startPin, INPUT_PULLUP);
  SerialCom.print(F("Start program\n"));
  #ifdef TESTSTANDESP32
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
  // let's do some dummy pressure reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
     ReadThrust();
  }

  //let's read the testStand 0
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadThrust();
    delay(50);
  }
  initialThrust = (sum / 10.0);

  SerialCom.print(F("before read list\n"));
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

  // check if eeprom is full
  canRecord = logger.CanRecord();
  if (!canRecord) {
    SerialCom.println("Cannot record");
    #ifdef TESTSTANDESP32
    Serial.println("Cannot record");
    #endif
    beginBeepSeq();
  }
  SerialCom.println("End init");
  #ifdef TESTSTANDESP32
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

#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2
    pinMode(PB1, INPUT_ANALOG);
    int batVoltage = analogRead(PB1);
    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
    dtostrf(bat, 4, 2, temp);
    strcat(testStandTelem, temp);
    strcat(testStandTelem, ",");
#endif
/*#ifdef TESTSTANDSTM32V2
    pinMode(PB1, INPUT_ANALOG);
    int batVoltage = analogRead(PB1);
    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
    dtostrf(bat, 4, 2, temp);
    strcat(testStandTelem, temp);
    strcat(testStandTelem, ",");
#endif*/
#ifdef TESTSTANDESP32
    int batVoltage =analogReadAdjusted(2);
    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
    dtostrf(bat, 4, 2, temp);
    strcat(testStandTelem, temp);
    strcat(testStandTelem, ",");
#endif
#ifdef TESTSTAND
    strcat(testStandTelem, "-1,");
#endif

     if (!recording) {
      sprintf(temp, "%i,", (int)(100 * ((float)logger.getLastThrustCurveEndAddress() / endAddress)) );
    }
    else {
      sprintf(temp, "%i,", (int)(100 * ((float) currentMemaddress / endAddress)) );
    }
    strcat(testStandTelem, temp);
    sprintf(temp, "%i,", logger.getLastThrustCurveNbr() + 1 );
    strcat(testStandTelem, temp);

#ifdef TESTSTANDSTM32V2
    sprintf(temp, "%i,", currPressure );
    strcat(testStandTelem, temp);
#endif
#ifdef TESTSTAND
    strcat(testStandTelem, "-1,");
#endif
#ifdef TESTSTANDSTM32
    strcat(testStandTelem, "-1,");
#endif
    unsigned int chk;
    chk = msgChk(testStandTelem, sizeof(testStandTelem));
    sprintf(temp, "%i", chk);
    strcat(testStandTelem, temp);
    strcat(testStandTelem, ";\n");
    #ifdef TESTSTANDESP32
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
    currThrust = (ReadThrust() - initialThrust);
    currPressure = ReadPressure();

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
        //Save start address
        logger.setThrustCurveStartAddress (currentThrustCurveNbr, currentMemaddress);
        delay(10);
#ifdef SERIAL_DEBUG
        SerialCom.println(F("Save start address\n"));
        SerialCom.println(currentMemaddress);
        SerialCom.println(currentThrustCurveNbr);
#endif
      }

    }
    unsigned long prevTime = 0;

    // loop until we have reach a thrustof of x kg
    while (recording)
    {
      unsigned long currentTime;
      unsigned long diffTime;
     
      currThrust = (ReadThrust() - initialThrust);
      if (currThrust < 0)
        currThrust = 0;
#ifdef TESTSTANDSTM32V2
      currPressure = ReadPressure();
      if (currPressure < 0)
        currPressure = 0;
#endif

#ifdef TESTSTANDESP32
      currPressure = ReadPressure();
      if (currPressure < 0)
        currPressure = 0;
#endif
      currentTime = millis() - initialTime;

      SendTelemetry(currentTime, 200);
      diffTime = currentTime - prevTime;
      prevTime = currentTime;


      if (canRecord)
      {
        logger.setThrustCurveTimeData( diffTime);
        logger.setThrustCurveData(currThrust);
#ifdef TESTSTANDSTM32V2
        logger.setPressureCurveData(currPressure);
#endif

#ifdef TESTSTANDESP32
        logger.setPressureCurveData(currPressure);
#endif

        if ( (currentMemaddress + logger.getSizeOfThrustCurveData())  > endAddress) {
          //memory is full let's save it
          //save end address
          logger.setThrustCurveEndAddress (currentThrustCurveNbr, currentMemaddress - 1);
          canRecord = false;
        } else {
          //SerialCom.println("Recording..");
          //SerialCom.print(currentMemaddress);
          SendTelemetry(millis() - initialTime, 100 );

          if (currThrust < 100000) {
            currentMemaddress = logger.writeFastThrustCurve(currentMemaddress);
            currentMemaddress++;
          }
        }
#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2 || defined TESTSTAND
        if (config.standResolution == 3)
          delay(10);
        else if (config.standResolution == 2)
          delay(20);
        else if (config.standResolution == 1)
          delay(30);
        else if (config.standResolution == 0)
          delay(40);
#endif
/*#ifdef TESTSTAND
        if (config.standResolution == 3)
          delay(10);
        else if (config.standResolution == 2)
          delay(20);
        else if (config.standResolution == 1)
          delay(30);
        else if (config.standResolution == 0)
          delay(40);
#endif
#ifdef TESTSTANDSTM32V2
       if (config.standResolution == 3)
          delay(10);
        else if (config.standResolution == 2)
          delay(20);
        else if (config.standResolution == 1)
          delay(30);
        else if (config.standResolution == 0)
          delay(40);
#endif*/

#ifdef TESTSTANDESP32
       if (config.standResolution == 3)
          delay(10);
        else if (config.standResolution == 2)
          delay(20);
        else if (config.standResolution == 1)
          delay(30);
        else if (config.standResolution == 0)
          delay(40);
#endif
      }

      //if ((canRecord && (currThrust < config.endRecordThrust) ) || ( (millis() - initialTime) > recordingTimeOut))
      if ( ( (millis() - initialTime) > recordingTimeOut))
      {
        //save end address
        logger.setThrustCurveEndAddress (currentThrustCurveNbr, currentMemaddress - 1);
        logger.writeThrustCurveList();
        delay(10);
        /*SerialCom.print("last: " );
          SerialCom.println(currentMemaddress);
          SerialCom.println(currentThrustCurveNbr);*/
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
      currThrust = (ReadThrust() - initialThrust);
      currPressure = ReadPressure();
      
      if (recording)
        SendTelemetry(millis() - initialTime, 200);
      
      startState = digitalRead(startPin);
      
      if (startState == HIGH)
      {
        //Serial.print("HIGH");
        SendTelemetry(0, 500);
        checkBatVoltage(BAT_MIN_VOLTAGE);
      }
      else
      {
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
    /*#ifdef TESTSTANDESP32
    while (Serial.available())
    {
      readVal = Serial.read();
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
    #endif*/
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
    #ifdef TESTSTANDESP32
    Serial.print(F("$start;\n"));
    #endif
    SerialCom.print(F("$start;\n"));
    int i;
    ///todo
    for (i = 0; i < logger.getLastThrustCurveNbr() + 1; i++)
    {
      logger.printThrustCurveData(i);
    }
    #ifdef TESTSTANDESP32
    Serial.print(F("$end;\n"));
    #endif
    SerialCom.print(F("$end;\n"));
  }
  //get Test Stand config
  else if (commandbuffer[0] == 'b')
  {
    #ifdef TESTSTANDESP32
    Serial.print(F("$start;\n"));
    #endif
    SerialCom.print(F("$start;\n"));
    
    printTestStandConfig();
    #ifdef TESTSTANDESP32
    Serial.print(F("$end;\n"));
    #endif
    SerialCom.print(F("$end;\n"));
  }
  //prepare calibrate
  else if (commandbuffer[0] == 'k')
  {
    //remove weight
    scale.tare();
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

    //calibrate(config.calibration_factor, (float)atof(temp));
    //calibrate(0, (float)atof(temp));
    SendCalibration(config.current_offset, (long)config.calibration_factor, "Init");
    scale.calibrate_scale((float)atof(temp), 5);
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
    initTestStand();
  }
  //this will erase all thrust curves
  else if (commandbuffer[0] == 'e')
  {
    SerialCom.println(F("Erase\n"));
    logger.clearThrustCurveList();
    logger.writeThrustCurveList();
    currentThrustCurveNbr = 0;
    currentMemaddress = 201;
  }
  //FastReading
  else if (commandbuffer[0] == 'f')
  {
    FastReading = true;
    #ifdef TESTSTANDESP32
    Serial.print(F("$OK;\n"));
    #endif
    SerialCom.print(F("$OK;\n"));
  }
  //FastReading off
  else if (commandbuffer[0] == 'g')
  {
    FastReading = false;
    #ifdef TESTSTANDESP32
    Serial.print(F("$OK;\n"));
    #endif
    SerialCom.print(F("$OK;\n"));
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
    #ifdef TESTSTANDESP32
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
    scale.tare();
    #ifdef TESTSTANDESP32
    Serial.print(F("$OK;\n"));
    #endif
    SerialCom.print(F("$OK;\n"));
  }
  //list all ThrustCurve
  else if (commandbuffer[0] == 'l')
  {
    SerialCom.println(F("ThrustCurve List: \n"));
    logger.printThrustCurveList();
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
    #ifdef TESTSTANDESP32
    Serial.print(F("$OK;\n"));
    #endif
    SerialCom.print(F("$OK;\n"));
  }
  //Number of ThrustCurve
  else if (commandbuffer[0] == 'n')
  {
    char thrustCurveData[30] = "";
    char temp[9] = "";
    #ifdef TESTSTANDESP32
    Serial.print(F("$start;\n"));
    #endif
    SerialCom.print(F("$start;\n"));
    strcat(thrustCurveData, "nbrOfThrustCurve,");
    sprintf(temp, "%i,", logger.getLastThrustCurveNbr() + 1 );
    strcat(thrustCurveData, temp);
    unsigned int chk = msgChk(thrustCurveData, sizeof(thrustCurveData));
    sprintf(temp, "%i", chk);
    strcat(thrustCurveData, temp);
    strcat(thrustCurveData, ";\n");
    #ifdef TESTSTANDESP32
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
    #ifdef TESTSTANDESP32
    Serial.print(F("$start;\n"));
    #endif
    SerialCom.print(F("$start;\n"));
    sendTestTram();
    #ifdef TESTSTANDESP32
    Serial.print(F("$end;\n"));
    #endif
    SerialCom.print(F("$end;\n"));
  }
  //test stand config param
  //write  config
  else if (commandbuffer[0] == 'p')
  {
    if (writeTestStandConfigV2(commandbuffer)) {
      #ifdef TESTSTANDESP32
      Serial.print(F("$OK;\n"));
      #endif
      SerialCom.print(F("$OK;\n"));
    }
    else {
      #ifdef TESTSTANDESP32
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
    #ifdef TESTSTANDESP32
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
    initTestStand();
    SerialCom.print(F("config reseted\n"));
  }
  // check memory
   else if (commandbuffer[0] == 'u')
  {
    int memSize = logger.checkMemorySize();
    SerialCom.print(F("Memory size: "));
    SerialCom.println(memSize);
    
    long errors = logger.checkMemoryErrors(65500);
    SerialCom.print(F("Nbr of errors: "));
    SerialCom.println(errors);
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
    #ifdef TESTSTANDESP32
      Serial.print(F("$OK;\n"));
    #endif
    SerialCom.print(F("$OK;\n"));
  }
  // empty command
  else if (commandbuffer[0] == ' ')
  {
    #ifdef TESTSTANDESP32
      Serial.print(F("$KO;\n"));
    #endif
    SerialCom.print(F("$K0;\n"));
  }
  else
  {
    #ifdef TESTSTANDESP32
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
  // recording = false;

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
}

/*
   Check if the battery voltage is OK.
   If not warn the user so that the battery does not get
   damaged by over discharging
*/
void checkBatVoltage(float minVolt) {
#ifdef TESTSTANDSTM32
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

#ifdef TESTSTANDSTM32V2
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
    if(currPressure == 0 || currPressure > 100) {
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
#ifdef TESTSTANDESP32
if ((millis() - lastBattWarning) > 10000) {
    lastBattWarning = millis();
   
    double batVoltage =analogReadAdjusted(2);
    
    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);

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
    if(currPressure == 0 || currPressure > 100) {
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

/*
   Calibrate the test stand
*/
/*void calibrate(float calibration_factor , float CALWEIGHT) {
  long currentOffset;
  float LB2KG  = 0.45352;
  
  currentOffset = offset; //scale.get_offset();
  
  SendCalibration(currentOffset, (long)calibration_factor, "Init");
  boolean done = false;
  uint8_t flipDirCount = 0;
  int8_t direction = 1;
  uint8_t dirScale = 100;
  double data = abs(scale.get_units());
  double prevData = data;

  while (!done)
  {
    // get data
    data = abs(scale.get_units());
   
    // if not match
    if (abs(data - CALWEIGHT) >= 0.01)
    {
      if (abs(data - CALWEIGHT) < abs(prevData - CALWEIGHT) && direction != 1 && data < CALWEIGHT)
      {
        direction = 1;
        flipDirCount++;
      }
      else if (abs(data - CALWEIGHT) >= abs(prevData - CALWEIGHT) && direction != -1 && data > CALWEIGHT)
      {
        direction = -1;
        flipDirCount++;
      }

      if (flipDirCount > 2)
      {
        if (dirScale != 1)
        {
          dirScale = dirScale / 10;
          flipDirCount = 0;
        }
      }
      // set new factor
      calibration_factor += direction * dirScale;

      scale.set_scale(calibration_factor);
      SendCalibration(currentOffset, (long)calibration_factor, "In progress");
      //short delay
      delay(5);
      // keep old data
      prevData = data;
    }
    // if match
    else
    {
      //Serial.println("NEW currentOffset = " + String(currentOffset));
      //Serial.println("NEW calibration_factor = " + String(calibration_factor));
      config.calibration_factor = (long) calibration_factor;
      config.current_offset = currentOffset;
      SendCalibration(currentOffset, (long)calibration_factor, "Done");
      //writeConfigStruc();
      done = true;

    }

  } // end while

  scale.set_offset(currentOffset);
  scale.set_scale((float)config.calibration_factor);
  SendCalibration(currentOffset, (long)calibration_factor, "Done");

}*/

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
  #ifdef TESTSTANDESP32
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

  #ifdef TESTSTANDESP32
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

#ifdef TESTSTANDESP32
double analogReadAdjusted(byte pinNumber){

  // Specify the adjustment factors.
  const double f1 = 1.7111361460487501e+001;
  const double f2 = 4.2319467860421662e+000;
  const double f3 = -1.9077375643188468e-002;
  const double f4 = 5.4338055402459246e-005;
  const double f5 = -8.7712931081088873e-008;
  const double f6 = 8.7526709101221588e-011;
  const double f7 = -5.6536248553232152e-014;
  const double f8 = 2.4073049082147032e-017;
  const double f9 = -6.7106284580950781e-021;
  const double f10 = 1.1781963823253708e-024;
  const double f11 = -1.1818752813719799e-028;
  const double f12 = 5.1642864552256602e-033;

  // Specify the number of loops for one measurement.
  const int loops = 40;

  // Specify the delay between the loops.
  const int loopDelay = 1;

  // Initialize the used variables.
  int counter = 1;
  int inputValue = 0;
  double totalInputValue = 0;
  double averageInputValue = 0;

  // Loop to get the average of different analog values.
  for (counter = 1; counter <= loops; counter++) {

    // Read the analog value.
    inputValue = analogRead(pinNumber);

    // Add the analog value to the total.
    totalInputValue += inputValue;

    // Wait some time after each loop.
    delay(loopDelay);
  }

  // Calculate the average input value.
  averageInputValue = totalInputValue / loops;

  // Calculate and return the adjusted input value.
  return f1 + f2 * pow(averageInputValue, 1) + f3 * pow(averageInputValue, 2) + f4 * pow(averageInputValue, 3) + f5 * pow(averageInputValue, 4) + f6 * pow(averageInputValue, 5) + f7 * pow(averageInputValue, 6) + f8 * pow(averageInputValue, 7) + f9 * pow(averageInputValue, 8) + f10 * pow(averageInputValue, 9) + f11 * pow(averageInputValue, 10) + f12 * pow(averageInputValue, 11);
}
#endif
/*int checkMemoryErrors(int memorySize) {
  int errors = 0;
  
  return errors;
}

int checkMemorySize() {
  int memorySize = 0;

  return memorySize;
}*/
