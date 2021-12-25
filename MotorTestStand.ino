/*
  Rocket Motor test stand ver 1.1
  Copyright Boris du Reau 2012-2021

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
  
*/

//Test Stand configuration lib
#include "config.h"
#include <Wire.h> //I2C library

#include "kalman.h"
#include "beepfunc.h"

#include "logger_i2c_eeprom.h"

#include "HX711.h"


// HX711 circuit wiring
#ifdef TESTSTAND
const int LOADCELL_DOUT_PIN = 3;// 2;
const int LOADCELL_SCK_PIN = 2; //3;
#endif
#ifdef TESTSTANDSTM32 
const int LOADCELL_DOUT_PIN = PB15;// 2;
const int LOADCELL_SCK_PIN = PB14; //3;
#endif

#ifdef TESTSTANDSTM32V2
const int LOADCELL_DOUT_PIN = PB15;// 2;
const int LOADCELL_SCK_PIN = PB14; //3;
#endif
/*#define DOUT  PB15
  #define CLK  PB14*/
//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////

HX711 scale;
//#define LB2KG  0.45352
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
long currPressure;
long currThrust = 0;
long initialThrust;

long thrustDetect;
boolean motorStarted = false;
unsigned long initialTime;
boolean FastReading = false;
#ifdef TESTSTANDSTM32
const int startPin =  PA1;
#endif
#ifdef TESTSTANDSTM32V2
const int startPin =  PA1;
#endif
#ifdef TESTSTAND
const int startPin =  10;
#endif
int startState = HIGH;
//telemetry
boolean telemetryEnable = false;
long lastTelemetry = 0;
long lastBattWarning = 0;
boolean recording = false;

void MainMenu();


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
  //scale.set_scale(2280.f);
  //float LB2KG  = 0.45352;
  if (config.calibration_factor == 0)
    config.calibration_factor = -29566;//-33666;
  if(config.current_offset == 0)
    config.current_offset = 606978; //-64591;
  scale.tare();  
  config.current_offset = scale.get_offset();
  //scale.set_scale((float)config.calibration_factor / LB2KG);
  scale.set_scale((float)config.calibration_factor );
  //scale.set_scale(2280.f);
  
  
  // scale.set_offset( config.current_offset);
  SerialCom.println(config.calibration_factor);
  SerialCom.println(config.current_offset);
  SerialCom.println("Scale tared");
}

/*
   ReadThrust()
*/
long ReadThrust() {

  return  (long) (abs(scale.get_units()) * 1000);
}

/*
   setup()
   Initialise the test Stand

*/
void setup()
{
  int val = 0;     // variable to store the read value
  int val1 = 0;     // variable to store the read value
  // initialise the connection
  Wire.begin();
  //Serial.begin(38400);
  //Serial.print(F("Start program\n"));
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

  // init Kalman filter
  KalmanInit();

  //You can change the baud rate here
  //and change it to 57600, 115200 etc..
  config.connectionSpeed = 38400;
  //SerialCom.begin(config.connectionSpeed);
  SerialCom.begin(38400);


  //  pinMode(A0, INPUT);
#ifdef TESTSTAND
  //software pull up so that all bluetooth modules work!!! took me a good day to figure it out
  pinMode(PD0, INPUT_PULLUP);
#endif

  //software pull up so that all bluetooth modules work!!!
#ifdef TESTSTANDSTM32
  pinMode(PB11, INPUT_PULLUP);
#endif
#ifdef TESTSTANDSTM32V2
  pinMode(PB11, INPUT_PULLUP);
#endif
  pinMode(startPin, INPUT);
  SerialCom.print(F("Start program\n"));
  initTestStand();
  pinMode(pinSpeaker, OUTPUT);


  digitalWrite(pinSpeaker, LOW);

  //initialisation give the version of the testStand
  //One long beep per major number and One short beep per minor revision
  //For example version 1.2 would be one long beep and 2 short beep
  beepTestStandVersion(MAJOR_VERSION, MINOR_VERSION);
  SerialCom.print("befor calman\n");
  // let's do some dummy altitude reading
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

  thrustDetect = config.startRecordThrust; //20;

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
    beginBeepSeq();
  }
  SerialCom.println("End init");
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

#ifdef TESTSTANDSTM32
    pinMode(PB1, INPUT_ANALOG);
    int batVoltage = analogRead(PB1);
    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
    //sprintf(temp, "%f,", bat);
    dtostrf(bat, 4, 2, temp);
    strcat(testStandTelem, temp);
    strcat(testStandTelem, ",");
#endif 
#ifdef TESTSTANDSTM32V2
    pinMode(PB1, INPUT_ANALOG);
    int batVoltage = analogRead(PB1);
    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
    //sprintf(temp, "%f,", bat);
    dtostrf(bat, 4, 2, temp);
    strcat(testStandTelem, temp);
    strcat(testStandTelem, ",");
#endif    
#ifdef TESTSTAND
    strcat(testStandTelem, "-1,");
#endif

    sprintf(temp, "%i,", (int)(100 * ((float)logger.getLastThrustCurveEndAddress() / endAddress)) );
    strcat(testStandTelem, temp);
    sprintf(temp, "%i,", logger.getLastThrustCurveNbr() + 1 );
    strcat(testStandTelem, temp);

    unsigned int chk;
    chk = msgChk(testStandTelem, sizeof(testStandTelem));
    sprintf(temp, "%i", chk);
    strcat(testStandTelem, temp);
    strcat(testStandTelem, ";\n");
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
  recordingTimeOut = config.endRecordTime*1000;
  while (!exitRecording)
  {
    //read current thrust
    currThrust = (ReadThrust() - initialThrust);

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
#ifdef SERIAL_DEBUG
        SerialCom.println(F("Save start address\n"));
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
      #ifdef TESTSTANDSTM32V2
      pinMode(PB0, INPUT_ANALOG);
      int pressure = analogRead(PB0);

      currPressure =(long) ( VOLT_DIVIDER * ((float)(pressure * 3300) / (float)4096000));
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

        if ( (currentMemaddress + logger.getSizeOfThrustCurveData())  > endAddress) {
          //flight is full let's save it
          //save end address
          logger.setThrustCurveEndAddress (currentThrustCurveNbr, currentMemaddress - 1);
          canRecord = false;
        } else {
          //SerialCom.println("Recording..");
          //SerialCom.print(currentMemaddress);
          SendTelemetry(millis() - initialTime, 100);
          currentMemaddress = logger.writeFastThrustCurve(currentMemaddress);
          currentMemaddress++;
        }
        
        if (config.standResolution ==3)
          delay(10);
        else if (config.standResolution ==2)
          delay(20);
        else if (config.standResolution ==1)
          delay(10);  
        else if (config.standResolution ==0)
          delay(40);    
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
        telemetryEnable = false;
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
      //SerialCom.println(currThrust);
      if (recording)
        SendTelemetry(millis() - initialTime, 200);
      //= digitalRead(buttonPin);
      startState = digitalRead(startPin);
      
      if (startState == HIGH)
      {
        
        SendTelemetry(0, 500);
        checkBatVoltage(BAT_MIN_VOLTAGE);
      }
      else
      {
        exitRecording = false;
        recordThrust();
      }
      long savedTime = millis();

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
   e  erase all saved flights
   r  followed by a number which is the flight number.
      This will retrieve all data for the specified flight
   w  Start or stop recording
   n  Return the number of recorded flights in the EEprom
   l  list all flights
   c  toggle continuity on and off
   a  get all flight data
   b  get altimeter config
   s  write altimeter config
   d  reset alti config
   t  reset alti config (why?)
   f  FastReading on
   g  FastReading off
   h  hello. Does not do much
   i  unused
   k  folowed by a number turn on or off the selected output
   y  followed by a number turn telemetry on/off. if number is 1 then
      telemetry in on else turn it off
   m  followed by a number turn main loop on/off. if number is 1 then
      main loop in on else turn it off
   x  delete last curve
   j  tare the testStand
*/
void interpretCommandBuffer(char *commandbuffer) {
  SerialCom.println((char*)commandbuffer);
  //this will erase all flight
  if (commandbuffer[0] == 'e')
  {
    SerialCom.println(F("Erase\n"));
    //i2c_eeprom_erase_fileList();
    logger.clearThrustCurveList();
    logger.writeThrustCurveList();
    currentThrustCurveNbr = 0;
    currentMemaddress = 201;
  }
  //this will read one Thrust curve
  else if (commandbuffer[0] == 'r')
  {
    char temp[3];
    //SerialCom.println(F("Read Thrust Curve: "));
    //SerialCom.println( commandbuffer[1]);
    //SerialCom.println( "\n");
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
      //SerialCom.println("StartThrustCurve;" );
      //logger.PrintFlight(atoi(temp));
      logger.printThrustCurveData(atoi(temp));
      //SerialCom.println("EndThrustCurve;" );
      SerialCom.print(F("$end;\n"));
    }
    else
      SerialCom.println(F("not a valid ThrustCurve"));
  }
  // Recording
  else if (commandbuffer[0] == 'w')
  {
    SerialCom.println(F("Recording \n"));
    recordThrust();
  }
  //Number of ThrustCurve
  else if (commandbuffer[0] == 'n')
  {
    /*SerialCom.println(F("Number of ThrustCurve \n"));
    SerialCom.print(F("n;"));
    //Serial.println(getThrustCurveList());
    logger.printThrustCurveList();*/
    //
    char thrustCurveData[30] = "";
    char temp[9] = "";
    SerialCom.print(F("$start;\n"));
    strcat(thrustCurveData, "nbrOfThrustCurve,");
    sprintf(temp, "%i,", logger.getLastThrustCurveNbr()+1 );
    strcat(thrustCurveData, temp);
    unsigned int chk = msgChk(thrustCurveData, sizeof(thrustCurveData));
    sprintf(temp, "%i", chk);
    strcat(thrustCurveData, temp);
    strcat(thrustCurveData, ";\n");
    SerialCom.print("$");
    SerialCom.print(thrustCurveData);
    SerialCom.print(F("$end;\n"));
  }
  //list all ThrustCurve
  else if (commandbuffer[0] == 'l')
  {
    SerialCom.println(F("ThrustCurve List: \n"));
    logger.printThrustCurveList();
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
    //SerialCom.println(temp);
    //calibrate(float calibration_factor , float CALWEIGHT)
    //calibrate(/*-7050*/ -29566, (float)atof(temp));
    calibrate(config.calibration_factor, (float)atof(temp));
    config.cksum = CheckSumConf(config);
    writeConfigStruc();
    SerialCom.print(F("$OK;\n"));

    /* if (noContinuity == false)
      {
       noContinuity = true;
       SerialCom.println(F("Continuity off \n"));
      }
      else
      {
       noContinuity = false;
       SerialCom.println(F("Continuity on \n"));
      }*/
  }
  //get all ThrustCurve data
  else if (commandbuffer[0] == 'a')
  {
//    commandCancelled = false;
    SerialCom.print(F("$start;\n"));

    int i;
    ///todo
    for (i = 0; i < logger.getLastThrustCurveNbr() + 1; i++)
    {
      //if(commandCancelled) 
      //break;
      logger.printThrustCurveData(i);
    }

    SerialCom.print(F("$end;\n"));
    //commandCancelled = false;
  }
  //get Test Stand config
  else if (commandbuffer[0] == 'b')
  {
    SerialCom.print(F("$start;\n"));

    printTestStandConfig();

    SerialCom.print(F("$end;\n"));
  }
  //write test stand config
  else if (commandbuffer[0] == 's')
  {
    if (writeTestStandConfig(commandbuffer)) {

      SerialCom.print(F("$OK;\n"));
      readTestStandConfig();
      initTestStand();
    }
    else {
      SerialCom.print(F("$KO;\n"));
    }
  }
  //reset test stand config this is equal to t why do I have 2 !!!!
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    //config.cksum = CheckSumConf(config);
    writeConfigStruc();
    initTestStand();
  }
  //reset config and set it to default
  else if (commandbuffer[0] == 't')
  {
    //reset config
    defaultConfig();
    //config.cksum = CheckSumConf(config);
    writeConfigStruc();
    initTestStand();
    SerialCom.print(F("config reseted\n"));
  }
  //FastReading
  else if (commandbuffer[0] == 'f')
  {
    FastReading = true;
    SerialCom.print(F("$OK;\n"));

  }
  //FastReading off
  else if (commandbuffer[0] == 'g')
  {
    FastReading = false;
    SerialCom.print(F("$OK;\n"));
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
    SerialCom.print(F("$OK;\n"));
  }
  // unused
  else if (commandbuffer[0] == 'i')
  {
    //exit continuity mode
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
    SerialCom.print(F("$OK;\n"));
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
    SerialCom.print(F("$OK;\n"));
  }
  //delete last curve
  else if (commandbuffer[0] == 'x')
  {
    logger.eraseLastThrustCurve();
  }
   //tare testStand
  else if (commandbuffer[0] == 'j')
  {
    scale.tare();
    SerialCom.print(F("$OK;\n"));
  }
  // send test tram
  else if (commandbuffer[0] == 'o')
  { 
    SerialCom.print(F("$start;\n"));
    sendTestTram();
    SerialCom.print(F("$end;\n"));
  }
  // empty command
  else if (commandbuffer[0] == ' ')
  {
    SerialCom.print(F("$K0;\n"));
  }
  else
  {
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
  }
#endif

}

/*
 * Calibrate the test stand
 */
void calibrate(float calibration_factor , float CALWEIGHT) {
  long currentOffset;
  float LB2KG  = 0.45352;
  //SerialCom.println("Calibrate");
  //SerialCom.println(CALWEIGHT);
  //scale.set_scale(calibration_factor / LB2KG);
  currentOffset = scale.get_offset();
  scale.set_scale(calibration_factor );
  // set tare and save value
  //scale.tare();
  
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
    //SerialCom.println("data = " + String(data, 2));
    //Serial.println("abs = " + String(abs(data - CALWEIGHT), 4));
    //SerialCom.println("calibration_factor = " + String(calibration_factor));
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
          //Serial.println("dirScale = " + String(dirScale));
        }
      }
      // set new factor
      calibration_factor += direction * dirScale;
      //scale.set_scale(calibration_factor / LB2KG);
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
  SerialCom.print("$");
  SerialCom.print(altiTest);

}
