/*
  Rocket Motor test stand ver 1.0
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
*/

//Test Stand configuration lib
#include "config.h"
#include <Wire.h> //I2C library

#include "kalman.h"
#include "beepfunc.h"

#include "logger_i2c_eeprom.h"

#include "HX711.h"


// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////

HX711 scale;

//EEProm address
logger_I2C_eeprom logger(0x50) ;
// End address of the 512 eeprom
long endAddress = 65536;
// current file number that you are recording
int currentFileNbr = 0;
// EEPROM start address for the flights. Anything before that is the flight index
long currentMemaddress = 200;
//stop recording a maximum of 20 seconds after main has fired
long recordingTimeOut = 20000;
boolean canRecord = true;
boolean exitRecording = true;
long currentThrustCurveNbr;
long currentThrust;
long currThrust = 0;
long initialThrust;

long thrustDetect;
boolean motorStarted = false;
unsigned long initialTime;
boolean FastReading = false;


//float FEET_IN_METER = 1;

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
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(2280.f);
  scale.tare();
  SerialCom.println("Scale tared");
}

/*
   ReadThrust()
*/
long ReadThrust() {

  return  (long) scale.get_units();
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
    writeConfigStruc();
  }

  // if the baud rate is invalid let's default it
  if (!CheckValideBaudRate(config.connectionSpeed))
  {
    config.connectionSpeed = 38400;
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

  SerialCom.print(F("Start program\n"));
  initTestStand();
  pinMode(pinSpeaker, OUTPUT);

  digitalWrite(pinSpeaker, LOW);

  //initialisation give the version of the testStand
  //One long beep per major number and One short beep per minor revision
  //For example version 1.2 would be one long beep and 2 short beep
  beepTestStandVersion(MAJOR_VERSION, MINOR_VERSION);

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
  //lastAltitude = 0;
  thrustDetect = 10; //config.startRecordThrust;//20;

  int v_ret;
  v_ret = logger.readThrustCurveList();

  long lastThrustCurveNbr = logger.getLastThrustCurveNbr();

  if (lastThrustCurveNbr < 0)
  {
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  else
  {
    currentMemaddress = logger.getThrustCurveStop(lastThrustCurveNbr) + 1;
    currentThrustCurveNbr = lastThrustCurveNbr + 1;
  }

  // check if eeprom is full
  canRecord = logger.CanRecord();
  if (!canRecord)
    SerialCom.println("Cannot record");
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
#else
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

  while (!exitRecording)
  {
    //read current altitude
    currThrust = (ReadThrust() - initialThrust);

    if (( currThrust > config.startRecordThrust) && !recording )
    {
      recording = true;
      SendTelemetry(0, 200);
      // save the time
      initialTime = millis();

      //resetThrustCurve();
      if (canRecord)
      {
        //Save start address
        logger.setThrustCurveStartAddress (currentFileNbr, currentMemaddress);
#ifdef SERIAL_DEBUG
        SerialCom.println(F("Save start address\n"));
#endif
      }

    }
    unsigned long prevTime = 0;
    //long prevAltitude = 0;
    // loop until we have reach a thrustof of x kg
    while (recording)
    {
      unsigned long currentTime;
      unsigned long diffTime;

      currThrust = (ReadThrust() - initialThrust);

      currentTime = millis() - initialTime;

      SendTelemetry(currentTime, 200);
      diffTime = currentTime - prevTime;
      prevTime = currentTime;


      if (canRecord)
      {
        logger.setThrustCurveTimeData( diffTime);
        logger.setThrustCurveData(currThrust);

        if ( (currentMemaddress + logger.getSizeOfThrustCurveData())  > endAddress) {
          //flight is full let's save it
          //save end address
          logger.setThrustCurveEndAddress (currentFileNbr, currentMemaddress - 1);
          canRecord = false;
        } else {
          SerialCom.println("Recording..");
          SerialCom.print(currentMemaddress);
          currentMemaddress = logger.writeFastThrustCurve(currentMemaddress);
          currentMemaddress++;
        }
        delay(50);
      }

      if ((canRecord && (currThrust < 10) ) || ( (millis() - initialTime) > recordingTimeOut))
      {
        //save end address
        logger.setThrustCurveEndAddress (currentThrustCurveNbr, currentMemaddress - 1);
        logger.writeThrustCurveList();
        delay(50);
        SerialCom.print("last: " );
        SerialCom.println(currentMemaddress);
        SerialCom.println(currentThrustCurveNbr);
        exitRecording = true;
        SendTelemetry(millis() - initialTime, 200);
        recording = false;
        SendTelemetry(millis() - initialTime, 200);
        // we have landed telemetry is not required anymore
        telemetryEnable = false;
        //resetThrustCurve();
      }

      /* if ((currThrust < 10)  || ( (millis() - initialTime) > recordingTimeOut))
        {

         recording = false;
         SendTelemetry(millis() - initialTime, 200);
         // we have landed telemetry is not required anymore
         telemetryEnable = false;
        }*/
    } // end while (liftoff)
  } //end while(recording)
}




//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
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
      if (recording)
        SendTelemetry(millis() - initialTime, 200);
      if (!( currThrust > thrustDetect) )
      {
        SendTelemetry(0, 500);
        checkBatVoltage(BAT_MIN_VOLTAGE);
      }
      else
      {
        exitRecording = false;
        //recordThrust();
        SerialCom.println(currThrust);
      }
      long savedTime = millis();
      /*  while (!recording )
        {
          // check if we have anything on the serial port
          if (SerialCom.available())
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
              resetThrustCurve();
              interpretCommandBuffer(commandbuffer);
              SerialCom.print("command1:" );
               SerialCom.println( commandbuffer);
               //commandbuffer[0]='\0';
               i= 0;
            }
          }
        }*/
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
  //SerialCom.print("command2:" );
  //SerialCom.println(commandbuffer);
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
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  //this will read one Thrust curve
  else if (commandbuffer[0] == 'r')
  {
    char temp[3];
    SerialCom.println(F("Read Thrust Curve: "));
    SerialCom.println( commandbuffer[1]);
    SerialCom.println( "\n");
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

      SerialCom.println("StartThrustCurve;" );
      //logger.PrintFlight(atoi(temp));
      logger.printThrustCurveData(atoi(temp));
      SerialCom.println("EndThrustCurve;" );
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
    SerialCom.println(F("Number of ThrustCurve \n"));
    SerialCom.print(F("n;"));
    //Serial.println(getThrustCurveList());
    logger.printThrustCurveList();
  }
  //list all ThrustCurve
  else if (commandbuffer[0] == 'l')
  {
    SerialCom.println(F("ThrustCurve List: \n"));
    logger.printThrustCurveList();
  }
  //toggle continuity on and off
  else if (commandbuffer[0] == 'c')
  {
    if (noContinuity == false)
    {
      noContinuity = true;
      SerialCom.println(F("Continuity off \n"));
    }
    else
    {
      noContinuity = false;
      SerialCom.println(F("Continuity on \n"));
    }
  }
  //get all ThrustCurve data
  else if (commandbuffer[0] == 'a')
  {
    SerialCom.print(F("$start;\n"));

    int i;
    ///todo
    for (i = 0; i < logger.getLastThrustCurveNbr() + 1; i++)
    {
      logger.printThrustCurveData(i);
    }

    SerialCom.print(F("$end;\n"));
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
    writeConfigStruc();
    initTestStand();
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
    currentFileNbr = lastThrustCurveNbr + 1;
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

}