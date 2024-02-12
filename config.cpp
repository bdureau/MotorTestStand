#include "config.h"


ConfigStruct config;
//================================================================
// read and write in the microcontroler eeprom
//================================================================
void defaultConfig()
{
  config.unit = 0;
  config.connectionSpeed = 38400;
  config.endRecordTime = 10; // stop recording 
  config.standResolution = 3; //0 to 3 ie: from low resolution to high
  config.eepromSize = 512;
  config.startRecordThrust = 10;
  config.batteryType = 1;
  config.calibration_factor = 0; 
  config.current_offset = 0;
  config.pressure_sensor_type = 6; //"100 PSI", "150 PSI", "200 PSI", "300 PSI", "500 PSI", "1000 PSI", "1600 PSI"
  config.telemetryType = 0;
  #if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3
  config.pressure_sensor_type2 = 6;
  #endif
  config.cksum = CheckSumConf(config);
}

bool readTestStandConfig() {
  //set the config to default values so that if any have not been configured we can use the default ones
  defaultConfig();
  int i;
  #if defined TESTSTANDESP32 || defined TESTSTANDESP32V3
  EEPROM.begin(512);
  #endif
  for ( i = 0; i < sizeof(config); i++ ) {
    *((char*)&config + i) = EEPROM.read(CONFIG_START + i);
  }
  #if defined TESTSTANDESP32 || defined TESTSTANDESP32V3
  EEPROM.end();
  #endif
  if ( config.cksum != CheckSumConf(config) ) {
    return false;
  }
  return true;
}


/*
  write the config received by the console

*/
/*bool writeTestStandConfig( char *p ) {

  char *str;
  int i = 0;
  int strChk=0;
  char msg[100]="";
  
  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {

    switch (i)
    {
      case 1:
        config.unit = atoi(str);
        strcat(msg, str);
        break;
      case 2:   
        config.connectionSpeed = atol(str);
        strcat(msg, str);
        break;
      case 3:
        config.endRecordTime = atoi(str);
        strcat(msg, str);
        break;
      case 4:
        config.standResolution  = atoi(str);
        strcat(msg, str);
        break;
      case 5:
        config.eepromSize = atoi(str);
        strcat(msg, str);
        break;
      case 6:
        config.startRecordThrust = atoi(str);
        strcat(msg, str);
        break;
      case 7:
        config.batteryType = atoi(str);
        strcat(msg, str);
        break;
      case 8:
        config.calibration_factor = atoi(str);
        strcat(msg, str);
        break;
      case 9:
        config.current_offset = atoi(str);
        strcat(msg, str);
        break;
      case 10:
        config.pressure_sensor_type = atoi(str);
        strcat(msg, str);
        break;
      case 11:
        config.telemetryType = atoi(str);
        strcat(msg, str);
        break;
      #if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3    
      case 12:
        config.pressure_sensor_type2 = atoi(str);
        strcat(msg, str);
        break;
      case 13:
        //our checksum
        strChk= atoi(str);
        break;
      #else
      case 12:
        //our checksum
        strChk= atoi(str);
        break;
      #endif
    }
    i++;

  }
  //we have a partial config
  #if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3 
  if (i<12)
    return false;
  #else
  if (i<11)
    return false;
  #endif
  if(msgChk(msg, sizeof(msg)) != strChk)
    return false;  
  // add checksum
  config.cksum = CheckSumConf(config);

  writeConfigStruc();
  return true;
}
*/
bool writeTestStandConfigV2( char *p ) {

  char *str;
  int i = 0;
  int command =0;
  long commandVal =0;
  int strChk = 0;
  char msg[100] = "";

  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    //SerialCom.println(str);
    if (i == 1) {
      command = atoi(str);
      strcat(msg, str);
    }
    if (i == 2) {
      commandVal =  atol(str);
      strcat(msg, str);
    }
    if (i == 3) {
      strChk  =  atoi(str);  
    }
    i++;

  }
    //we have a partial config
  if (i < 4)
    return false;
  //checksum is ivalid ? 
  if (msgChk(msg, sizeof(msg)) != strChk)
    return false;  
    
  switch (command)
    {
      case 1:
        config.unit = (int) commandVal;
        break;
      case 2:
        config.connectionSpeed = commandVal;
        break;
      case 3:
        config.endRecordTime = (int) commandVal;
        break;
      case 4:
        config.standResolution = (int) commandVal;
        break;
      case 5:
        config.eepromSize = (int) commandVal;
        break;
      case 6:
        config.startRecordThrust = (int) commandVal;
        break;
      case 7:
        config.batteryType = (int)commandVal;
        break;
      case 8:
        config.calibration_factor = (int)commandVal;
        break;
      case 9:
        config.current_offset = (int)commandVal;
        break;
      case 10:
        config.pressure_sensor_type = (int)commandVal;
        break;
      case 11:
        config.telemetryType = (int)commandVal;
        break;    
      #if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3
      case 12:
        config.pressure_sensor_type2 = (int)commandVal;
        break;
      #endif
    }

  // add checksum
  config.cksum = CheckSumConf(config);

  return true;
}
/*

   Write config structure to the EEPROM

*/
void writeConfigStruc()
{
  int i;
  #if defined TESTSTANDESP32 || defined TESTSTANDESP32V3
  EEPROM.begin(512);
  #endif
  for ( i = 0; i < sizeof(config); i++ ) {
    EEPROM.write(CONFIG_START + i, *((char*)&config + i));
  }
  #if defined TESTSTANDESP32 || defined TESTSTANDESP32V3
  EEPROM.commit();
  EEPROM.end();
  #endif
}
/*

   Print test stand config to the Serial line

*/
void printTestStandConfig()
{
  
  char testStandConfig[120] = "";
  char temp[10] = "";
  bool ret = readTestStandConfig();
  if (!ret)
    SerialCom.print(F("invalid conf"));

  strcat(testStandConfig, "teststandconfig,");
  
  //Unit
  sprintf(temp, "%i,", config.unit);
  strcat(testStandConfig, temp);
  //test StandName
  strcat(testStandConfig, BOARD_FIRMWARE);
  strcat(testStandConfig,",");
  //alti major version
  sprintf(temp, "%i,", MAJOR_VERSION);
  strcat(testStandConfig, temp);
  //alti minor version
  sprintf(temp, "%i,", MINOR_VERSION);
  strcat(testStandConfig, temp);
  
  sprintf(temp, "%lu,", config.connectionSpeed);
  strcat(testStandConfig, temp);

  //startRecordThrust
  sprintf(temp, "%i,", config.startRecordThrust);
  strcat(testStandConfig, temp);
  
  sprintf(temp, "%i,", config.endRecordTime);
  strcat(testStandConfig, temp);
  
  sprintf(temp, "%i,", config.standResolution);
  strcat(testStandConfig, temp);
 
  sprintf(temp, "%i,", config.eepromSize);
  strcat(testStandConfig, temp);
  
  //Battery type
  sprintf(temp, "%i,", config.batteryType);
  strcat(testStandConfig, temp);

  sprintf(temp, "%i,",config.calibration_factor);
  strcat(testStandConfig, temp);
  sprintf(temp, "%i,",config.current_offset);
  strcat(testStandConfig, temp);

  sprintf(temp, "%i,",config.pressure_sensor_type);
  strcat(testStandConfig, temp);

  sprintf(temp, "%i,",config.telemetryType);
  strcat(testStandConfig, temp);

  #if defined TESTSTANDSTM32V3 || defined TESTSTANDESP32V3
  sprintf(temp, "%i,",config.pressure_sensor_type2);
  strcat(testStandConfig, temp);
  #endif
  unsigned int chk = 0;
  chk = msgChk( testStandConfig, sizeof(testStandConfig) );
  sprintf(temp, "%i;\n", chk);
  strcat(testStandConfig, temp);

  #if defined TESTSTANDESP32 || defined TESTSTANDESP32V3
  Serial.print("$");
  Serial.print(testStandConfig);
  #endif
  SerialCom.print("$");
  //SerialCom.print(testStandConfig);
  // the following will slow down the connection speed so that it works better with telemetry module
  // or bluetooth module with no buffer
  /*if (config.telemetryType == 1){
  for(int j=0; j< sizeof(testStandConfig);j++) {
    SerialCom.print(testStandConfig[j]);
    delay(2);
  }
  } 
  else*/
    SerialCom.print(testStandConfig);
  

}
bool CheckValideBaudRate(long baudRate)
{
  bool valid = false;
  if (baudRate == 300 ||
      baudRate == 1200 ||
      baudRate == 2400 ||
      baudRate == 4800 ||
      baudRate == 9600 ||
      baudRate == 14400 ||
      baudRate == 19200 ||
      baudRate == 28800 ||
      baudRate == 38400 ||
      baudRate == 57600 ||
      baudRate == 115200 ||
      baudRate == 230400)
    valid = true;
  return valid;
}
long checkEEPromEndAdress(int eepromSize)
{
  /*long endAdress=0;
    switch(eepromSize)
    {
    case 64:
  	endAdress =16384;
  	break;
    case 128:
  	endAdress =16384;
  	break;
    case 256:
  	endAdress =32768;
  	break;
    case 512:
  	endAdress =65536;
  	break;
    case 1024:
  	endAdress =131072;
  	break;
    }*/
  return eepromSize * 128;
}
/*
   Calculate Checksum for the config
*/
unsigned int CheckSumConf( ConfigStruct cnf)
{
  int i;
  unsigned int chk = 0;

  for (i = 0; i < (sizeof(cnf) - sizeof(int)); i++)
    chk += *((char*)&cnf + i);

  return chk;
}
unsigned int msgChk( char * buffer, long length ) {

  long index;
  unsigned int checksum;

  for ( index = 0L, checksum = 0; index < length; checksum += (unsigned int) buffer[index++] );
  return (unsigned int) ( checksum % 256 );

}
