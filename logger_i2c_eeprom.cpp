#include "logger_i2c_eeprom.h"
#include "IC2extEEPROM.h"
extEEPROM eep(kbits_512, 1, 64);
logger_I2C_eeprom::logger_I2C_eeprom(uint8_t deviceAddress)
{
}

void logger_I2C_eeprom::begin()
{
  Wire.begin();
}
/*
   clearThrustCurveList()
   Clear the Thrust Curve list. Rather than clearing the entire eeprom
   let's just reset addresses 0 to 200 which contains the Thrust Curve addresses

*/
void logger_I2C_eeprom::clearThrustCurveList()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    _ThrustCurveConfig[i].ThrustCurve_start = 0;
    _ThrustCurveConfig[i].ThrustCurve_stop = 0;
  }
}


/*
   readThrustCurveList()

*/
int logger_I2C_eeprom::readThrustCurveList()
{
  eep.read(0, ((byte*)&_ThrustCurveConfig), sizeof(_ThrustCurveConfig));
  return THRUSTCURVE_LIST_START + sizeof(_ThrustCurveConfig) ;
}
/*
   readThrustCurve(int eeaddress)

*/
unsigned long logger_I2C_eeprom::readThrustCurve(unsigned long eeaddress)
{
  eep.read(eeaddress, ((byte*)&_ThrustCurveData), sizeof(_ThrustCurveData));
  return eeaddress + sizeof(_ThrustCurveData);
}

/*
   writeThrustCurveList()

*/
int logger_I2C_eeprom::writeThrustCurveList()
{
  eep.write(THRUSTCURVE_LIST_START, ((byte*)&_ThrustCurveConfig), sizeof(_ThrustCurveConfig));
  return THRUSTCURVE_LIST_START + sizeof(_ThrustCurveConfig);
}

/*
   writeFastThrustCurve(int eeaddress)

*/
unsigned long logger_I2C_eeprom::writeFastThrustCurve(unsigned long eeaddress)
{
  eep.write(eeaddress, ((byte*)&_ThrustCurveData), sizeof(_ThrustCurveData));
  return eeaddress + sizeof(_ThrustCurveData);
}

/*

   getLastThrustCurveNbr()
   Parse the Thrust curves index end check if the ThrustCurve_start address is > 0
   return -1 if no thrust curves have been recorded else return the thrust curve number

*/
int logger_I2C_eeprom::getLastThrustCurveNbr()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_ThrustCurveConfig[i].ThrustCurve_start == 0)
    {
      break;
    }
  }
  i--;
  return i;
}

/*

 eraseLastThrustCurve()
 
 */
bool logger_I2C_eeprom::eraseLastThrustCurve(){
int i;
  for (i = 0; i < 25; i++)
  {
    if (_ThrustCurveConfig[i].ThrustCurve_start == 0)
    {
      if(i>0) {
        _ThrustCurveConfig[i-1].ThrustCurve_start = 0;
        _ThrustCurveConfig[i-1].ThrustCurve_stop = 0;
        writeThrustCurveList();
        return true;
      }
    }
  }
  return false;
}

/*

   getLastThrustCurveEndAddress()
   Parse the ThrustCurve index end check if the ThrustCurve_start address is > 0
   return -1 if no ThrustCurve have been recorded else return the ThrustCurve number

*/
long logger_I2C_eeprom::getLastThrustCurveEndAddress()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_ThrustCurveConfig[i].ThrustCurve_start == 0)
    {
      break;
    }
  }
  i--;
  return _ThrustCurveConfig[i].ThrustCurve_stop;
}

/*

   printThrustCurveList()


*/
int logger_I2C_eeprom::printThrustCurveList()
{
  //retrieve from the eeprom
  int v_ret =  readThrustCurveList();

  //Read the stucture
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_ThrustCurveConfig[i].ThrustCurve_start == 0)
      break;
    #ifdef TESTSTANDESP32
    Serial.print("ThrustCurve Nbr: ");
    Serial.println(i);
    Serial.print("Start: ");
    Serial.println(_ThrustCurveConfig[i].ThrustCurve_start);
    Serial.print("End: ");
    Serial.println(_ThrustCurveConfig[i].ThrustCurve_stop);
    #endif
    SerialCom.print("ThrustCurve Nbr: ");
    SerialCom.println(i);
    SerialCom.print("Start: ");
    SerialCom.println(_ThrustCurveConfig[i].ThrustCurve_start);
    SerialCom.print("End: ");
    SerialCom.println(_ThrustCurveConfig[i].ThrustCurve_stop);
  }
  return i;
}

void logger_I2C_eeprom::setThrustCurveStartAddress(int ThrustCurveNbr, long startAddress)
{
  _ThrustCurveConfig[ThrustCurveNbr].ThrustCurve_start = startAddress;
}

void logger_I2C_eeprom::setThrustCurveEndAddress(int ThrustCurveNbr, long endAddress)
{
  _ThrustCurveConfig[ThrustCurveNbr].ThrustCurve_stop = endAddress;
}

void logger_I2C_eeprom::setThrustCurveTimeData( long difftime)
{
  _ThrustCurveData.diffTime = difftime;
}
void logger_I2C_eeprom::setThrustCurveData( long thrust)
{
  _ThrustCurveData.thrust = thrust;
}

long logger_I2C_eeprom::getThrustCurveStart(int ThrustCurveNbr)
{
  return  _ThrustCurveConfig[ThrustCurveNbr].ThrustCurve_start;
}
long logger_I2C_eeprom::getThrustCurveStop(int ThrustCurveNbr)
{
  return  _ThrustCurveConfig[ThrustCurveNbr].ThrustCurve_stop;
}
long logger_I2C_eeprom::getThrustCurveTimeData()
{
  return _ThrustCurveData.diffTime;
}
long logger_I2C_eeprom::getThrustCurveData()
{
  return _ThrustCurveData.thrust;
}
#if defined TESTSTANDSTM32V2 || defined TESTSTANDESP32
long logger_I2C_eeprom::getPressureCurveData()
{
  return _ThrustCurveData.casing_pressure;
}
void logger_I2C_eeprom::setPressureCurveData( long pressure)
{
  _ThrustCurveData.casing_pressure = pressure;
}
#endif

long logger_I2C_eeprom::getSizeOfThrustCurveData()
{
  return sizeof(_ThrustCurveData);
}

void logger_I2C_eeprom::printThrustCurveData(int ThrustCurveNbr)
{
  unsigned long startaddress;
  unsigned long endaddress;

  startaddress = getThrustCurveStart(ThrustCurveNbr);
  endaddress = getThrustCurveStop(ThrustCurveNbr);

  if (startaddress > 200)
  {
    unsigned long i = startaddress;
    unsigned long currentTime = 0;

    while (i < (endaddress + 1))
    {
      i = readThrustCurve(i) + 1;
      char ThrustCurveData[120] = "";
      char temp[20] = "";
      currentTime = currentTime + getThrustCurveTimeData();
      strcat(ThrustCurveData, "data,");
      sprintf(temp, "%i,", ThrustCurveNbr );
      strcat(ThrustCurveData, temp);
      sprintf(temp, "%i,", (int) currentTime );
      //sprintf(temp, "%lu,", currentTime );
      strcat(ThrustCurveData, temp);
      sprintf(temp, "%i,", (int)getThrustCurveData() );
      //sprintf(temp, "%lu,", getThrustCurveData() );
      strcat(ThrustCurveData, temp);
      #if defined TESTSTANDSTM32V2 || defined TESTSTANDESP32
      sprintf(temp, "%i,", (int)getPressureCurveData() );
      strcat(ThrustCurveData, temp);
      #endif
      unsigned int chk = msgChk(ThrustCurveData, sizeof(ThrustCurveData));
      sprintf(temp, "%i", chk);
      strcat(ThrustCurveData, temp);
      strcat(ThrustCurveData, ";\n");
      #ifdef TESTSTANDESP32
      Serial.print("$");
      Serial.print(ThrustCurveData);
      #endif
      SerialCom.print("$");
      SerialCom.print(ThrustCurveData);

      //This will slow down the data
      // this is for telemetry modules without enought buffer
      if (config.telemetryType == 0) 
        delay(0);
      else if (config.telemetryType == 1)  
        delay(20); 
      else if (config.telemetryType == 2)
        delay(50);
      else if (config.telemetryType == 3)
        delay(100);
    }

  }
}
/*
   CanRecord()
   First count the number of Thrust Curves. It cannot be greater than 25
   if last Thrust Curve end address is greater than the max possible
   address then the EEprom is full
*/
boolean logger_I2C_eeprom::CanRecord()
{
  long lastThrustCurve;
  lastThrustCurve = getLastThrustCurveNbr();
  if (lastThrustCurve == -1)
    return true;

  if (lastThrustCurve == 24)
  {
    return false;
  }
  // Check if eeprom is full
  if (getThrustCurveStop(lastThrustCurve) > 65500 )
  {
    return false;
  }
  return true;
}

// this is to check your eeprom byte by byte. This will take approx 11 minutes for a 512 eeprom
long logger_I2C_eeprom::checkMemoryErrors(long memoryLastAddress) {
  long errors = 0;
  for (long i = 0; i < memoryLastAddress; i++)
  {
    //read byte and save it
    byte currentByte;
    eep.read(i, ((byte*)&currentByte), sizeof(byte));
    
    //write byte 
    byte bToWrite = ~currentByte; 
    eep.write(i, ((byte*)&bToWrite), sizeof(currentByte));
    delay(5);
    //check if value is the same
    byte myByte;
    eep.read(i, ((byte*)&myByte), sizeof(byte));
    // if not the same add an error
    if(myByte != bToWrite)
      errors++;
      
    //restore previous value
    eep.write(i, ((byte*)&currentByte), sizeof(currentByte));
    delay(5);
  }
  return errors;
}

// this will check the memory size
int logger_I2C_eeprom::checkMemorySize() {
  int memSize=0;

  // check for 4093 = 32K
  if (checkWrite(4093))
      memSize = 32;
  // check for 8187 = 64 K
  if (checkWrite(8187))
      memSize = 64;
  // check for 16375 = 128k
  if (checkWrite(16375))
      memSize = 128;
  // check for 32750 = 256K
  if (checkWrite(32750))
      memSize = 256;
  // check for 65500=512K
  if (checkWrite(65500))
      memSize = 512;
  return memSize;
}

bool logger_I2C_eeprom::checkWrite(long address) {
    bool ok = false;
   //read byte and save it
    byte currentByte;
    eep.read(address, ((byte*)&currentByte), sizeof(byte));
    SerialCom.print("currentByte:");
    SerialCom.println(currentByte);
    //write byte 
    byte bToWrite = ~currentByte; 
    eep.write(address, ((byte*)&bToWrite), sizeof(currentByte));
    SerialCom.print("bToWrite:");
    SerialCom.println(bToWrite);
    delay(5);
    //check if value is the same
    byte myByte;
    eep.read(address, ((byte*)&myByte), sizeof(byte));
    SerialCom.print("myByte:");
    SerialCom.println(myByte);
    if(myByte == bToWrite)
      ok = true;
    //restore previous value
    eep.write(address, ((byte*)&currentByte), sizeof(currentByte));
    delay(5);  
    return ok;
}
