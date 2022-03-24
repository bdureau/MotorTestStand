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
 #ifdef TESTSTANDSTM32V2
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
      char temp[9] = "";
      currentTime = currentTime + getThrustCurveTimeData();
      strcat(ThrustCurveData, "data,");
      sprintf(temp, "%i,", ThrustCurveNbr );
      strcat(ThrustCurveData, temp);
      sprintf(temp, "%i,", currentTime );
      //sprintf(temp, "%lu,", currentTime );
      strcat(ThrustCurveData, temp);
      sprintf(temp, "%i,", getThrustCurveData() );
      //sprintf(temp, "%lu,", getThrustCurveData() );
      strcat(ThrustCurveData, temp);
      #ifdef TESTSTANDSTM32V2
      sprintf(temp, "%i,", getPressureCurveData() );
      strcat(ThrustCurveData, temp);
      #endif
      unsigned int chk = msgChk(ThrustCurveData, sizeof(ThrustCurveData));
      sprintf(temp, "%i", chk);
      strcat(ThrustCurveData, temp);
      strcat(ThrustCurveData, ";\n");
      SerialCom.print("$");
      SerialCom.print(ThrustCurveData);
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
