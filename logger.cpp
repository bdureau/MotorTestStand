// config.h must be included BEFORE the board test: TESTSTANDESP32V4 is defined there.
#include "config.h"
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3 || defined TESTSTANDESP32V4
#include "logger.h"

/**
 * @brief Constructor. No heap allocation here: the RAM buffer grows on demand.
 */
logger::logger() {
  thrustCurveData = nullptr;
  dataPos = 0;
  dataCapacity = 0;
  fsReady = false;
}

/**
 * @brief Mounts LittleFS, formatting the partition if it cannot be mounted.
 * Requires a partition scheme that contains a "spiffs" data partition
 * (e.g. "Huge APP (3MB No OTA/1MB SPIFFS)" or "No OTA (2MB APP/2MB SPIFFS)").
 */
bool logger::initFileSystem() {
  fsReady = LittleFS.begin(false);
  if (!fsReady) {
#ifdef DEBUG
    Serial.println(F("LITTLEFS mount failed, formatting"));
#endif
    fsReady = LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED);
  }
  return fsReady;
}

bool logger::isReady() {
  return fsReady;
}

/**
 * @brief Builds "/thrustCurve<N>.bin".
 */
void logger::buildFileName(char *buf, size_t len, long thrustCurveNbr) {
  snprintf(buf, len, "/" THRUSTCURVE_FILE_PREFIX "%ld" THRUSTCURVE_FILE_EXT, thrustCurveNbr);
}

/**
 * @brief Returns N for "thrustCurve<N>.bin" (with or without leading path), -1 otherwise.
 * Works whether File::name() returns "thrustCurve3.bin" (core 2.x/3.x) or
 * "/thrustCurve3.bin" (older cores).
 */
long logger::parseThrustCurveNbr(const char *fileName) {
  if (fileName == nullptr) return -1;
  const char *slash = strrchr(fileName, '/');
  const char *name = slash ? slash + 1 : fileName;

  const size_t prefixLen = strlen(THRUSTCURVE_FILE_PREFIX);
  const size_t extLen = strlen(THRUSTCURVE_FILE_EXT);
  const size_t nameLen = strlen(name);
  if (nameLen <= prefixLen + extLen) return -1;
  if (strncmp(name, THRUSTCURVE_FILE_PREFIX, prefixLen) != 0) return -1;
  if (strcmp(name + nameLen - extLen, THRUSTCURVE_FILE_EXT) != 0) return -1;

  long nbr = 0;
  for (const char *p = name + prefixLen; p < name + nameLen - extLen; p++) {
    if (*p < '0' || *p > '9') return -1;
    nbr = nbr * 10 + (*p - '0');
  }
  return nbr;
}

/**
 * @brief Deletes every thrust curve file (".bin" and legacy ".json").
 * Files are never removed while the directory is being iterated: each pass finds one
 * file, closes the directory, then removes it. Non thrust-curve files are left alone.
 */
bool logger::clearThrustCurveList() {
  if (!fsReady) return false;
  const size_t prefixLen = strlen(THRUSTCURVE_FILE_PREFIX);

  for (int guard = 0; guard < 1000; guard++) {
    char toDelete[40] = "";
    {
      File root = LittleFS.open("/");
      if (!root || !root.isDirectory()) return false;
      File file = root.openNextFile();
      while (file) {
        const char *slash = strrchr(file.name(), '/');
        const char *name = slash ? slash + 1 : file.name();
        if (strncmp(name, THRUSTCURVE_FILE_PREFIX, prefixLen) == 0) {
          snprintf(toDelete, sizeof(toDelete), "/%s", name);
          file.close();
          break;
        }
        file.close();
        file = root.openNextFile();
      }
      root.close();
    }
    if (toDelete[0] == '\0') return true;      // nothing left
    if (!LittleFS.remove(toDelete)) return false;
  }
  return false;
}

/**
 * @brief Deletes the highest numbered thrust curve.
 */
bool logger::eraseLastThrustCurve() {
  if (!fsReady) return false;
  long last = getLastThrustCurveNbr();
  if (last < 0) return false;
  char fileName[40];
  buildFileName(fileName, sizeof(fileName), last);
  return LittleFS.remove(fileName);
}

/**
 * @brief Highest N among "thrustCurve<N>.bin", or -1 if none.
 */
long logger::getLastThrustCurveNbr() {
  long maxThrustCurveNumber = -1;
  if (!fsReady) return maxThrustCurveNumber;

  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) return maxThrustCurveNumber;

  File file = root.openNextFile();
  while (file) {
    long nbr = parseThrustCurveNbr(file.name());
    if (nbr > maxThrustCurveNumber) maxThrustCurveNumber = nbr;
    file.close();
    file = root.openNextFile();
  }
  root.close();
  return maxThrustCurveNumber;
}

bool logger::writeFastThrustCurve() {
  return writeThrustCurve(getLastThrustCurveNbr() + 1);
}

/**
 * @brief Writes the RAM buffer to "/thrustCurve<N>.bin" in one write.
 * A partially written file (filesystem full) is removed so it can't be read back corrupted.
 */
bool logger::writeThrustCurve(long thrustCurveNbr) {
  if (!fsReady || thrustCurveData == nullptr || dataPos <= 0) return false;

  char fileName[40];
  buildFileName(fileName, sizeof(fileName), thrustCurveNbr);

  File file = LittleFS.open(fileName, "w");
  if (!file) return false;

  const size_t expected = (size_t)dataPos * sizeof(ThrustCurveDataStruct);
  size_t written = file.write((const uint8_t *)thrustCurveData, expected);
  file.close();

  if (written != expected) {
    LittleFS.remove(fileName);
    return false;
  }
  return true;
}

/**
 * @brief Loads "/thrustCurve<N>.bin" into the RAM buffer.
 */
bool logger::readThrustCurve(long thrustCurveNbr) {
  if (!fsReady) return false;

  char fileName[40];
  buildFileName(fileName, sizeof(fileName), thrustCurveNbr);

  File file = LittleFS.open(fileName, "r");
  if (!file) return false;

  size_t fileSize = file.size();
  if (fileSize == 0 || (fileSize % sizeof(ThrustCurveDataStruct)) != 0) {
    file.close();
    return false;
  }

  long nbrOfSamples = fileSize / sizeof(ThrustCurveDataStruct);
  ThrustCurveDataStruct *buf = (ThrustCurveDataStruct *)malloc(fileSize);
  if (buf == nullptr) {
    file.close();
    return false;
  }

  size_t readBytes = file.read((uint8_t *)buf, fileSize);
  file.close();
  if (readBytes != fileSize) {
    free(buf);
    return false;
  }

  if (thrustCurveData != nullptr) free(thrustCurveData);
  thrustCurveData = buf;
  dataPos = nbrOfSamples;
  dataCapacity = nbrOfSamples;
  return true;
}

/**
 * @brief Appends currentRecord. The buffer grows by THRUSTCURVE_ALLOC_CHUNK samples at a
 * time instead of one realloc per sample (less heap fragmentation next to the BT stack).
 * @return false if the heap is exhausted (the samples already stored are kept).
 */
bool logger::addToCurrentThrustCurve() {
  if (dataPos >= dataCapacity) {
    long newCapacity = dataCapacity + THRUSTCURVE_ALLOC_CHUNK;
    ThrustCurveDataStruct *newBuf = (ThrustCurveDataStruct *)realloc(
        thrustCurveData, newCapacity * sizeof(ThrustCurveDataStruct));
    if (newBuf == nullptr) return false;
    thrustCurveData = newBuf;
    dataCapacity = newCapacity;
  }
  thrustCurveData[dataPos++] = currentRecord;
  return true;
}

bool logger::initThrustCurve() {
  if (thrustCurveData != nullptr) free(thrustCurveData);
  thrustCurveData = nullptr;
  dataPos = 0;
  dataCapacity = 0;
  return true;
}

ThrustCurveDataStruct* logger::getThrustCurveData() {
  return thrustCurveData;
}

void logger::setThrustCurveTimeData(long diffTime) {
  currentRecord.diffTime = diffTime;
}

void logger::setThrustCurveData(long thrust) {
  currentRecord.thrust = thrust;
}

void logger::setPressureCurveData(long pressure) {
  currentRecord.pressure = pressure;
}

void logger::setPressureCurveData2(long pressure) {
  currentRecord.pressure2 = pressure;
}

long logger::getThrustCurveSize() {
  return dataPos;
}

/**
 * @brief Filesystem usage in percent (used for the telemetry "memory used" field).
 */
int logger::getUsedPercent() {
  if (!fsReady) return 0;
  size_t total = LittleFS.totalBytes();
  if (total == 0) return 0;
  return (int)((100ULL * LittleFS.usedBytes()) / total);
}

void logger::getThrustCurveMinAndMax(long thrustCurveNbr)
{
  _ThrustCurveMinAndMax.minThrust = 0;
  _ThrustCurveMinAndMax.maxThrust = 0;
  _ThrustCurveMinAndMax.minPressure = 0;
  _ThrustCurveMinAndMax.maxPressure = 0;
  _ThrustCurveMinAndMax.minPressure2 = 0;
  _ThrustCurveMinAndMax.maxPressure2 = 0;
  _ThrustCurveMinAndMax.thrustCurveDuration = 0;

  if (readThrustCurve(thrustCurveNbr)) {
    for (long i = 0; i < dataPos ; i++)
    {
      _ThrustCurveMinAndMax.thrustCurveDuration += thrustCurveData[i].diffTime;

      if (thrustCurveData[i].thrust < _ThrustCurveMinAndMax.minThrust)
        _ThrustCurveMinAndMax.minThrust = thrustCurveData[i].thrust;
      if (thrustCurveData[i].thrust > _ThrustCurveMinAndMax.maxThrust)
        _ThrustCurveMinAndMax.maxThrust = thrustCurveData[i].thrust;

      if (thrustCurveData[i].pressure < _ThrustCurveMinAndMax.minPressure)
        _ThrustCurveMinAndMax.minPressure = thrustCurveData[i].pressure;
      if (thrustCurveData[i].pressure > _ThrustCurveMinAndMax.maxPressure)
        _ThrustCurveMinAndMax.maxPressure = thrustCurveData[i].pressure;

      if (thrustCurveData[i].pressure2 < _ThrustCurveMinAndMax.minPressure2)
        _ThrustCurveMinAndMax.minPressure2 = thrustCurveData[i].pressure2;
      if (thrustCurveData[i].pressure2 > _ThrustCurveMinAndMax.maxPressure2)
        _ThrustCurveMinAndMax.maxPressure2 = thrustCurveData[i].pressure2;
    }
  }
}

long logger::getThrustCurveDuration() { return _ThrustCurveMinAndMax.thrustCurveDuration; }
long logger::getMinThrust()           { return _ThrustCurveMinAndMax.minThrust; }
long logger::getMaxThrust()           { return _ThrustCurveMinAndMax.maxThrust; }
long logger::getMaxPressure()         { return _ThrustCurveMinAndMax.maxPressure; }
long logger::getMaxPressure2()        { return _ThrustCurveMinAndMax.maxPressure2; }

/**
 * @brief Streams one curve as "$data,<nbr>,<time>,<thrust>,<p1>,<p2>,<chk>;\n" frames.
 * Same frame layout as before; uses %ld for long values and the same telemetryType
 * throttling as the EEPROM logger.
 */
void logger::printThrustCurveData(int thrustCurveNbr)
{
  unsigned long currentTime = 0;

  if (readThrustCurve(thrustCurveNbr)) {
    for (long i = 0; i < dataPos ; i++)
    {
      char thrustCurveDt[120] = "";
      char temp[20] = "";
      currentTime = currentTime + thrustCurveData[i].diffTime;
      strcat(thrustCurveDt, "data,");
      snprintf(temp, sizeof(temp), "%d,", thrustCurveNbr);
      strcat(thrustCurveDt, temp);
      snprintf(temp, sizeof(temp), "%lu,", currentTime);
      strcat(thrustCurveDt, temp);
      snprintf(temp, sizeof(temp), "%ld,", thrustCurveData[i].thrust);
      strcat(thrustCurveDt, temp);
      snprintf(temp, sizeof(temp), "%ld,", thrustCurveData[i].pressure);
      strcat(thrustCurveDt, temp);
      snprintf(temp, sizeof(temp), "%ld,", thrustCurveData[i].pressure2);
      strcat(thrustCurveDt, temp);

      unsigned int chk = msgChk(thrustCurveDt, sizeof(thrustCurveDt));
      snprintf(temp, sizeof(temp), "%u", chk);
      strcat(thrustCurveDt, temp);
      strcat(thrustCurveDt, ";\n");

      Serial.print("$");
      Serial.print(thrustCurveDt);
      SerialCom.print("$");
      SerialCom.print(thrustCurveDt);

      // same throttling as logger_I2C_eeprom::printThrustCurveData
      if (config.telemetryType == 1)
        delay(20);
      else if (config.telemetryType == 2)
        delay(50);
      else if (config.telemetryType == 3)
        delay(100);
    }
  }
}

unsigned int logger::msgChk( char * buffer, long length ) {
  long index;
  unsigned int checksum;
  for ( index = 0L, checksum = 0; index < length; checksum += (unsigned int) buffer[index++] );
  return (unsigned int) ( checksum % 256 );
}
#endif
