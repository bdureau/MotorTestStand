//
//    FILE: HX711.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.3.9
// PURPOSE: Library for load cells for UNO
//     URL: https://github.com/RobTillaart/HX711


#include "BHX711.h"
#include "kalman.h"

BHX711::BHX711()
{
  reset();
}


BHX711::~BHX711()
{
}


void BHX711::begin(uint8_t dataPin, uint8_t clockPin)
{
  _dataPin  = dataPin;
  _clockPin = clockPin;

  pinMode(_dataPin, INPUT);
  pinMode(_clockPin, OUTPUT);
  digitalWrite(_clockPin, LOW);

  reset();
  // init Kalman filter
  KalmanInit();
  // let's do some dummy thrust reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    kalman_read();
  }
}


void BHX711::reset()
{
  power_down();
  power_up();
  _offset   = 0;
  _scale    = 1;
  _gain     = HX711_CHANNEL_A_GAIN_128;
  _lastRead = 0;
  _mode     = HX711_AVERAGE_MODE;
}


bool BHX711::is_ready()
{
  return digitalRead(_dataPin) == LOW;
}


void BHX711::wait_ready(uint32_t ms)
{
  while (!is_ready())
  {
    delay(ms);
  }
}


bool BHX711::wait_ready_retry(uint8_t retries, uint32_t ms)
{
  while (retries--)
  {
    if (is_ready()) return true;
    delay(ms);
  }
  return false;
}


bool BHX711::wait_ready_timeout(uint32_t timeout, uint32_t ms)
{
  uint32_t start = millis();
  while (millis() - start < timeout)
  {
    if (is_ready()) return true;
    delay(ms);
  }
  return false;
}


///////////////////////////////////////////////////////////
//
//  READ
//
//  From datasheet page 4
//  When output data is not ready for retrieval,
//       digital output pin DOUT is HIGH.
//  Serial clock input PD_SCK should be LOW.
//  When DOUT goes to LOW, it indicates data is ready for retrieval.
float BHX711::read()
{
  //  this BLOCKING wait takes most time...
  while (digitalRead(_dataPin) == HIGH) yield();

  union
  {
    long value = 0;
    uint8_t data[4];
  } v;

  //  blocking part ...
  noInterrupts();

  //  Pulse the clock pin 24 times to read the data.
  //  v.data[2] = shiftIn(_dataPin, _clockPin, MSBFIRST);
  //  v.data[1] = shiftIn(_dataPin, _clockPin, MSBFIRST);
  //  v.data[0] = shiftIn(_dataPin, _clockPin, MSBFIRST);
  v.data[2] = _shiftIn();
  v.data[1] = _shiftIn();
  v.data[0] = _shiftIn();

  //  TABLE 3 page 4 datasheet
  //
  //  CLOCK      CHANNEL      GAIN      m
  //  ------------------------------------
  //   25           A         128       1    //  default
  //   26           B          32       2
  //   27           A          64       3
  //
  //  only default 128 verified,
  //  selection goes through the set_gain(gain)
  //
  uint8_t m = 1;
  if      (_gain == HX711_CHANNEL_A_GAIN_128) m = 1;
  else if (_gain == HX711_CHANNEL_A_GAIN_64)  m = 3;
  else if (_gain == HX711_CHANNEL_B_GAIN_32)  m = 2;

  while (m > 0)
  {
    //  delayMicroSeconds(1) needed for fast processors?
    digitalWrite(_clockPin, HIGH);
    digitalWrite(_clockPin, LOW);
    m--;
  }

  interrupts();
  //  yield();

  //  SIGN extend
  if (v.data[2] & 0x80) v.data[3] = 0xFF;

  _lastRead = millis();
  return 1.0 * v.value;
}

float BHX711::kalman_read() {
  _last_read_value = read();
  return KalmanCalc(_last_read_value);
}

float BHX711::read_average(uint8_t times)
{
  if (times < 1) times = 1;
  float sum = 0;
  float sum_average = 0;
  for (uint8_t i = 0; i < times; i++)
  {
    //sum += read();
    sum += kalman_read();
    sum_average += _last_read_value;
    yield();
  }
  //_last_average_read_value = sum_average / times;
  //return sum / times;
  _last_average_read_value = sum / times;
  return sum_average / times;
}


float BHX711::read_median(uint8_t times)
{
  if (times > 15) times = 15;
  if (times < 3)  times = 3;
  float samples[15];
  for (uint8_t i = 0; i < times; i++)
  {
    samples[i] = read();
    yield();
  }
  _insertSort(samples, times);
  if (times & 0x01) return samples[times/2];
  return (samples[times/2] + samples[times/2 + 1]) / 2;
}


float BHX711::read_medavg(uint8_t times)
{
  if (times > 15) times = 15;
  if (times < 3)  times = 3;
  float samples[15];
  for (uint8_t i = 0; i < times; i++)
  {
    samples[i] = read();
    yield();
  }
  _insertSort(samples, times);
  float sum = 0;
  //  iterate over 1/4 to 3/4 of the array
  uint8_t count = 0;
  uint8_t first = (times + 2) / 4;
  uint8_t last  = times - first - 1;
  for (uint8_t i = first; i <= last; i++)  //  !! include last one too
  {
    sum += samples[i];
    count++;
  }
  return sum / count;
}


float BHX711::read_runavg(uint8_t times, float alpha)
{
  if (times < 1)  times = 1;
  if (alpha < 0)  alpha = 0;
  if (alpha > 1)  alpha = 1;
  float val = read();
  for (uint8_t i = 1; i < times; i++)
  {
    val += alpha * (read() - val);
    yield();
  }
  return val;
}


///////////////////////////////////////////////////////
//
//  MODE
//
void BHX711::set_raw_mode()
{
  _mode = HX711_RAW_MODE;
}


void BHX711::set_average_mode()
{
  _mode = HX711_AVERAGE_MODE;
}


void BHX711::set_median_mode()
{
  _mode = HX711_MEDIAN_MODE;
}


void BHX711::set_medavg_mode()
{
  _mode = HX711_MEDAVG_MODE;
}


//  set_run_avg will use a default alpha of 0.5.
void BHX711::set_runavg_mode()
{
  _mode = HX711_RUNAVG_MODE;
}


uint8_t BHX711::get_mode()
{
  return _mode;
}


float BHX711::get_value(uint8_t times)
{
  float raw;
  switch(_mode)
  {
    case HX711_RAW_MODE:
      raw = read();
      break;
    case HX711_RUNAVG_MODE:
      raw = read_runavg(times);
      break;
    case HX711_MEDAVG_MODE:
      raw = read_medavg(times);
      break;
    case HX711_MEDIAN_MODE:
      raw = read_median(times);
      break;
    case HX711_AVERAGE_MODE:
    default:
      raw = read_average(times);
      
      break;
  }
  return raw - _offset;
};


float BHX711::get_units(uint8_t times)
{
  float units = get_value(times) * _scale;
  return units;
};

float BHX711::get_units_filtered(){
  float units = (_last_average_read_value - _offset) * _scale;
  return units;
}


///////////////////////////////////////////////////////
//
//  TARE
//
void BHX711::tare(uint8_t times)
{
  _offset = read_average(times);
}


float BHX711::get_tare()
{
  return -_offset * _scale;
}


bool BHX711::tare_set()
{
  return _offset != 0;
}


///////////////////////////////////////////////////////
//
//  GAIN
//
//  note: if parameter gain == 0xFF40 some compilers
//  will map that to 0x40 == HX711_CHANNEL_A_GAIN_64;
//  solution: use uint32_t or larger parameters everywhere.
//  note that changing gain/channel may take up to 400 ms (page 3)
bool BHX711::set_gain(uint8_t gain, bool forced)
{
  if ( (not forced) && (_gain == gain)) return true;
  switch(gain)
  {
    case HX711_CHANNEL_B_GAIN_32:
    case HX711_CHANNEL_A_GAIN_64:
    case HX711_CHANNEL_A_GAIN_128:
      _gain = gain;
      read();     //  next user read() is from right channel / gain
      return true;
  }
  return false;   //  unchanged, but incorrect value.
}


uint8_t BHX711::get_gain()
{
  return _gain;
}


///////////////////////////////////////////////////////
//
//  CALIBRATION AND SETUP
//
bool BHX711::set_scale(float scale)
{
  if (scale == 0) return false;
  _scale = 1.0 / scale;
  return true;
}


float BHX711::get_scale()
{
  return 1.0 / _scale;
}


void BHX711::set_offset(long offset)
{
  _offset = offset;
}


long BHX711::get_offset()
{
  return _offset;
}


//  assumes tare() has been set.
/*void HX711::calibrate_scale(uint16_t weight, uint8_t times)
{
  _scale = (1.0 * weight) / (read_average(times) - _offset);
}
*/
void BHX711::calibrate_scale(float weight, uint8_t times)
{
  _scale = (1.0 * weight) / (read_average(times) - _offset);
}


///////////////////////////////////////////////////////
//
//  POWER
//
void BHX711::power_down()
{
  //  at least 60 us HIGH
  digitalWrite(_clockPin, HIGH);
  delayMicroseconds(64);
}


void BHX711::power_up()
{
  digitalWrite(_clockPin, LOW);
}


///////////////////////////////////////////////////////
//
//  MISC
//
uint32_t BHX711::last_read()
{
  return _lastRead;
}


/////////////////////////////////////////////////////////
//
//  PRIVATE
//

void BHX711::_insertSort(float * array, uint8_t size)
{
  uint8_t t, z;
  float temp;
  for (t = 1; t < size; t++)
  {
    z = t;
    temp = array[z];
    while( (z > 0) && (temp < array[z - 1] ))
    {
      array[z] = array[z - 1];
      z--;
    }
    array[z] = temp;
    yield();
  }
}


//  MSB_FIRST optimized shiftIn
//  see datasheet page 5 for timing
uint8_t BHX711::_shiftIn()
{
  //  local variables are faster.
  uint8_t clk   = _clockPin;
  uint8_t data  = _dataPin;
  uint8_t value = 0;
  uint8_t mask  = 0x80;
  while (mask > 0)
  {
    digitalWrite(clk, HIGH);
    delayMicroseconds(1);   //  T2  >= 0.2 us
    if (digitalRead(data) == HIGH)
    {
      value |= mask;
    }
    digitalWrite(clk, LOW);
    delayMicroseconds(1);   //  keep duty cycle ~50%
    mask >>= 1;
  }
  return value;
}


//  -- END OF FILE --
