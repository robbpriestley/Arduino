// *** ENVIRONMENT LOGGER ***
// Logs temperature in °C, humidity in %, and pressure in hPa

// Data Logging Tutorial:    https://learn.adafruit.com/adafruit_data_logger_shield/light_and_temperature_logger_use_it
// Data Logger Shield Specs: https://cdn-learn.adafruit.com/downloads/pdf/adafruit-data-logger-shield.pdf
// Realtime Clock (RTC):     https://learn.adafruit.com/adafruit_data_logger_shield/using_the_real_time_clock_2
// Button Tutorial:          https://www.arduino.cc/en/tutorial/button 
// DHT Temperature Sensor:   https://learn.adafruit.com/dht
// BMP Pressure Sensor:      https://learn.adafruit.com/adafruit-bmp388/arduino

// LED resistors: 2k, Switch pull-down resistors: 10k, DHT pull-up resistor: 10k

/*
  ERROR CODES:
  0: No error
  1: SD card failed, or not present
  2: Could not init RTC
  3: RTC is not running
  4: SD card is not inserted
  5: SD card is write protected
  6: Log file error
  7: Unknown status
  8: BMP init error
  9: BMP read error
 */

#include <SD.h>
#include <EEPROM.h>
#include "DHT.h"
#include "RTClib.h"
#include "Adafruit_BMP3XX.h"

#define DHTTYPE DHT22
#define REC_PERIOD_ARRAY_LEN 6
#define EEPROM_ADDRESS_REC_PERIOD_IDX 0  // EEPROM Address for Recording Period Index

// ANALOG PINS

#define READ_A_PIN_TEMP 1

// DIGITAL PINS

#define READ_D_PIN_DHT               2   // Pin used to read the DHT22 temperature sensor
#define READ_D_PIN_BUTTON_DISP_EN    4   // Button to turn display on and off
#define READ_D_PIN_BUTTON_REC_PERIOD 5   // Button to change recording period
#define WRITE_D_PIN_RED_LED          6   // Red LED
#define WRITE_D_PIN_GREEN_LED        7   // Green LED
#define READ_D_PIN_SD_WP             8   // SD card WP pin indicates if card is write protected
#define READ_D_PIN_SD_CD             9   // SD card CD pin indicates if card is inserted
#define READ_D_PIN_SD                10  // SD card base access pin

bool _first;
bool _debug;
bool _statusReady;
bool _statusError;
bool _flashSdWrite;
bool _flashActivity;
bool _displayEnabled;
bool _buttonRecPeriod;
bool _buttonDisplayEnabled;

char _zero = '0';
char _year[4] = {0};
char _digit[2] = {0};
char _timestamp[20] = {0};

int _errorCode = 0;
int _periodFlashSdWrite = 333;
int _periodFlashActivity = 55;

unsigned long _lastMillisRec = 0;
unsigned long _currentMillisRec = 0;
unsigned long _lastMillisFlashSdWrite = 0;
unsigned long _currentMillisFlashSdWrite = 0;
unsigned long _lastMillisFlashActivity = 0;
unsigned long _currentMillisFlashActivity = 0;

unsigned long _recPeriod;
unsigned long _recPeriodRemaining;
unsigned long _debugRecPeriod = 2000;

int _recPeriodIndex = 0;
const int _recPeriodMins[REC_PERIOD_ARRAY_LEN] = { 1, 5, 15, 30, 60, 90 };
const unsigned long _recPeriodMillis[REC_PERIOD_ARRAY_LEN] = { 60000, 300000, 900000, 1800000, 3600000, 5400000 };

const char* C_STARTUP_MESSAGE = "ENVIRONMENT LOGGER";
const char* C_DEBUG_MODE = "DEBUG MODE";
const char* C_REC_PERIOD = "REC PERIOD";
const char* C_DISPLAY = "DISPLAY";
const char* C_ERROR = "ERROR";
const char* C_STARS = "***";
const char C_COMMA = ',';
const char C_SPACE = ' ';

// OBJECTS

File _logfile;                      // SD card logfile
RTC_PCF8523 _rtc;                   // Realtime clock
Adafruit_BMP3XX _bmp;               // Pressure sensor
DHT _dht(READ_D_PIN_DHT, DHTTYPE);  // Temperature/humidity sensor

void setup() 
{
  Serial.begin(57600);
  
  Serial.print(C_STARS);
  Serial.print(C_SPACE);
  Serial.print(C_STARTUP_MESSAGE);
  Serial.print(C_SPACE);
  Serial.println(C_STARS);
    
  PinInit();
  RtcInit();
  BmpInit();
  CharInit();
  _dht.begin();

  _first = true;
  _debug = true;
  _displayEnabled = true;

  SetRecPeriodIndex();
  DisplayRecPeriod();
}

void loop() 
{
  int sdCardIn = digitalRead(READ_D_PIN_SD_CD);            // Is an SD card inserted?
  int sdCardWriteProtect = digitalRead(READ_D_PIN_SD_WP);  // Is the SD card write protected?

  CheckSdInit(sdCardIn);
  DetermineStatusError(sdCardIn, sdCardWriteProtect);
  DetermineStatusReady(sdCardIn, sdCardWriteProtect);
  ButtonStatus();
  UpdateStatusLeds();

  _currentMillisRec = millis();
  _currentMillisFlashSdWrite = millis();
  _currentMillisFlashActivity = millis();
  RecPeriodRemaining();

  if (_currentMillisFlashSdWrite - _lastMillisFlashSdWrite >= _periodFlashSdWrite)
  {
    _flashSdWrite = false;
  }

  if (_currentMillisFlashActivity - _lastMillisFlashActivity >= _periodFlashActivity)
  {
    _flashActivity = false;
  }

  if (_statusError)
  {
    UpdateSerial();
  }
  else if (_currentMillisRec - _lastMillisRec >= _recPeriod || _first)
  {
    float temperature = _dht.readTemperature();
    float humidity = _dht.readHumidity();
    float pressure = ReadPressure();

    SdCardWrite(temperature, humidity, pressure);
    UpdateSerial(temperature, humidity, pressure);

    _lastMillisRec = _currentMillisRec;
    _first = false;
  }
}

void SdInit()
{
  if (!SD.begin(READ_D_PIN_SD))
  {
    _statusError = true;
    _errorCode = 1;
    return;
  }

  char filename[] = "LOGGER00.CSV";
  
  for (uint8_t i = 0; i < 100; i++)
  {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    
    if (!SD.exists(filename))
    {
      _logfile = SD.open(filename, FILE_WRITE);  // Only open a new file if it doesn't exist.
      break;
    }
  }
  
  if (_logfile)
  {
    Serial.println(filename);
  }
}

void PinInit()
{
  pinMode(READ_D_PIN_DHT, INPUT);
  pinMode(READ_D_PIN_BUTTON_REC_PERIOD, INPUT);
  
  pinMode(READ_D_PIN_SD_CD, INPUT_PULLUP);
  pinMode(READ_D_PIN_SD_WP, INPUT_PULLUP);

  pinMode(WRITE_D_PIN_GREEN_LED, OUTPUT);
  pinMode(WRITE_D_PIN_RED_LED, OUTPUT);
}

void RtcInit()
{
  if (!_rtc.begin())
  {
    _errorCode = 2;
    Serial.print(C_ERROR);
    Serial.print(C_SPACE);
    Serial.println(_errorCode);
    digitalWrite(WRITE_D_PIN_RED_LED, HIGH);
    while (1);
  }

  // _rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Uncomment to set date and time.
}

void BmpInit()
{
  if (!_bmp.begin()) {
    _errorCode = 8;
    Serial.print(C_ERROR);
    Serial.print(C_SPACE);
    Serial.println(_errorCode);
    digitalWrite(WRITE_D_PIN_RED_LED, HIGH);
    while (1);
  }

  _bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  _bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  _bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  // Perform first reading to "clear" bad value
  if (!_bmp.performReading())
  {
    _errorCode = 9;
    Serial.print(C_ERROR);
    Serial.print(C_SPACE);
    Serial.println(_errorCode);
    return;
  }
}

void CharInit()
{
  _digit[0] = 'X';
  _digit[1] = 'X';

  memset(_timestamp, 0x00, 20);
  
  _timestamp[4] = '-';
  _timestamp[7] = '-';
  _timestamp[10] = ' ';
  _timestamp[13] = ':';
  _timestamp[16] = ':';
}

void SetRecPeriodIndex()
{
  if (_debug)
  {
    _recPeriod = _debugRecPeriod;
  }
  else
  {
    EEPROM.get(EEPROM_ADDRESS_REC_PERIOD_IDX, _recPeriodIndex);

    if (_recPeriodIndex < 0 || _recPeriodIndex >= REC_PERIOD_ARRAY_LEN)
    {
      // Bad value stored in EEPROM, or first time use.
      _recPeriodIndex = 0;
      EEPROM.put(EEPROM_ADDRESS_REC_PERIOD_IDX, 0);
    }

    _recPeriod = _recPeriodMillis[_recPeriodIndex];
  }
}

void DisplayRecPeriod()
{
  if (!_debug)
  {
    int mins = _recPeriodMins[_recPeriodIndex];
    Serial.print(C_REC_PERIOD);
    Serial.print(C_SPACE);
    Serial.println(mins);
  }
}

void CheckSdInit(int sdCardIn)
{
  if (sdCardIn == LOW && !_logfile)  // SD card is in but file has not been initialized.
  {
    SdInit();
  }
}

void DetermineStatusError(int sdCardIn, int sdCardWriteProtect)
{  
  if (!_rtc.initialized())
  {
    _statusError = true;
    _errorCode = 3;
  }
  else if (sdCardIn == HIGH)  // CD and WP pins are open drain and normal LOW/HIGH logic is reversed.
  {
    if (_logfile)
    {
      _logfile.close();  
    }
    
    _statusError = true;
    _errorCode = 4;
  }
  else if (sdCardIn == LOW && sdCardWriteProtect == HIGH)  // CD and WP pins are open drain and normal LOW/HIGH logic is reversed.
  {
    _statusError = true;
    _errorCode = 5;
  }
  else if (!_logfile)
  {
    _statusError = true;
    _errorCode = 6;
  }
  else  // No error.
  {
    _statusError = false;
    _errorCode = 0;
  }
}

void DetermineStatusReady(int sdCardIn, int sdCardWriteProtect)
{
  // CD and WP pins are open drain and normal LOW/HIGH logic is reversed.
  _statusReady = !_statusError && sdCardIn == LOW && sdCardWriteProtect == LOW;
}

void ButtonStatus()
{
  // Button Recording Period
  
  int buttonRecPeriodState = digitalRead(READ_D_PIN_BUTTON_REC_PERIOD);

  if (buttonRecPeriodState == HIGH)
  {
    _buttonRecPeriod = true;
  }
  else if (_buttonRecPeriod)
  {
    _first = true;
    _buttonRecPeriod = false;
    _recPeriodIndex = _recPeriodIndex == REC_PERIOD_ARRAY_LEN - 1 ? 0 : _recPeriodIndex + 1;
    _recPeriod = _recPeriodMillis[_recPeriodIndex];
    _recPeriodRemaining = _recPeriodMillis[_recPeriodIndex];
    EEPROM.put(EEPROM_ADDRESS_REC_PERIOD_IDX, _recPeriodIndex);
    DisplayRecPeriod();
  }

  // Button Display Enabled

  int buttonDisplayEnabledState = digitalRead(READ_D_PIN_BUTTON_DISP_EN);

  if (buttonDisplayEnabledState == HIGH)
  {
    _buttonDisplayEnabled = true;
  }
  else if (_buttonDisplayEnabled)
  {
    _buttonDisplayEnabled = false;
    _displayEnabled = !_displayEnabled;

    Serial.print(C_DISPLAY);
    Serial.print(C_SPACE);
    Serial.println(_displayEnabled);
  }
}

void UpdateStatusLeds()
{
  if (!_displayEnabled)
  {
    digitalWrite(WRITE_D_PIN_GREEN_LED, LOW);
    digitalWrite(WRITE_D_PIN_RED_LED, LOW);
  }
  else
  {
    int green = _statusReady && !_flashActivity ? HIGH : LOW;
    int red = _statusError || _flashSdWrite ? HIGH : LOW;
    
    digitalWrite(WRITE_D_PIN_GREEN_LED, green);
    digitalWrite(WRITE_D_PIN_RED_LED, red);
  }
}

void RecPeriodRemaining()
{
  unsigned long recPeriodRemaining = _recPeriod - (_currentMillisRec - _lastMillisRec);

  if (recPeriodRemaining / 1000 != _recPeriodRemaining / 1000)
  {
    _flashActivity = true;
    _lastMillisFlashActivity = _currentMillisFlashActivity;
    
    _recPeriodRemaining = recPeriodRemaining;
    Serial.println(_recPeriodRemaining / 1000); 
  }
}

float ReadPressure()
{
  if (!_bmp.performReading())
  {
    _errorCode = 9;
    Serial.print(C_ERROR);
    Serial.print(C_SPACE);
    Serial.println(_errorCode);
    return;
  }
  
  return _bmp.pressure / 100.0;
}

void UpdateTimestamp()
{
  DateTime now  = _rtc.now();

  itoa(now.year(), _year, 10);

  _timestamp[0] = _year[0];
  _timestamp[1] = _year[1];
  _timestamp[2] = _year[2];
  _timestamp[3] = _year[3];
  
  ModifyTimestampComponent(now.month(), 5);
  ModifyTimestampComponent(now.day(), 8);
  ModifyTimestampComponent(now.hour(), 11);
  ModifyTimestampComponent(now.minute(), 14);
  ModifyTimestampComponent(now.second(), 17);
}

void ModifyTimestampComponent(int digit, int i)
{
  _digit[0] = '0';
  _digit[1] = '0';
  
  itoa(digit, _digit, 10);

  if (digit < 10)
  {
    _timestamp[i]     = _zero;
    _timestamp[i + 1] = _digit[0];
  }
  else
  {
    _timestamp[i]     = _digit[0];
    _timestamp[i + 1] = _digit[1];
  }
}

/*
  Previously, GetTimestamp() was called once, at the beginning of the loop() function, and the returned value was stored in a local string variable for re-use.
  However, due to some glitch, if the DHT was accessed after the call to GetTimestamp(), the local string variable value got corrupted and would be populated
  with non-printable characters. This would look bad at best, and at worst it seemed to cause general havoc such as hanging the Uno or restarting it. So, I
  decided to only access the RTC immediately prior to using the timestamp value. This seems to have resolved the problem with the glitch.
*/

void SdCardWrite(float temperature, float humidity, float pressure)
{
  UpdateTimestamp();
  
  _logfile.print(_timestamp);
  _logfile.print(C_COMMA);
  _logfile.print(temperature);
  _logfile.print(C_COMMA);
  _logfile.print(humidity);
  _logfile.print(C_COMMA);
  _logfile.print(pressure);
  _logfile.println();

  _logfile.flush();  // Don't do this too frequently, i.e. < 1s

  _flashSdWrite = !_first;  // Don't flash the LED the first time.
  _lastMillisFlashSdWrite = _currentMillisFlashSdWrite;
}

void UpdateSerial()
{
  UpdateTimestamp();
  
  if (_debug)
  {
    Serial.println(C_DEBUG_MODE);
  }
  
  if (_statusReady || !_statusError)
  {
    _errorCode = 7;
  }

  Serial.print(_timestamp);
  Serial.print(C_SPACE);
  Serial.print(C_ERROR);
  Serial.print(C_SPACE);
  Serial.println(_errorCode);
}

void UpdateSerial(float temperature, float humidity, float pressure)
{
  UpdateTimestamp();

  if (_debug)
  {
    Serial.print(C_DEBUG_MODE);
    Serial.print(C_SPACE);
  }
  
  if (_statusReady)
  {
    Serial.print(_timestamp);
    Serial.print(C_COMMA);
    Serial.print(temperature);
    Serial.print(C_COMMA);
    Serial.print(humidity);
    Serial.print(C_COMMA);
    Serial.println(pressure);
  }
  else if (_statusError)
  {
    Serial.print(_timestamp);
    Serial.print(C_SPACE);
    Serial.print(C_ERROR);
    Serial.print(C_SPACE);
    Serial.println(_errorCode);
  }
  else
  {
    _errorCode = 8;
    Serial.print(_timestamp);
    Serial.print(C_SPACE);
    Serial.print(C_ERROR);
    Serial.print(C_SPACE);
    Serial.println(_errorCode);
  }
}
