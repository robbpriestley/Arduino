// *** ENVIRONMENT LOGGER ***
// Logs temperature in °C, humidity in %, and pressure in hPa

// Data Logging Tutorial:    https://learn.adafruit.com/adafruit_data_logger_shield/light_and_temperature_logger_use_it
// Data Logger Shield Specs: https://cdn-learn.adafruit.com/downloads/pdf/adafruit-data-logger-shield.pdf
// Realtime Clock (RTC):     https://learn.adafruit.com/adafruit_data_logger_shield/using_the_real_time_clock_2
// Button Tutorial:          https://www.arduino.cc/en/tutorial/button 
// DHT Temperature Sensor:   https://learn.adafruit.com/dht
// BMP Pressure Sensor:      https://learn.adafruit.com/adafruit-bmp388/arduino

// LED resistors: 2k, Switch pull-down resistors: 10k, DHT pull-up resistor: 10k

#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#include "RTClib.h"
#include "Adafruit_BMP3XX.h"

#define DHTTYPE DHT22

bool _first;
bool _flash;
bool _debug;
bool _statusReady;
bool _statusError;
bool _displayEnabled;
bool _buttonDisplayEnabled;
bool _buttonRecordingPeriod;

char _zero = '0';
char _year[4] = {0};
char _digit[2] = {0};
char _timestamp[20] = {0};

int _errorCode = 0;
int _periodFlash = 333;

unsigned long _lastMillisFlash = 0;
unsigned long _currentMillisFlash = 0;
unsigned long _lastMillisRecord = 0;
unsigned long _currentMillisRecord = 0;

unsigned long _recordingPeriod;
unsigned long _recordingPeriodRemaining;
unsigned long _debugRecordingPeriod = 2000;

int _recordingPeriodIndex = 0;
const int REC_PERIOD_ARRAY_LEN = 6;
const int _recordingPeriodMins[REC_PERIOD_ARRAY_LEN] = { 1, 5, 15, 30, 60, 90 };
const unsigned long _recordingPeriodMillis[REC_PERIOD_ARRAY_LEN] = { 60000, 300000, 900000, 1800000, 3600000, 5400000 };

const int READ_A_PIN_TEMP = 1;
const int READ_D_PIN_DHT = 2;                 // Pin used to read the DHT22 temperature sensor
const int READ_D_PIN_BUTTON_DISP_EN = 4;      // Button to turn display on and off
const int READ_D_PIN_BUTTON_REC_PERIOD = 5;   // Button to change recording period
const int READ_D_PIN_SD = 10;                 // SD card base access pin
const int READ_D_PIN_SD_CD = 9;               // SD card CD pin indicates if card is inserted
const int READ_D_PIN_SD_WP = 8;               // SD card WP pin indicates if card is write protected
const int WRITE_D_PIN_READY_LED = 7;          // Green LED
const int WRITE_D_PIN_ERROR_LED = 6;          // Red LED
const int EEPROM_ADDRESS_REC_PERIOD_IDX = 0;  // EEPROM Address for Recording Period Index
const float REFERENCE_VOLTAGE = 5120;         // In mV

const char* C_STARTUP_MESSAGE = "ENVIRONMENT LOGGER";
const char* C_DEBUG_MODE = "DEBUG MODE";
const char* C_REC_PERIOD = "REC PERIOD";
const char* C_DISPLAY = "DISPLAY";
const char* C_ERROR = "ERROR";
const char* C_STARS = "***";
const char C_COMMA = ',';
const char C_SPACE = ' ';

File _logfile;                      // SD card logfile
RTC_PCF8523 _rtc;                   // Realtime clock
Adafruit_BMP3XX _bmp;               // Pressure sensor
DHT _dht(READ_D_PIN_DHT, DHTTYPE);  // Temperature/humidity sensor

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

// https://arduino.stackexchange.com/a/39127
void DateTimeCb(uint16_t* date, uint16_t* time)
{
   DateTime now = _rtc.now();
   
   // return date using FAT_DATE macro to format fields
   *date = FAT_DATE(now.year(), now.month(), now.day());
  
   // return time using FAT_TIME macro to format fields
   *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

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

  SetRecordingPeriodIndex();
  DisplayRecordingPeriod();
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

  _currentMillisFlash = millis();
  _currentMillisRecord = millis();
  //RecPeriodRemaining();

  if (_currentMillisFlash - _lastMillisFlash >= _periodFlash)
  {
    _flash = false;
  }

  if (_statusError)
  {
    UpdateSerial();
  }
  else if (_currentMillisRecord - _lastMillisRecord >= _recordingPeriod || _first)
  {
    float temperature = _dht.readTemperature();
    float humidity = _dht.readHumidity();
    float pressure = ReadPressure();

    SdCardWrite(temperature, humidity, pressure);
    UpdateSerial(temperature, humidity, pressure);

    _lastMillisRecord = _currentMillisRecord;
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
      SdFile::dateTimeCallback(DateTimeCb);
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

  pinMode(WRITE_D_PIN_READY_LED, OUTPUT);
  pinMode(WRITE_D_PIN_ERROR_LED, OUTPUT);
}

void RtcInit()
{
  if (!_rtc.begin())
  {
    _errorCode = 2;
    Serial.print(C_ERROR);
    Serial.print(C_SPACE);
    Serial.println(_errorCode);
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

void SetRecordingPeriodIndex()
{
  if (_debug)
  {
    _recordingPeriod = _debugRecordingPeriod;
  }
  else
  {
    EEPROM.get(EEPROM_ADDRESS_REC_PERIOD_IDX, _recordingPeriodIndex);

    if (_recordingPeriodIndex < 0 || _recordingPeriodIndex >= REC_PERIOD_ARRAY_LEN)
    {
      // Bad value stored in EEPROM, or first time use.
      _recordingPeriodIndex = 0;
      EEPROM.put(EEPROM_ADDRESS_REC_PERIOD_IDX, 0);
    }

    _recordingPeriod = _recordingPeriodMillis[_recordingPeriodIndex];
  }
}

void DisplayRecordingPeriod()
{
  if (!_debug)
  {
    int mins = _recordingPeriodMins[_recordingPeriodIndex];
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
  
  int buttonRecordingPeriodState = digitalRead(READ_D_PIN_BUTTON_REC_PERIOD);

  if (buttonRecordingPeriodState == HIGH)
  {
    _buttonRecordingPeriod = true;
  }
  else if (_buttonRecordingPeriod)
  {
    _first = true;
    _buttonRecordingPeriod = false;
    _recordingPeriodIndex = _recordingPeriodIndex == REC_PERIOD_ARRAY_LEN - 1 ? 0 : _recordingPeriodIndex + 1;
    EEPROM.put(EEPROM_ADDRESS_REC_PERIOD_IDX, _recordingPeriodIndex);
    DisplayRecordingPeriod();
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
    digitalWrite(WRITE_D_PIN_READY_LED, LOW);
    digitalWrite(WRITE_D_PIN_ERROR_LED, LOW);
  }
  else
  {
    int statusReadyWrite = _statusReady && !_flash ? HIGH : LOW;
    int statusErrorWrite = _statusError ? HIGH : LOW;
    
    digitalWrite(WRITE_D_PIN_READY_LED, statusReadyWrite);
    digitalWrite(WRITE_D_PIN_ERROR_LED, statusErrorWrite);
  }
}

void RecPeriodRemaining()
{
  
  
  unsigned long recordingPeriodRemaining = _recordingPeriod - _currentMillisRecord - _lastMillisRecord;

  if (recordingPeriodRemaining / 1000 != _recordingPeriodRemaining / 1000)
  {
    _recordingPeriodRemaining = recordingPeriodRemaining;
    Serial.println(_recordingPeriodRemaining / 1000); 
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
  
  RecordTimestampComponent(now.month(), 5);
  RecordTimestampComponent(now.day(), 8);
  RecordTimestampComponent(now.hour(), 11);
  RecordTimestampComponent(now.minute(), 14);
  RecordTimestampComponent(now.second(), 17);
}

void RecordTimestampComponent(int digit, int i)
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

  _flash = !_first;  // Don't flash the LED the first time.
  _lastMillisFlash = _currentMillisFlash;
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
