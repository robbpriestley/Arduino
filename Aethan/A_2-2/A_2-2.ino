// *** DATA LOGGER ***

// Code duplicated from:     https://github.com/robbpriestley/Arduino/blob/master/ArduinoDataLogger/ArduinoDataLogger.ino

// Data Logging Tutorial:    https://learn.adafruit.com/adafruit_data_logger_shield/light_and_temperature_logger_use_it
// Data Logger Shield Specs: https://cdn-learn.adafruit.com/downloads/pdf/adafruit-data-logger-shield.pdf
// Realtime Clock (RTC):     https://learn.adafruit.com/adafruit_data_logger_shield/using_the_real_time_clock_2

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
 */

#include <SD.h>
#include <SPI.h>
#include "RTClib.h"

bool _first;
bool _statusReady;
bool _statusError;
bool _flashSdWrite;
bool _flashActivity;

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

unsigned long _recPeriod = 5000;    // *** RECORDING PERIOD in milliseconds ***
unsigned long _recPeriodRemaining;

const char* C_STARTUP_MESSAGE = "DATA LOGGER";
const char* C_REC_PERIOD = "REC PERIOD";
const char* C_ERROR = "ERROR";
const char* C_STARS = "***";
const char C_COMMA = ',';
const char C_SPACE = ' ';

// ANALOG PINS

// None

// DIGITAL PINS

const int WRITE_D_PIN_RED_LED   = 6;   // Red LED
const int WRITE_D_PIN_GREEN_LED = 7;   // Green LED
const int READ_D_PIN_SD_WP      = 8;   // SD card WP pin indicates if card is write protected
const int READ_D_PIN_SD_CD      = 9;   // SD card CD pin indicates if card is inserted
const int READ_D_PIN_SD         = 10;  // SD card base access pin

// OBJECTS

File _logfile;     // SD card logfile
RTC_PCF8523 _rtc;  // Realtime clock

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
  CharInit();

  _first = true;

  DisplayRecPeriod();
}

void loop() 
{
  int sdCardIn = digitalRead(READ_D_PIN_SD_CD);            // Is an SD card inserted?
  int sdCardWriteProtect = digitalRead(READ_D_PIN_SD_WP);  // Is the SD card write protected?

  CheckSdInit(sdCardIn);
  DetermineStatusError(sdCardIn, sdCardWriteProtect);
  DetermineStatusReady(sdCardIn, sdCardWriteProtect);
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
    float value = 0.0;

    SdCardWrite(value);
    UpdateSerial(value);

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

  _rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Uncomment to set date and time.
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

void DisplayRecPeriod()
{
  Serial.print(C_REC_PERIOD);
  Serial.print(C_SPACE);
  Serial.println(_recPeriod);
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

void UpdateStatusLeds()
{
  int green = _statusReady && !_flashActivity ? HIGH : LOW;
  int red = _statusError || _flashSdWrite ? HIGH : LOW;
  
  digitalWrite(WRITE_D_PIN_GREEN_LED, green);
  digitalWrite(WRITE_D_PIN_RED_LED, red);
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

void SdCardWrite(float value)
{
  UpdateTimestamp();
  
  _logfile.print(_timestamp);
  _logfile.print(C_COMMA);
  _logfile.print(value);
  _logfile.println();

  _logfile.flush();  // Don't do this too frequently, i.e. < 1s

  _flashSdWrite = !_first;  // Don't flash the LED the first time.
  _lastMillisFlashSdWrite = _currentMillisFlashSdWrite;
}

void UpdateSerial()
{
  UpdateTimestamp();
    
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

void UpdateSerial(float value)
{
  UpdateTimestamp();
  
  if (_statusReady)
  {
    Serial.print(_timestamp);
    Serial.print(C_COMMA);
    Serial.print(value);
    Serial.println();
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
