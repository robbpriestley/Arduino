// https://learn.adafruit.com/adafruit_data_logger_shield/light_and_temperature_logger_use_it
// https://learn.adafruit.com/adafruit_data_logger_shield/using_the_real_time_clock_2
// https://cdn-learn.adafruit.com/downloads/pdf/adafruit-data-logger-shield.pdf
// https://www.arduino.cc/en/tutorial/button 
// LED resistors: 2k, Switch pull-down resistors: 10k

#include <SD.h>
#include <SPI.h>
#include "RTClib.h"

File _logfile;
RTC_PCF8523 _rtc;

bool _first;
bool _flash;
bool _statusReady;
bool _statusError;
bool _buttonRecordingPeriod;

String _errorMessage;
int _periodFlash = 333;

unsigned long _lastMillisFlash = 0;
unsigned long _currentMillisFlash = 0;
unsigned long _lastMillisRecord = 0;
unsigned long _currentMillisRecord = 0;

const int _recordingPeriod = 1500; // in milliseconds

const int READ_D_PIN_SD = 10;          // SD card base access pin
const int READ_D_PIN_SD_CD = 9;        // SD card CD pin indicates if card is inserted
const int READ_D_PIN_SD_WP = 8;        // SD card WP pin indicates if card is write protected
const int WRITE_D_PIN_READY_LED = 7;   // Green LED
const int WRITE_D_PIN_ERROR_LED = 6;   // Red LED
const float REFERENCE_VOLTAGE = 5120;  // In mV

void setup() 
{
  Serial.begin(57600);
  Serial.println("*** ARDUINO LOGGER ***");
    
  PinInit();
  RtcInit();

  _first = true;

  DisplayRecordingPeriod();
}

void loop() 
{
  String timestamp = GetTimestamp();

  int sdCardIn = digitalRead(READ_D_PIN_SD_CD);            // Is an SD card inserted?
  int sdCardWriteProtect = digitalRead(READ_D_PIN_SD_WP);  // Is the SD card write protected?

  CheckSdInit(sdCardIn);
  DetermineStatusError(sdCardIn, sdCardWriteProtect);
  DetermineStatusReady(sdCardIn, sdCardWriteProtect);
  UpdateStatusLeds();

  _currentMillisFlash = millis();
  _currentMillisRecord = millis();

  if (_currentMillisFlash - _lastMillisFlash >= _periodFlash)
  {
    _flash = false;
  }

  if (_statusError)
  {
    UpdateSerial(timestamp, 0.0);
  }
  else if (_currentMillisRecord - _lastMillisRecord >= _recordingPeriod || _first)
  {
    float value = GetValue();
    
    SdCardWrite(timestamp, value);
    UpdateSerial(timestamp, value);

    _lastMillisRecord = _currentMillisRecord;
    _first = false;
  }
}

float GetValue()
{
  // DO SOMETHING TO GET A READING FROM SOMEWHERE OR A VALUE FROM A SENSOR

  float value = 1.0;  // 1.0 == dummy value
  
  return value;
}

void SdInit()
{
  if (!SD.begin(READ_D_PIN_SD))
  {
    _statusError = true;
    _errorMessage = "SD card failed, or not present";
    return;
  }

  // Create a new file.
  
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
    Serial.print("Logging to: ");
    Serial.println(filename);
  }
}

void PinInit()
{
  pinMode(READ_D_PIN_SD, OUTPUT);
  pinMode(READ_D_PIN_SD_CD, INPUT_PULLUP);
  pinMode(READ_D_PIN_SD_WP, INPUT_PULLUP);

  pinMode(WRITE_D_PIN_READY_LED, OUTPUT);
  pinMode(WRITE_D_PIN_ERROR_LED, OUTPUT);
}

void RtcInit()
{
  if (!_rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // _rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Uncomment to set date and time.
}

void DisplayRecordingPeriod()
{
  Serial.print("Recording Period: ");
  Serial.print(_recordingPeriod);
  Serial.println(_recordingPeriod == 1 ? " milliseconds" : " milliseconds");
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
    _errorMessage = "RTC is not running.";
  }
  else if (sdCardIn == HIGH)  // CD and WP pins are open drain and normal LOW/HIGH logic is reversed.
  {
    if (_logfile)
    {
      _logfile.close();  
    }
    
    _statusError = true;
    _errorMessage = "SD card is not inserted.";
  }
  else if (sdCardIn == LOW && sdCardWriteProtect == HIGH)  // CD and WP pins are open drain and normal LOW/HIGH logic is reversed.
  {
    _statusError = true;
    _errorMessage = "SD card is write protected.";
  }
  else if (!_logfile)
  {
    _statusError = true;
    _errorMessage = "Log file error.";
  }
  else  // No error.
  {
    _statusError = false;
    _errorMessage = "";
  }
}

void DetermineStatusReady(int sdCardIn, int sdCardWriteProtect)
{
  // CD and WP pins are open drain and normal LOW/HIGH logic is reversed.
  _statusReady = !_statusError && sdCardIn == LOW && sdCardWriteProtect == LOW;
}

void UpdateStatusLeds()
{
  int statusReadyWrite = _statusReady && !_flash ? HIGH : LOW;
  int statusErrorWrite = _statusError ? HIGH : LOW;
  
  digitalWrite(WRITE_D_PIN_READY_LED, statusReadyWrite);
  digitalWrite(WRITE_D_PIN_ERROR_LED, statusErrorWrite);
}

String GetTimestamp()
{
  DateTime now  = _rtc.now();

  String month  = now.month()  < 10 ? "0" + String(now.month())  : String(now.month());
  String day    = now.day()    < 10 ? "0" + String(now.day())    : String(now.day());
  String hour   = now.hour()   < 10 ? "0" + String(now.hour())   : String(now.hour());
  String minute = now.minute() < 10 ? "0" + String(now.minute()) : String(now.minute());
  String second = now.second() < 10 ? "0" + String(now.second()) : String(now.second());

  return String(now.year()) + "-" + month + "-" + day + " " + hour + ":" + minute + ":" + second;
}

void SdCardWrite(String timestamp, float value)
{
  _logfile.print(timestamp);
  _logfile.print(", ");
  _logfile.print(value);

  _logfile.flush();  // Don't do this too frequently, i.e. < 1s

  _flash = !_first;  // Don't flash the LED the first time.
  _lastMillisFlash = _currentMillisFlash;
}

void UpdateSerial(String timestamp, float value)
{
  if (_statusReady)
  {
    Serial.print(timestamp);
    Serial.print(", ");
    Serial.print(value);
    Serial.println();
  }
  else if (_statusError)
  {
    Serial.print(timestamp);
    Serial.print(" ERROR: ");
    Serial.print(_errorMessage);
    Serial.println();
  }
  else
  {
    Serial.print(timestamp);
    Serial.print(" UNKNOWN STATUS");
    Serial.println();
  }
}
