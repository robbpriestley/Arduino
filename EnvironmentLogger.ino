// https://learn.adafruit.com/adafruit_data_logger_shield/light_and_temperature_logger_use_it
// https://learn.adafruit.com/adafruit_data_logger_shield/using_the_real_time_clock_2
// https://cdn-learn.adafruit.com/downloads/pdf/adafruit-data-logger-shield.pdf

#include <SD.h>
#include <SPI.h>
#include "RTClib.h"

File _logfile;
RTC_PCF8523 _rtc;

bool _statusReady;
bool _statusError;
String _errorMessage;

bool _flash;
int _periodFlash = 333;
unsigned long _lastMillisFlash = 0;
unsigned long _currentMillisFlash = 0;

int _periodRecord = 1500;
unsigned long _lastMillisRecord = 0;
unsigned long _currentMillisRecord = 0;
const String RECORD_PERIOD_UNITS = " ms";

const int READ_A_PIN_TEMP = 1;
const int READ_D_PIN_LOGGER_CD = 9;
const int READ_D_PIN_LOGGER_WP = 8;
const int READ_D_PIN_SD_CARD = 10;
const int WRITE_D_PIN_READY_LED = 7;
const int WRITE_D_PIN_ERROR_LED = 6;
const float REFERENCE_VOLTAGE = 5120;  // mV

void setup() 
{
  Serial.begin(57600);
  Serial.println("*** ENVIRONMENT LOGGER ***");
    
  PinInit();
  RtcInit();

  DisplayRecordingPeriod();
}

void loop() 
{
  String timestamp = GetTimestamp();

  int sdCardIn = digitalRead(READ_D_PIN_LOGGER_CD);            // Is an SD card inserted?
  int sdCardWriteProtect = digitalRead(READ_D_PIN_LOGGER_WP);  // Is the SD card write protected?

  if (sdCardIn == LOW && !_logfile)  // SD card is in but file has not been initialized.
  {
    SdInit();
  }

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
    UpdateSerial(timestamp, 0, 0, 0);
  }
  else if (_currentMillisRecord - _lastMillisRecord >= _periodRecord)
  {
    float humidity = 0.0;
    float pressure = 0.0;

    // TMP36 https://learn.adafruit.com/tmp36-temperature-sensor
    int tempReading = analogRead(READ_A_PIN_TEMP);
    float temperature = (tempReading * (REFERENCE_VOLTAGE / 1024) - 500.0) / 10.0;
  
    SdCardWrite(timestamp, temperature, humidity, pressure);
    UpdateSerial(timestamp, temperature, humidity, pressure);

    _lastMillisRecord = _currentMillisRecord;
  }
}

void SdInit()
{
  if (!SD.begin(READ_D_PIN_SD_CARD))
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
  pinMode(READ_D_PIN_SD_CARD, OUTPUT);
  pinMode(READ_D_PIN_LOGGER_CD, INPUT_PULLUP);
  pinMode(READ_D_PIN_LOGGER_WP, INPUT_PULLUP);

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
  Serial.print(_periodRecord);
  Serial.print(" ");
  Serial.println(RECORD_PERIOD_UNITS);
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

void SdCardWrite(String timestamp, float temperature, float humidity, float pressure)
{
  _logfile.print(timestamp);
  _logfile.print(", ");
  _logfile.print(temperature);
  _logfile.print(", ");
  _logfile.print(humidity);
  _logfile.print(", ");
  _logfile.print(pressure);
  _logfile.println();

  _logfile.flush();  // Don't do this too frequently, i.e. < 1s

  _flash = true;
  _lastMillisFlash = _currentMillisFlash;
}

void UpdateSerial(String timestamp, float temperature, float humidity, float pressure)
{
  if (_statusReady)
  {
    Serial.print(timestamp);
    Serial.print(", ");
    Serial.print(temperature);
    Serial.print(" °C, ");
    Serial.print(humidity);
    Serial.print(", ");
    Serial.print(pressure);
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
