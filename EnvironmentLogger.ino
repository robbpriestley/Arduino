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
bool _statusInitError;
String _errorMessage;
int _recordingPeriod = 1500;
const String RECORDING_PERIOD_UNITS = " ms";

const int READ_A_PIN_TEMP = 1;
const int READ_D_PIN_LOGGER_CD = 9;
const int READ_D_PIN_LOGGER_WP = 8;
const int READ_D_PIN_SD_CARD = 10;
const int WRITE_D_PIN_READY_LED = 7;
const int WRITE_D_PIN_ERROR_LED = 6;
const float REFERENCE_VOLTAGE = 5.12;

void setup() 
{
  Serial.begin(57600);
  Serial.println("*** ENVIRONMENT LOGGER ***");
    
  PinInit();
  RtcInit();
  SdInit();

  DisplayRecordingPeriod();
}

void loop() 
{
  String timestamp = GetTimestamp();

  int sdCardIn = digitalRead(READ_D_PIN_LOGGER_CD);            // Is an SD card inserted?
  int sdCardWriteProtect = digitalRead(READ_D_PIN_LOGGER_WP);  // Is the SD card write protected?

  DetermineStatusError(sdCardIn, sdCardWriteProtect);
  DetermineStatusReady(sdCardIn, sdCardWriteProtect);

  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;
  
  if (!_statusError && !_statusInitError)
  {
    //  float tempReading = analogRead(READ_A_PIN_TEMP);
    //  float voltage = tempReading * REFERENCE_VOLTAGE / 1024;  // 1024 steps
    //  float tempCelsius = (voltage - 0.5) * 100;
    //
    //  Serial.print("tempReading: ");
    //  Serial.print(tempReading);
    //  Serial.print(", voltage: ");
    //  Serial.print(voltage);
    //  Serial.print(", tempCelsius: ");
    //  Serial.println(tempCelsius);
  
    SdCardWrite(timestamp, temperature, humidity, pressure);
  }
  
  UpdateStatusLeds();
  UpdateSerial(timestamp, temperature, humidity, pressure);

  delay(_recordingPeriod);
}

void SdInit()
{
  if (!SD.begin(READ_D_PIN_SD_CARD))
  {
    _statusInitError = true;
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
  
  if (!_logfile)
  {
    _statusInitError = true;
    _errorMessage = String("Log file couldn't be created ") + String(filename);
  }
  else
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

  if (!_rtc.initialized())
  {
    _statusInitError = true;
    _errorMessage = "RTC is NOT running!";
  }
}

void DisplayRecordingPeriod()
{
  Serial.print("Recording Period: ");
  Serial.print(_recordingPeriod);
  Serial.print(" ");
  Serial.println(RECORDING_PERIOD_UNITS);
}

void DetermineStatusError(int sdCardIn, int sdCardWriteProtect)
{
  // CD and WP pins are open drain and normal LOW/HIGH logic is reversed.
  _statusError = sdCardIn == LOW && sdCardWriteProtect == HIGH;

  if (!_statusError && !_statusInitError)
  {
    _errorMessage = "";
  }
  else if (sdCardIn == LOW && sdCardWriteProtect == HIGH)
  {
    _errorMessage = "SD card is write protected";
  }
  else
  {
    // ... TBD
  }
}

void DetermineStatusReady(int sdCardIn, int sdCardWriteProtect)
{
  // CD and WP pins are open drain and normal LOW/HIGH logic is reversed.
  _statusReady = !_statusError && sdCardIn == LOW && sdCardWriteProtect == LOW;
}

void UpdateStatusLeds()
{
  int statusReadyWrite = _statusReady ? HIGH : LOW;
  int statusErrorWrite = _statusError || _statusInitError ? HIGH : LOW;
  
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
  else if (_statusError || _statusInitError)
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
