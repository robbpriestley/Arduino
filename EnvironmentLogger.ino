// https://learn.adafruit.com/adafruit_data_logger_shield/light_and_temperature_logger_use_it
// https://learn.adafruit.com/adafruit_data_logger_shield/using_the_real_time_clock_2
// https://cdn-learn.adafruit.com/downloads/pdf/adafruit-data-logger-shield.pdf

#include <SD.h>
#include <SPI.h>
#include "RTClib.h"

File _logfile;
RTC_PCF8523 _rtc;

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

  String errorMessage;
  bool statusError = DetermineStatusError(sdCardIn, sdCardWriteProtect, errorMessage);
  bool statusReady = !statusError && DetermineStatusReady(sdCardIn, sdCardWriteProtect);

  UpdateStatusLeds(statusReady, statusError);

  float temperature = 100.0;
  float humidity = 200.0;
  float pressure = 300.0;
  
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
  UpdateSerial(timestamp, statusReady, statusError, errorMessage, temperature, humidity, pressure);

  delay(_recordingPeriod);
}

void SdInit()
{
  if (!SD.begin(READ_D_PIN_SD_CARD))
  {
    Serial.print("SD card failed, or not present");
  }

  // Create a new file.
  
  char filename[] = "LOGGER00.CSV";
  
  for (uint8_t i = 0; i < 100; i++)
  {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    
    if (! SD.exists(filename))
    {
      _logfile = SD.open(filename, FILE_WRITE);  // Only open a new file if it doesn't exist.
      break;
    }
  }
  
  if (!_logfile)
  {
    Serial.print("ERROR: Log file couldn't be created ");
    Serial.println(filename);
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
    Serial.println("ERROR: Couldn't find RTC");
    while (1);
  }

  // _rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Uncomment to set date and time.

  if (!_rtc.initialized())
  {
    Serial.println("ERROR: RTC is NOT running!");
  }
}

void DisplayRecordingPeriod()
{
  Serial.print("Recording Period: ");
  Serial.print(_recordingPeriod);
  Serial.print(" ");
  Serial.println(RECORDING_PERIOD_UNITS);
}

bool DetermineStatusError(int sdCardIn, int sdCardWriteProtect, String& errorMessage)
{
  // CD and WP pins are open drain and normal LOW/HIGH logic is reversed.
  bool statusError = sdCardIn == LOW && sdCardWriteProtect == HIGH;

  if (!statusError)
  {
    errorMessage = "";
  }
  else if (sdCardIn == LOW && sdCardWriteProtect == HIGH)
  {
    errorMessage = "CARD WRITE PROTECTED";
  }
  else
  {
    // ... TBD
  }
  
  return statusError;
}

bool DetermineStatusReady(int sdCardIn, int sdCardWriteProtect)
{
  // CD and WP pins are open drain and normal LOW/HIGH logic is reversed.
  return sdCardIn == LOW && sdCardWriteProtect == LOW;
}

void UpdateStatusLeds(bool statusReady, bool statusError)
{
  int statusReadyWrite = statusReady ? HIGH : LOW;
  int statusErrorWrite = statusError ? HIGH : LOW;
  
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

void UpdateSerial(String timestamp, bool statusReady, bool statusError, String errorMessage, float temperature, float humidity, float pressure)
{
  if (statusReady)
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
  else if (statusError)
  {
    Serial.print("ERROR: ");
    Serial.print(errorMessage);
    Serial.println();
  }
  else
  {
    Serial.print("UNKNOWN STATUS");
    Serial.println();
  }
}
