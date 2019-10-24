const int analogInPin = A0;
const int digitalOutPin = 7;
 
int delayMillis = 0;
int sensorValue = 0;
bool active = false;
unsigned long currentMillis = 0;
unsigned long lastMillis = 0;   

void setup() 
{
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(digitalOutPin, OUTPUT);
}

void loop() 
{  
  sensorValue = analogRead(analogInPin);
  delayMillis = map(sensorValue, 0, 1023, 0, 1000);
  
  currentMillis = millis();

  if (currentMillis - lastMillis > delayMillis)
  {
    if (active)
    {
      digitalWrite(digitalOutPin, LOW);
    }
    else
    {
      digitalWrite(digitalOutPin, HIGH);
    }

    lastMillis = currentMillis;
    active = !active;
  }

  Serial.println(currentMillis);
}
