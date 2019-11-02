#include "SR04.h"

#define TRIG_PIN 12
#define ECHO_PIN 11

const int RED_LED = 7;

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long distance;

void setup()
{
   Serial.begin(9600);
   pinMode(RED_LED, OUTPUT);
}

void loop()
{  
  distance = sr04.Distance();

  if (distance <= 290)
  {    
    if (distance <= 30)
    {
      digitalWrite(RED_LED, HIGH);
    }
    else
    {
      digitalWrite(RED_LED, LOW);
    }
    
    Serial.print(distance);
    Serial.println("cm");
  }
  
  delay(100);
}
