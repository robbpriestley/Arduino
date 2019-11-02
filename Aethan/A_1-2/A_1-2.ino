// CODE BASED ON:  http://www.arduino.cc/en/Tutorial/AnalogInOutSerial

const int ANALOG_POT_IN = 0;  // Analog input pin that the potentiometer is attached to
const int RED_LED = 7;
const int BUZZER = 6;
const int BLUE_LED = 5;

int sensorValue = 0;        // value read from the pot
int FlashRate = 0;        // value in milliseconds

void setup()
{
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  pinMode(RED_LED, OUTPUT);  
  pinMode(BUZZER, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
}

void loop()
{
  // read the analog in value:
  sensorValue = analogRead(ANALOG_POT_IN);

  FlashRate = map(sensorValue, 0, 1023, 0, 1000);

  digitalWrite(RED_LED, HIGH);
  digitalWrite(BUZZER, HIGH);
  digitalWrite(BLUE_LED, LOW);
  delay(FlashRate);                       
  digitalWrite(RED_LED, LOW);    
  digitalWrite(BUZZER, LOW);
  digitalWrite(BLUE_LED, HIGH);
  delay(FlashRate);                       
 
  // print the results to the Serial Monitor:
  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print("\t output = ");
  Serial.println(FlashRate);
}
