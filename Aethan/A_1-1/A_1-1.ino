/*
  Blink

  Hello

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/
const int BlueLed = 10;
const int Led_External= 13;
const int Button= 2;
const int Loud = 8;
bool ButtonDown = false;
bool LED_Power= false;

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(Led_External, OUTPUT);
  pinMode(Button, INPUT);
  pinMode(BlueLed, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {

  int buttonState = digitalRead(Button);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    
    Serial.print("ButtonDown: ");
    Serial.println(ButtonDown);
    
    ButtonDown = true;
  }
  else if (buttonState == LOW && ButtonDown == true) {
    
    LED_Power = !LED_Power;
    ButtonDown = false;

    Serial.print("LED_Power: ");
    Serial.println(LED_Power);
    Serial.print("ButtonDown: ");
    Serial.println(ButtonDown);
  }

  if (LED_Power == true) {
    // turn LED on:
    digitalWrite(Led_External, HIGH);
  } else {
    // turn LED off:
    digitalWrite(Led_External, LOW);
  }
    if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(BlueLed, HIGH);
  } else {
    // turn LED off:
    digitalWrite(BlueLed, LOW);
  }
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(Loud, HIGH);
  } else {
    // turn LED off:
    digitalWrite(Loud, LOW);
  }
}
