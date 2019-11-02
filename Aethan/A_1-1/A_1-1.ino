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

const int BLUE_LED = 10;
const int RED_LED = 13;
const int BUTTON_INPUT = 2;
const int BUZZER = 8;

bool Button_Pressed = false;
bool Red_LED_State= false;

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  pinMode(BUTTON_INPUT, INPUT);
  
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
}

// the loop function runs over and over again forever
void loop()
{
  int buttonState = digitalRead(BUTTON_INPUT);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH)
  {
    
    Serial.print("Button_Pressed: ");
    Serial.println(Button_Pressed);
    
    Button_Pressed = true;
  }
  else if (buttonState == LOW && Button_Pressed == true)
  {
    
    Red_LED_State = !Red_LED_State;
    Button_Pressed = false;

    Serial.print("Red_LED_State: ");
    Serial.println(Red_LED_State);
    Serial.print("Button_Pressed: ");
    Serial.println(Button_Pressed);
  }

  if (Red_LED_State == true)
  {
    digitalWrite(RED_LED, HIGH);
  }
  else
  {
    digitalWrite(RED_LED, LOW);
  }
  
  if (buttonState == HIGH)
  {
    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(BUZZER, HIGH);
  }
  else
  {
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(BUZZER, LOW);
  }
}
