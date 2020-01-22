// https://github.com/silveirago/the-transport
// https://www.musiconerd.com/single-post/build-this-midi-controller-the-transport

// HOW TO MAP: Use Logic Pro X → Key Commands → Edit, find command, and Learn New Assignment.

#include "MIDIUSB.h" // Download it here: https://github.com/arduino-libraries/MIDIUSB

const int NButtons = 5;
const int buttonPin[NButtons] = {2, 3 ,4 ,5, 6};

int buttonCState[NButtons] = {0};  // stores the button current value
int buttonPState[NButtons] = {0};  // stores the button previous value

unsigned long lastDebounceTime[NButtons] = {0};  // the last time the output pin was toggled
unsigned long debounceDelay = 13;                // the debounce time; increase if the output flickers

byte midiCh = 15; // MIDI channel to be used
byte note = 1;    // Lowest note to be used
byte cc = 1;      // Lowest MIDI CC to be used

// *** LED BRIGHTNESS ***

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9;  // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

void setup() 
{

    // Serial.begin(57600);

    for (int i = 0; i < NButtons; i++) 
    {
        pinMode(buttonPin[i], INPUT_PULLUP);
    }
}

void loop() {

    sensorValue = analogRead(analogInPin);
    outputValue = map(sensorValue, 0, 1023, 0, 255);
    analogWrite(analogOutPin, outputValue);
    
    buttons();

}

void buttons()
{
    for (int i = 0; i < NButtons; i++) 
    {
        buttonCState[i] = digitalRead(buttonPin[i]);
        
        if ((millis() - lastDebounceTime[i]) > debounceDelay) 
        {
        
          if (buttonPState[i] != buttonCState[i])
          {
            lastDebounceTime[i] = millis();
            
            if (buttonCState[i] == LOW)
            {
                noteOn(midiCh, note + i, 127);  // channel, note, velocity
                MidiUSB.flush();
                
                Serial.print("button on  >> ");
                Serial.println(i);
            }
            else
            {
                noteOn(midiCh, note + i, 0);  // channel, note, velocity
                MidiUSB.flush();
                
                Serial.print("button off >> ");
                Serial.println(i);
            }
            
            buttonPState[i] = buttonCState[i];
          }
        }
    }
}

void noteOn(byte channel, byte pitch, byte velocity) 
{
    midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
    MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity)
{
    midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
    MidiUSB.sendMIDI(noteOff);
}
