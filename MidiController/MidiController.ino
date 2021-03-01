// https://github.com/silveirago/the-transport
// https://www.musiconerd.com/single-post/build-this-midi-controller-the-transport

// HOW TO MAP: Use Logic Pro X → Key Commands → Edit, find command, and Learn New Assignment.

#include "MIDIUSB.h" // Download it here: https://github.com/arduino-libraries/MIDIUSB

const int NButtons = 9;
const int buttonPin[NButtons] = {5, 6, 7, 8, 9, 10, 14, 15, 16};

int buttonCState[NButtons] = {0};  // stores the button current value
int buttonPState[NButtons] = {0};  // stores the button previous value

unsigned long lastDebounceTime[NButtons] = {0};  // the last time the output pin was toggled
unsigned long debounceDelay = 100;               // the debounce time in ms; increase if the output flickers

byte midiCh = 16; // MIDI channel to be used
byte note = 1;    // Lowest note to be used
byte cc = 102;    // Lowest MIDI CC to be used

void setup() 
{

    // Serial.begin(57600);

    for (int i = 0; i < NButtons; i++) 
    {
        pinMode(buttonPin[i], INPUT_PULLUP);
    }
}

void loop() {
    
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
                // NoteOn(midiCh, note + i, 127);
                ControlChange(midiCh, cc + i, 127);
                MidiUSB.flush();
                
                Serial.print("button on  >> ");
                Serial.println(i);
            }
            
            buttonPState[i] = buttonCState[i];
          }
        }
    }
}

void ControlChange(byte channel, byte control, byte value)
{
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

void NoteOn(byte channel, byte pitch, byte velocity) 
{
    midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
    MidiUSB.sendMIDI(noteOn);
}

void NoteOff(byte channel, byte pitch, byte velocity)
{
    midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
    MidiUSB.sendMIDI(noteOff);
}
