/*
  Source MUX - Firmware Rev 1.7

  Includes code by:
    Dave Benn - Handling MUXs, a few other bits and original inspiration  https://www.notesandvolts.com/2019/01/teensy-synth-part-10-hardware.html

  Arduino IDE
  Tools Settings:
  Board: "Teensy3.6"
  USB Type: "Serial + MIDI + Audio"
  CPU Speed: "180"
  Optimize: "Fastest"

  Additional libraries:
    Agileware CircularBuffer available in Arduino libraries manager
    Replacement files are in the Modified Libraries folder and need to be placed in the teensy Audio folder.
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <MIDI.h>
#include <USBHost_t36.h>
#include "MidiCC.h"
#include "Constants.h"
#include "Parameters.h"
#include "PatchMgr.h"
#include "HWControls.h"
#include "EepromMgr.h"
#include "Settings.h"
#include <ShiftRegister74HC595.h>
#include <RoxMux.h>

#define PARAMETER 0      //The main page for displaying the current patch and control (parameter) changes
#define RECALL 1         //Patches list
#define SAVE 2           //Save patch page
#define REINITIALISE 3   // Reinitialise message
#define PATCH 4          // Show current patch bypassing PARAMETER
#define PATCHNAMING 5    // Patch naming page
#define DELETE 6         //Delete patch page
#define DELETEMSG 7      //Delete patch message page
#define SETTINGS 8       //Settings page
#define SETTINGSVALUE 9  //Settings page
unsigned int state = PARAMETER;
#include "ST7735Display.h"

//
// Mux values
//
int DelayForSH3 = 10;
int patchNo = 0;
unsigned long buttonDebounce = 0;
boolean cardStatus = false;

//
//USB HOST MIDI Class Compliant
//
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
MIDIDevice midi1(myusb);

//MIDI 5 Pin DIN
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);  //RX - Pin 0

//
// MIDI to CV conversion
//

bool notes[88] = { 0 };
int8_t noteOrder[80] = { 0 }, orderIndx = { 0 };
int noteMsg;
unsigned long trigTimer = 0;
#define NOTE_SF 51.57f  // This value can be tuned if CV output isn't exactly 1V/octave
#define trigTimeout 50
int transpose = 0;
int realoctave = 0;
float previousMillis = millis();  //For MIDI Clk Sync
int count = 0;                    //For MIDI Clk Sync
long earliestTime = millis();     //For voice allocation - initialise to now
int modulation;
int pitchbend;
float bend = 0;
int8_t d2, i;

//
//Shift Register setup
//
// data, clk, latch
//
ShiftRegister74HC595<6> srpanel(6, 7, 9);

// pins for 74HC595
#define BOARD_DATA 36   // pin 14 on 74HC595 (DATA)
#define BOARD_LATCH 37  // pin 12 on 74HC595 (LATCH)
#define BOARD_CLK 39    // pin 11 on 74HC595 (CLK)
#define BOARD_PWM -1    // pin 13 on 74HC595
#define SWITCH_TOTAL 3
Rox74HC595<SWITCH_TOTAL> boardswitch;

// pins for 74HC165
#define BTN_DEBOUNCE 50
#define PIN_DATA 35  // pin 9 on 74HC165 (DATA)
#define PIN_LOAD 34  // pin 1 on 74HC165 (LOAD)
#define PIN_CLK 33   // pin 2 on 74HC165 (CLK))
#define MUX_TOTAL 6
RoxOctoswitch<MUX_TOTAL, BTN_DEBOUNCE> mux;

//
// Start setup
//

void setup() {
  SPI.begin();
  setupDisplay();
  setUpSettings();
  setupHardware();

  cardStatus = SD.begin(BUILTIN_SDCARD);
  if (cardStatus) {
    Serial.println("SD card is connected");
    //Get patch numbers and names from SD card
    loadPatches();
    if (patches.size() == 0) {
      //save an initialised patch to SD card
      savePatch("1", INITPATCH);
      loadPatches();
    }
  } else {
    Serial.println("SD card is not connected or unusable");
    reinitialiseToPanel();
    showPatchPage("No SD", "conn'd / usable");
  }

  //Read MIDI Channel from EEPROM
  midiChannel = getMIDIChannel();
  Serial.println("MIDI Ch:" + String(midiChannel) + " (0 is Omni On)");

  //USB HOST MIDI Class Compliant
  delay(200);  //Wait to turn on USB Host
  myusb.begin();
  midi1.setHandleControlChange(myControlChange);
  midi1.setHandlePitchChange(myPitchBend);
  midi1.setHandleProgramChange(myProgramChange);
  midi1.setHandleNoteOff(myNoteOff);
  midi1.setHandleNoteOn(myNoteOn);
  Serial.println("USB HOST MIDI Class Compliant Listening");

  //USB Client MIDI
  usbMIDI.setHandleControlChange(myControlChange);
  usbMIDI.setHandlePitchChange(myPitchBend);
  usbMIDI.setHandleProgramChange(myProgramChange);
  usbMIDI.setHandleNoteOff(myNoteOff);
  usbMIDI.setHandleNoteOn(myNoteOn);
  usbMIDI.setHandleClock(myMIDIclock);
  usbMIDI.setHandleStart(myMIDIClockStart);
  usbMIDI.setHandleStop(myMIDIClockStop);

  Serial.println("USB Client MIDI Listening");

  //MIDI 5 Pin DIN
  MIDI.begin();
  MIDI.setHandleControlChange(myControlChange);
  MIDI.setHandlePitchBend(myPitchBend);
  MIDI.setHandleProgramChange(myProgramChange);
  MIDI.setHandleNoteOn(myNoteOn);
  MIDI.setHandleNoteOff(myNoteOff);
  MIDI.setHandleClock(myMIDIclock);
  MIDI.setHandleStart(myMIDIClockStart);
  MIDI.setHandleStop(myMIDIClockStop);

  Serial.println("MIDI In DIN Listening");

  //Read Key Tracking from EEPROM, this can be set individually by each patch.
  keyMode = getKeyMode();

  //Read Pitch Bend Range from EEPROM, this can be set individually by each patch.
  pitchBendRange = getPitchBendRange();

  //Read Mod Wheel Depth from EEPROM, this can be set individually by each patch.
  modWheelDepth = getModWheelDepth();




  //Read Encoder Direction from EEPROM
  encCW = getEncoderDir();
  level1 = 1;
  level2 = 0;
  srpanel.set(OSC2_32_LED, HIGH);
  srpanel.set(OSC2_32_LED, LOW);

  mux.begin(PIN_DATA, PIN_LOAD, PIN_CLK);
  mux.setCallback(onButtonPress);

  boardswitch.begin(BOARD_DATA, BOARD_LATCH, BOARD_CLK, BOARD_PWM);

  clocksource = getClockSource();
  oldclocksource = clocksource;
  switch (clocksource) {
    case 0:
      boardswitch.writePin(CLOCK_SOURCE, LOW);
      break;

    case 1:
      boardswitch.writePin(CLOCK_SOURCE, HIGH);
      break;
  }

  patchNo = getLastPatch();
  recallPatch(patchNo);  //Load first patch
}

void setVoltage(int dacpin, bool channel, bool gain, unsigned int mV) {
  int command = channel ? 0x9000 : 0x1000;

  command |= gain ? 0x0000 : 0x2000;
  command |= (mV & 0x0FFF);

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dacpin, LOW);
  SPI.transfer(command >> 8);
  SPI.transfer(command & 0xFF);
  digitalWrite(dacpin, HIGH);
  SPI.endTransaction();
}

void showPatchNumberButton() {
  srpanel.set(BUTTON1_LED, LOW);
  srpanel.set(BUTTON2_LED, LOW);
  srpanel.set(BUTTON3_LED, LOW);
  srpanel.set(BUTTON4_LED, LOW);
  srpanel.set(BUTTON5_LED, LOW);
  srpanel.set(BUTTON6_LED, LOW);
  srpanel.set(BUTTON7_LED, LOW);
  srpanel.set(BUTTON8_LED, LOW);
  srpanel.set(BUTTON9_LED, LOW);
  srpanel.set(BUTTON10_LED, LOW);
  srpanel.set(BUTTON11_LED, LOW);
  srpanel.set(BUTTON12_LED, LOW);
  srpanel.set(BUTTON13_LED, LOW);
  srpanel.set(BUTTON14_LED, LOW);
  srpanel.set(BUTTON15_LED, LOW);
  srpanel.set(BUTTON16_LED, LOW);
  switch (patchNo) {
    case 1:
      srpanel.set(BUTTON1_LED, HIGH);
      break;
    case 2:
      srpanel.set(BUTTON2_LED, HIGH);
      break;
    case 3:
      srpanel.set(BUTTON3_LED, HIGH);
      break;
    case 4:
      srpanel.set(BUTTON4_LED, HIGH);
      break;
    case 5:
      srpanel.set(BUTTON5_LED, HIGH);
      break;
    case 6:
      srpanel.set(BUTTON6_LED, HIGH);
      break;
    case 7:
      srpanel.set(BUTTON7_LED, HIGH);
      break;
    case 8:
      srpanel.set(BUTTON8_LED, HIGH);
      break;
    case 9:
      srpanel.set(BUTTON9_LED, HIGH);
      break;
    case 10:
      srpanel.set(BUTTON10_LED, HIGH);
      break;
    case 11:
      srpanel.set(BUTTON11_LED, HIGH);
      break;
    case 12:
      srpanel.set(BUTTON12_LED, HIGH);
      break;
    case 13:
      srpanel.set(BUTTON13_LED, HIGH);
      break;
    case 14:
      srpanel.set(BUTTON14_LED, HIGH);
      break;
    case 15:
      srpanel.set(BUTTON15_LED, HIGH);
      break;
    case 16:
      srpanel.set(BUTTON16_LED, HIGH);
      break;
  }
}

void myMIDIclock() {

  if (millis() > clock_timeout + 300) clock_count = 0;  // Prevents Clock from starting in between quarter notes after clock is restarted!
  clock_timeout = millis();

  if (clock_count == 0) {
    //Serial.println("Starting pulse");
    digitalWrite(CLOCK, HIGH);  // Start clock pulse
    clock_timer = millis();
  }
  clock_count++;

  if (clock_count == 24) {  // MIDI timing clock sends 24 pulses per quarter note.  Sent pulse only once every 24 pulses
    clock_count = 0;
  }
}

void myMIDIClockStart() {
  MIDIClkSignal = true;
}

void myMIDIClockStop() {
  MIDIClkSignal = false;
}


void stopClockPulse() {
  //Serial.println("Stopping pulse");
  if ((clock_timer > 0) && (millis() - clock_timer > 20)) {
    digitalWrite(CLOCK, LOW);  // Set clock pulse low after 20 msec
    clock_timer = 0;
  }
}

void myPitchBend(byte channel, int bend) {
  if ((midiChannel == channel) || (channel == 0)) {
    // Pitch bend output from 0 to 1023 mV.  Left shift d2 by 4 to scale from 0 to 2047.
    // With DAC gain = 1X, this will yield a range from 0 to 1023 mV.  Additional amplification
    // after DAC will rescale to -1 to +1V.
    d2 = midi1.getData2();  // d2 from 0 to 127, mid point = 64;
    pitchbend = bend;
    //setVoltage(PITCH_DAC, PITCH_AB, 1, int((bend / 8) + 1024)); // DAC7, channel 0, gain = 1X
  }
}

void myControlChange(byte channel, byte number, byte value) {
  if ((midiChannel == channel || channel == 0)) {
    d2 = MIDI.getData2();
    // CC range from 0 to 2047 mV  Left shift d2 by 5 to scale from 0 to 2047,
    // and choose gain = 2X
    modulation = value;
    //setVoltage(CC_DAC, CC_AB, 1, value << 4); // DAC7, channel 1, gain = 1X
  }
}

void commandTopNote() {
  int topNote = 0;
  bool noteActive = false;
  for (int i = 0; i < 88; i++) {
    if (notes[i]) {
      topNote = i;
      noteActive = true;
    }
  }
  if (noteActive)
    commandNote(topNote);
  else  // All notes are off, turn off gate
    digitalWrite(GATE_NOTE1, LOW);
  gatepulse = 0;
}

void commandBottomNote() {
  int bottomNote = 0;
  bool noteActive = false;
  for (int i = 87; i >= 0; i--) {
    if (notes[i]) {
      bottomNote = i;
      noteActive = true;
    }
  }
  if (noteActive)
    commandNote(bottomNote);
  else  // All notes are off, turn off gate
    digitalWrite(GATE_NOTE1, LOW);
  gatepulse = 0;
}

void commandLastNote() {
  int8_t noteIndx;
  for (int i = 0; i < 80; i++) {
    noteIndx = noteOrder[mod(orderIndx - i, 80)];
    if (notes[noteIndx]) {
      commandNote(noteIndx);
      return;
    }
  }
  digitalWrite(GATE_NOTE1, LOW);  // All notes are off
  gatepulse = 0;
}

void commandNote(int noteMsg) {
  unsigned int CV = (unsigned int)((float)(noteMsg + transpose + realoctave) * NOTE_SF * 1.0 + 0.5);

  analogWrite(A21, CV);

  if (gatepulse == 0 && single == 1) {
    digitalWrite(TRIG_NOTE1, HIGH);
  }
  if (multi == 1) {
    digitalWrite(TRIG_NOTE1, HIGH);
  }
  digitalWrite(GATE_NOTE1, HIGH);
  gatepulse = 1;
  trigTimer = millis();
  while (millis() < trigTimer + trigTimeout) {
    // wait 50 milliseconds
  }
  digitalWrite(TRIG_NOTE1, LOW);
}


void myNoteOn(byte channel, byte note, byte velocity) {
  noteMsg = note;

  if (velocity == 0) {
    notes[noteMsg] = false;
  } else {
    notes[noteMsg] = true;
  }
  unsigned int velCV = ((unsigned int)((float)velocity) * 22);

  analogWrite(A22, velCV);

  if (keyMode == 4) {  // Highest note priority
    commandTopNote();
  } else if (keyMode == 5) {  // Lowest note priority
    commandBottomNote();
  } else {                 // Last note priority
    if (notes[noteMsg]) {  // If note is on and using last note priority, add to ordered list
      orderIndx = (orderIndx + 1) % 40;
      noteOrder[orderIndx] = noteMsg;
    }
    commandLastNote();
  }
}


void myNoteOff(byte channel, byte note, byte velocity) {
  noteMsg = note;

  //if (velocity == 0) {
  notes[noteMsg] = false;
  //} else {
  //  notes[noteMsg] = true;
  //}

  // Pins NP_SEL1 and NP_SEL2 indictate note priority
  unsigned int velmV = ((unsigned int)((float)velocity) * 1.25);
  setVoltage(DAC_NOTE1, 1, 1, velmV << 4);
  if (keyMode == 4) {  // Highest note priority
    commandTopNote();
  } else if (keyMode == 5) {  // Lowest note priority
    commandBottomNote();
  } else {                 // Last note priority
    if (notes[noteMsg]) {  // If note is on and using last note priority, add to ordered list
      orderIndx = (orderIndx + 1) % 40;
      noteOrder[orderIndx] = noteMsg;
    }
    commandLastNote();
  }
}

void allNotesOff() {
  digitalWrite(GATE_NOTE1, LOW);
  gatepulse = 0;
}

void firstNoteOff() {
  digitalWrite(GATE_NOTE1, LOW);
  gatepulse = 0;
}

void updateosc1_32() {
  if (osc1_32 == 1) {
    showCurrentParameterPage("Osc1 Footage", "32 Foot");
    osc1foot = 0;
    srpanel.set(OSC1_32_LED, HIGH);  // LED on
    srpanel.set(OSC1_16_LED, LOW);   // LED off
    srpanel.set(OSC1_8_LED, LOW);    // LED off
  }
}

void updateosc1_16() {
  if (osc1_16 == 1) {
    showCurrentParameterPage("Osc1 Footage", "16 Foot");
    osc1foot = 2012;
    srpanel.set(OSC1_32_LED, LOW);   // LED on
    srpanel.set(OSC1_16_LED, HIGH);  // LED off
    srpanel.set(OSC1_8_LED, LOW);    // LED off
  }
}

void updateosc1_8() {
  if (osc1_8 == 1) {
    showCurrentParameterPage("Osc1 Footage", "8 Foot");
    osc1foot = 4024;
    srpanel.set(OSC1_32_LED, LOW);  // LED on
    srpanel.set(OSC1_16_LED, LOW);  // LED off
    srpanel.set(OSC1_8_LED, HIGH);  // LED off
  }
}

void updateosc1_saw() {
  if (osc1_saw == 1) {
    showCurrentParameterPage("Osc1 Wave", "Sawtooth");
    srpanel.set(OSC1_SAW_LED, HIGH);
    srpanel.set(OSC1_TRI_LED, LOW);
    srpanel.set(OSC1_PULSE, LOW);
    boardswitch.writePin(OSC1_WAVE1, LOW);
    boardswitch.writePin(OSC1_WAVE2, LOW);
  }
}

void updateosc1_tri() {
  if (osc1_tri == 1) {
    showCurrentParameterPage("Osc1 Wave", "Triangle");
    srpanel.set(OSC1_SAW_LED, LOW);   // LED on
    srpanel.set(OSC1_TRI_LED, HIGH);  // LED off
    srpanel.set(OSC1_PULSE, LOW);     // LED off
    boardswitch.writePin(OSC1_WAVE1, HIGH);
    boardswitch.writePin(OSC1_WAVE2, LOW);
  }
}

void updateosc1_pulse() {
  if (osc1_pulse == 1) {
    showCurrentParameterPage("Osc1 Wave", "Pulse");
    srpanel.set(OSC1_SAW_LED, LOW);  // LED on
    srpanel.set(OSC1_TRI_LED, LOW);  // LED off
    srpanel.set(OSC1_PULSE, HIGH);   // LED off
    boardswitch.writePin(OSC1_WAVE1, LOW);
    boardswitch.writePin(OSC1_WAVE2, HIGH);
  }
}

void updatesingle() {
  if (single == 1) {
    showCurrentParameterPage("Single Trigger", "On");
    srpanel.set(SINGLE_TRIG_LED, HIGH);   // LED on
    srpanel.set(MULTIPLE_TRIG_LED, LOW);  // LED off
  }
}

void updatemulti() {
  if (multi == 1) {
    showCurrentParameterPage("Multi Trigger", "On");
    srpanel.set(MULTIPLE_TRIG_LED, HIGH);  // LED on
    srpanel.set(SINGLE_TRIG_LED, LOW);     // LED off
  }
}

void updatelfoTriangle() {
  if (lfoTriangle == 1) {
    showCurrentParameterPage("LFO Waveform", "Triangle");
    LfoWave = 400;
    srpanel.set(LFO_TRIANGLE_LED, HIGH);  // LED on
    srpanel.set(LFO_SQUARE_LED, LOW);     // LED off
  }
}

void updatelfoSquare() {
  if (lfoSquare == 1) {
    showCurrentParameterPage("LFO Waveform", "Square");
    LfoWave = 300;
    srpanel.set(LFO_TRIANGLE_LED, LOW);
    srpanel.set(LFO_SQUARE_LED, HIGH);
  }
}

void updatesyncOff() {
  if (syncOff == 1) {
    showCurrentParameterPage("Oscillator Sync", "Off");
    boardswitch.writePin(PB_OSC1, LOW);    // pb osc1 on
    boardswitch.writePin(PB_OSC2, LOW);    // pb osc2 on
    boardswitch.writePin(SYNC, LOW);       // sync off
    boardswitch.writePin(SOFT_SYNC, LOW);  // soft sync off
    boardswitch.writePin(HARD_SYNC, LOW);  // hard sync off
    srpanel.set(SYNC_OFF_LED, HIGH);
    srpanel.set(SYNC_ON_LED, LOW);
  }
}

void updatesyncOn() {
  if (syncOn == 1) {
    showCurrentParameterPage("Oscillator Sync", "On");
    boardswitch.writePin(PB_OSC1, LOW);     // pb osc1 off
    boardswitch.writePin(PB_OSC2, HIGH);    // pb osc2 on
    boardswitch.writePin(SYNC, HIGH);       // sync on
    boardswitch.writePin(SOFT_SYNC, HIGH);  // soft sync on
    boardswitch.writePin(HARD_SYNC, HIGH);  // hard sync off
    srpanel.set(SYNC_OFF_LED, LOW);
    srpanel.set(SYNC_ON_LED, HIGH);
  }
}

void updateoctave0() {
  if (octave0 == 1) {
    showCurrentParameterPage("KBD Octave", "0");
    boardswitch.writePin(OCTAVE, LOW);  // LED on
    srpanel.set(OCTAVE_0_LED, HIGH);
    srpanel.set(OCTAVE_1_LED, LOW);
  }
}

void updateoctave1() {
  if (octave1 == 1) {
    showCurrentParameterPage("KBD Octave", "+1");
    boardswitch.writePin(OCTAVE, HIGH);  // LED on
    srpanel.set(OCTAVE_0_LED, LOW);
    srpanel.set(OCTAVE_1_LED, HIGH);
  }
}

void updatekbOff() {
  if (kbOff == 1) {
    showCurrentParameterPage("KBD Tracking", "Off");
    boardswitch.writePin(KEYTRACK1, LOW);
    boardswitch.writePin(KEYTRACK2, LOW);
    srpanel.set(KB_OFF_LED, HIGH);
    srpanel.set(KB_HALF_LED, LOW);
    srpanel.set(KB_FULL_LED, LOW);
  }
}

void updatekbHalf() {
  if (kbHalf == 1) {
    showCurrentParameterPage("KBD Tracking", "Half");
    boardswitch.writePin(KEYTRACK1, LOW);
    boardswitch.writePin(KEYTRACK2, HIGH);
    srpanel.set(KB_OFF_LED, LOW);
    srpanel.set(KB_HALF_LED, HIGH);
    srpanel.set(KB_FULL_LED, LOW);
  }
}

void updatekbFull() {
  if (kbFull == 1) {
    showCurrentParameterPage("KBD Tracking", "Full");
    boardswitch.writePin(KEYTRACK1, HIGH);
    boardswitch.writePin(KEYTRACK2, HIGH);
    srpanel.set(KB_OFF_LED, LOW);
    srpanel.set(KB_HALF_LED, LOW);
    srpanel.set(KB_FULL_LED, HIGH);
  }
}

void updateosc2_32() {
  if (osc2_32 == 1) {
    showCurrentParameterPage("Osc2 Footage", "32 Foot");
    osc2foot = 0;
    srpanel.set(OSC2_32_LED, HIGH);
    srpanel.set(OSC2_16_LED, LOW);
    srpanel.set(OSC2_8_LED, LOW);
  }
}

void updateosc2_16() {
  if (osc2_16 == 1) {
    showCurrentParameterPage("Osc2 Footage", "16 Foot");
    osc2foot = 2024;
    srpanel.set(OSC2_32_LED, LOW);
    srpanel.set(OSC2_16_LED, HIGH);
    srpanel.set(OSC2_8_LED, LOW);
  }
}

void updateosc2_8() {
  if (osc2_8 == 1) {
    showCurrentParameterPage("Osc2 Footage", "8 Foot");
    osc2foot = 4048;
    srpanel.set(OSC2_32_LED, LOW);
    srpanel.set(OSC2_16_LED, LOW);
    srpanel.set(OSC2_8_LED, HIGH);
  }
}

void updateosc2_saw() {
  if (osc2_saw == 1) {
    showCurrentParameterPage("Osc2 Wave", "Sawtooth");
    srpanel.set(OSC2_SAW_LED, HIGH);
    srpanel.set(OSC2_TRI_LED, LOW);
    srpanel.set(OSC2_PULSE_LED, LOW);
    boardswitch.writePin(OSC2_WAVE1, LOW);
    boardswitch.writePin(OSC2_WAVE2, LOW);
  }
}

void updateosc2_tri() {
  if (osc2_tri == 1) {
    showCurrentParameterPage("Osc2 Wave", "Triangle");
    srpanel.set(OSC2_SAW_LED, LOW);
    srpanel.set(OSC2_TRI_LED, HIGH);
    srpanel.set(OSC2_PULSE_LED, LOW);
    boardswitch.writePin(OSC2_WAVE1, HIGH);
    boardswitch.writePin(OSC2_WAVE2, LOW);
  }
}

void updateosc2_pulse() {
  if (osc2_pulse == 1) {
    showCurrentParameterPage("Osc2 Wave", "On");
    srpanel.set(OSC2_SAW_LED, LOW);
    srpanel.set(OSC2_TRI_LED, LOW);
    srpanel.set(OSC2_PULSE_LED, HIGH);
    boardswitch.writePin(OSC2_WAVE1, LOW);
    boardswitch.writePin(OSC2_WAVE2, HIGH);
  }
}

void updatelfoOscOff() {
  if (lfoOscOff == 1) {
    showCurrentParameterPage("LFO to Osc", "Off");
    boardswitch.writePin(LFO_TO_OSC, LOW);
    srpanel.set(LFO_OSC_OFF_LED, HIGH);
    srpanel.set(LFO_OSC_ON_LED, LOW);
  }
}

void updatelfoOscOn() {
  if (lfoOscOn == 1) {
    showCurrentParameterPage("LFO to Osc", "On");
    boardswitch.writePin(LFO_TO_OSC, HIGH);
    srpanel.set(LFO_OSC_OFF_LED, LOW);
    srpanel.set(LFO_OSC_ON_LED, HIGH);
  }
}

void updatelfoVCFOff() {
  if (lfoVCFOff == 1) {
    showCurrentParameterPage("LFO to VCF", "Off");
    boardswitch.writePin(LFO_TO_OSC, LOW);
    srpanel.set(LFO_VCF_OFF_LED, HIGH);
    srpanel.set(LFO_VCF_ON_LED, LOW);
  }
}

void updatelfoVCFOn() {
  if (lfoVCFOn == 1) {
    showCurrentParameterPage("LFO to VCF", "On");
    boardswitch.writePin(LFO_TO_OSC, HIGH);
    srpanel.set(LFO_VCF_OFF_LED, LOW);
    srpanel.set(LFO_VCF_ON_LED, HIGH);
  }
}

void updatelevel1() {
  if (level1 == 1)
    level2 = 0;
  {
    showCurrentParameterPage("Level 1", "Selected");
    srpanel.set(LEVEL1_LED, HIGH);
    srpanel.set(LEVEL2_LED, LOW);

    showPatchNumberButton();
  }
}

void updatelevel2() {
  if (level2 == 1) {
    showCurrentParameterPage("Level 2", "Selected");
    srpanel.set(LEVEL1_LED, LOW);
    srpanel.set(LEVEL2_LED, HIGH);

    srpanel.set(BUTTON1_LED, LOW);
    srpanel.set(BUTTON2_LED, LOW);
    srpanel.set(BUTTON3_LED, LOW);
    srpanel.set(BUTTON4_LED, LOW);
    srpanel.set(BUTTON5_LED, LOW);
    srpanel.set(BUTTON6_LED, LOW);
    srpanel.set(BUTTON7_LED, LOW);
    srpanel.set(BUTTON8_LED, LOW);
    srpanel.set(BUTTON9_LED, LOW);
    srpanel.set(BUTTON10_LED, LOW);
    srpanel.set(BUTTON11_LED, LOW);
    srpanel.set(BUTTON12_LED, LOW);
    srpanel.set(BUTTON13_LED, LOW);
    srpanel.set(BUTTON14_LED, LOW);
    srpanel.set(BUTTON15_LED, LOW);
    srpanel.set(BUTTON16_LED, LOW);
    level1 = 0;

    updateshvco();
    updateshvcf();
    updatevcfVelocity();
    updatevcaVelocity();
    updatevcfLoop();
    updatevcaLoop();
    updatevcfLinear();
    updatevcaLinear();
  }
}

void updateshvco() {
  if (shvco == 1) {
    boardswitch.writePin(SH_TO_VCO, HIGH);
    if (level2 == 1) {
      srpanel.set(BUTTON10_LED, HIGH);
    }
  } else {
    boardswitch.writePin(SH_TO_VCO, LOW);
    srpanel.set(BUTTON10_LED, LOW);
  }
}

void updateshvcf() {
  if (shvcf == 1) {
    boardswitch.writePin(SH_TO_VCF, HIGH);
    if (level2 == 1) {
      srpanel.set(BUTTON11_LED, HIGH);
    }
  } else {
    boardswitch.writePin(SH_TO_VCF, LOW);
    srpanel.set(BUTTON11_LED, LOW);
  }
}

void updatevcfVelocity() {
  if (vcfVelocity == 1) {
    boardswitch.writePin(VCF_VELOCITY, HIGH);
    if (level2 == 1) {
      srpanel.set(BUTTON3_LED, HIGH);
    }
  } else {
    boardswitch.writePin(VCF_VELOCITY, LOW);
    srpanel.set(BUTTON3_LED, LOW);
  }
}

void updatevcaVelocity() {
  if (vcaVelocity == 1) {
    boardswitch.writePin(VCA_LOG_LIN, HIGH);
    if (level2 == 1) {
      srpanel.set(BUTTON4_LED, HIGH);
    }
  } else {
    boardswitch.writePin(VCA_LOG_LIN, LOW);
    srpanel.set(BUTTON4_LED, LOW);
  }
}

void updatevcfLoop() {
  if (vcfLoop == 1) {
    boardswitch.writePin(VCF_LOOP, HIGH);
    if (level2 == 1) {
      srpanel.set(BUTTON5_LED, HIGH);
    }
  } else {
    boardswitch.writePin(VCF_LOOP, LOW);
    srpanel.set(BUTTON5_LED, LOW);
  }
}

void updatevcaLoop() {
  if (vcaLoop == 1) {
    boardswitch.writePin(VCA_LOOP, HIGH);
    if (level2 == 1) {
      srpanel.set(BUTTON6_LED, HIGH);
    }
  } else {
    boardswitch.writePin(VCA_LOOP, LOW);
    srpanel.set(BUTTON6_LED, LOW);
  }
}

void updatevcfLinear() {
  if (vcfLinear == 1) {
    boardswitch.writePin(VCF_LOG_LIN, HIGH);
    if (level2 == 1) {
      srpanel.set(BUTTON7_LED, HIGH);
    }
  } else {
    boardswitch.writePin(VCF_LOG_LIN, LOW);
    srpanel.set(BUTTON7_LED, LOW);
  }
}

void updatevcaLinear() {
  if (vcaLinear == 1) {
    boardswitch.writePin(VCA_LOG_LIN, HIGH);
    if (level2 == 1) {
      srpanel.set(BUTTON8_LED, HIGH);
    }
  } else {
    boardswitch.writePin(VCA_LOG_LIN, LOW);
    srpanel.set(BUTTON8_LED, LOW);
  }
}

void setPatchButton(int patchNo) {
  state = PATCH;
  recallPatch(patchNo);
  showPatchNumberButton();
  state = PARAMETER;
}

void updatebutton1() {
  if (level2 == 1 && button1switch == 1) {
    srpanel.set(BUTTON1_LED, HIGH);
    srpanel.set(BUTTON2_LED, LOW);
    button2switch = 0;
    state = SETTINGS;
    settings::reset_settings();
    settings::increment_setting();
    settings::increment_setting();
    settings::increment_setting();
    showSettingsPage();
  }
  if (level2 == 1 && button1switch == 0) {
    srpanel.set(BUTTON1_LED, LOW);
    state = PARAMETER;
  }
  if (level1 == 1) {
    patchNo = 1;
    setPatchButton(patchNo);
  }
}

void updatebutton2() {
  if (level2 == 1 && button2switch == 1) {
    srpanel.set(BUTTON2_LED, HIGH);
    srpanel.set(BUTTON1_LED, LOW);
    button1switch = 0;
    state = SETTINGS;
    settings::reset_settings();
    settings::increment_setting();
    settings::increment_setting();
    settings::increment_setting();
    settings::increment_setting();
    showSettingsPage();
  }
  if (level2 == 1 && button2switch == 0) {
    srpanel.set(BUTTON2_LED, LOW);
    state = PARAMETER;
  }
  if (level1 == 1) {
    patchNo = 2;
    setPatchButton(patchNo);
  }
}

void turnOffOneandTwo() {
  if (button1switch) {
    srpanel.set(BUTTON1_LED, LOW);
    button1switch = 0;
  }
  if (button2switch) {
    srpanel.set(BUTTON2_LED, LOW);
    button2switch = 0;
  }
}

void updatebutton3() {
  if (level2 == 1 && button3switch == 1) {
    showCurrentParameterPage("VCF Vel", "On");
    vcfVelocity = 1;
    srpanel.set(BUTTON3_LED, HIGH);
    turnOffOneandTwo();
    boardswitch.writePin(VCF_VELOCITY, HIGH);
  }
  if (level2 == 1 && button3switch == 0) {
    showCurrentParameterPage("VCF Vel", "Off ");
    vcfVelocity = 0;
    srpanel.set(BUTTON3_LED, LOW);
    turnOffOneandTwo();
    boardswitch.writePin(VCF_VELOCITY, LOW);
  }
  if (level1 == 1) {
    patchNo = 3;
    setPatchButton(patchNo);
  }
}

void updatebutton4() {
  if (level2 == 1 && button4switch == 1) {
    showCurrentParameterPage("VCA Vel", "On");
    vcaVelocity = 1;
    srpanel.set(BUTTON4_LED, HIGH);
    turnOffOneandTwo();
    boardswitch.writePin(VCA_LOG_LIN, HIGH);
  }
  if (level2 == 1 && button4switch == 0) {
    showCurrentParameterPage("VCA Vel", "Off ");
    vcaVelocity = 0;
    srpanel.set(BUTTON4_LED, LOW);
    turnOffOneandTwo();
    boardswitch.writePin(VCA_LOG_LIN, LOW);
  }
  if (level1 == 1) {
    patchNo = 4;
    setPatchButton(patchNo);
  }
}

void updatebutton5() {
  if (level2 == 1 && button5switch == 1) {
    showCurrentParameterPage("VCF Loop", "On");
    vcfLoop = 1;
    srpanel.set(BUTTON5_LED, HIGH);
    turnOffOneandTwo();
    boardswitch.writePin(VCF_LOOP, HIGH);
  }
  if (level2 == 1 && button5switch == 0) {
    showCurrentParameterPage("VCF Loop", "Off ");
    vcfLoop = 0;
    srpanel.set(BUTTON5_LED, LOW);
    turnOffOneandTwo();
    boardswitch.writePin(VCF_LOOP, LOW);
  }
  if (level1 == 1) {
    patchNo = 5;
    setPatchButton(patchNo);
  }
}

void updatebutton6() {
  if (level2 == 1 && button6switch == 1) {
    showCurrentParameterPage("VCA Loop", "On");
    vcaLoop = 1;
    srpanel.set(BUTTON6_LED, HIGH);
    turnOffOneandTwo();
    boardswitch.writePin(VCA_LOOP, HIGH);
  }
  if (level2 == 1 && button6switch == 0) {
    showCurrentParameterPage("VCA Loop", "Off ");
    vcaLoop = 0;
    srpanel.set(BUTTON6_LED, LOW);
    turnOffOneandTwo();
    boardswitch.writePin(VCA_LOOP, LOW);
  }
  if (level1 == 1) {
    patchNo = 6;
    setPatchButton(patchNo);
  }
}

void updatebutton7() {
  if (level2 == 1 && button7switch == 1) {
    showCurrentParameterPage("VCF Lin EG", "On");
    vcfLinear = 1;
    srpanel.set(BUTTON7_LED, HIGH);
    turnOffOneandTwo();
    boardswitch.writePin(VCF_LOG_LIN, HIGH);
  }
  if (level2 == 1 && button7switch == 0) {
    showCurrentParameterPage("VCF Lin EG", "Off ");
    vcfLinear = 0;
    srpanel.set(BUTTON7_LED, LOW);
    turnOffOneandTwo();
    boardswitch.writePin(VCF_LOG_LIN, LOW);
  }
  if (level1 == 1) {
    patchNo = 7;
    setPatchButton(patchNo);
  }
}

void updatebutton8() {
  if (level2 == 1 && button8switch == 1) {
    showCurrentParameterPage("VCA Lin EG", "On");
    vcaLinear = 1;
    srpanel.set(BUTTON8_LED, HIGH);
    turnOffOneandTwo();
    boardswitch.writePin(VCA_LOG_LIN, HIGH);
  }
  if (level2 == 1 && button8switch == 0) {
    showCurrentParameterPage("VCA Lin EG", "Off ");
    vcaLinear = 0;
    srpanel.set(BUTTON8_LED, LOW);
    turnOffOneandTwo();
    boardswitch.writePin(VCA_LOG_LIN, LOW);
  }
  if (level1 == 1) {
    patchNo = 8;
    setPatchButton(patchNo);
  }
}

void updatebutton9() {
  if (level2 == 1) {
    showCurrentParameterPage("Level 2", "No Function");
    turnOffOneandTwo();
  }
  if (level1 == 1) {
    patchNo = 9;
    setPatchButton(patchNo);
  }
}

void updatebutton10() {
  if (level2 == 1 && button10switch == 1) {
    showCurrentParameterPage("Sample & Hold", "To VCO");
    srpanel.set(BUTTON10_LED, HIGH);
    turnOffOneandTwo();
    boardswitch.writePin(SH_TO_VCO, HIGH);
  }
  if (level2 == 1 && button10switch == 0) {
    showCurrentParameterPage("Sample & Hold", "Off ");
    srpanel.set(BUTTON10_LED, LOW);
    turnOffOneandTwo();
    boardswitch.writePin(SH_TO_VCO, LOW);
  }
  if (level1 == 1) {
    patchNo = 10;
    setPatchButton(patchNo);
  }
}

void updatebutton11() {
  if (level2 == 1 && button11switch == 1) {
    showCurrentParameterPage("Sample & Hold", "To VCF ");
    srpanel.set(BUTTON11_LED, HIGH);
    turnOffOneandTwo();
    boardswitch.writePin(SH_TO_VCF, HIGH);
  }
  if (level2 == 1 && button11switch == 0) {
    showCurrentParameterPage("Sample & Hold", "Off ");
    srpanel.set(BUTTON11_LED, LOW);
    turnOffOneandTwo();
    boardswitch.writePin(SH_TO_VCF, LOW);
  }
  if (level1 == 1) {
    patchNo = 11;
    setPatchButton(patchNo);
  }
}

void updatebutton12() {
  if (level2 == 1) {
    showCurrentParameterPage("Level 2", "No Function");
    turnOffOneandTwo();
  }
  if (level1 == 1) {
    patchNo = 12;
    setPatchButton(patchNo);
  }
}

void updatebutton13() {
  if (level2 == 1) {
    showCurrentParameterPage("Level 2", "No Function");
    turnOffOneandTwo();
  }
  if (level1 == 1) {
    patchNo = 13;
    setPatchButton(patchNo);
  }
}

void updatebutton14() {
  if (level2 == 1) {
    showCurrentParameterPage("Level 2", "No Function");
    turnOffOneandTwo();
  }
  if (level1 == 1) {
    patchNo = 14;
    setPatchButton(patchNo);
  }
}

void updatebutton15() {
  if (level2 == 1) {
    showCurrentParameterPage("Level 2", "No Function");
    turnOffOneandTwo();
  }
  if (level1 == 1) {
    patchNo = 15;
    setPatchButton(patchNo);
  }
}

void updatebutton16() {
  if (level2 == 1) {
    showCurrentParameterPage("Level 2", "No Function");
    turnOffOneandTwo();
  }
  if (level1 == 1) {
    patchNo = 16;
    setPatchButton(patchNo);
  }
}

void updatevolume() {
  showCurrentParameterPage("Volume", int(volumestr));
}

void updatefilterRes() {
  showCurrentParameterPage("Resonance", int(filterResstr));
}

void updatefilterLevel() {
  showCurrentParameterPage("Contour Amt", int(filterLevelstr));
}

void updateglide() {
  showCurrentParameterPage("Glide", int(glidestr));
}

void updateNoiseLevel() {
  showCurrentParameterPage("Noise Level", int(noiseLevelstr));
}

void updateFilterCutoff() {
  showCurrentParameterPage("Cutoff", String(filterCutoffstr) + " Hz");
}

void updateLfoRate() {
  showCurrentParameterPage("LFO Rate", String(LfoRatestr) + " Hz");
}

void updatepwLFO() {
  showCurrentParameterPage("PWM Rate", String(pwLFOstr) + " Hz");
}

void updateosc2level() {
  showCurrentParameterPage("OSC2 Level", int(osc2levelstr));
}

void updateosc1level() {
  showCurrentParameterPage("OSC1 Level", int(osc1levelstr));
}

void updateosc2interval() {
  showCurrentParameterPage("OSC2 Interval", int(osc2intervalstr));
}

void updateosc1PW() {
  showCurrentParameterPage("OSC1 PW", String(osc1PWstr) + " %");
}

void updateosc2PW() {
  showCurrentParameterPage("OSC2 PW", String(osc2PWstr) + " %");
}

void updateosc1PWM() {
  showCurrentParameterPage("OSC1 PWM", int(osc1PWMstr));
}

void updateosc2PWM() {
  showCurrentParameterPage("OSC2 PWM", int(osc2PWMstr));
}

void updateampAttack() {
  if (ampAttackstr < 1000) {
    showCurrentParameterPage("Amp Attack", String(int(ampAttackstr)) + " ms", AMP_ENV);
  } else {
    showCurrentParameterPage("Amp Attack", String(ampAttackstr * 0.001) + " s", AMP_ENV);
  }
}

void updateampDecay() {
  if (ampDecaystr < 1000) {
    showCurrentParameterPage("Amp Decay", String(int(ampDecaystr)) + " ms", AMP_ENV);
  } else {
    showCurrentParameterPage("Amp Decay", String(ampDecaystr * 0.001) + " s", AMP_ENV);
  }
}

void updateampSustain() {
  showCurrentParameterPage("Amp Sustain", String(ampSustainstr), AMP_ENV);
}

void updateampRelease() {
  if (ampReleasestr < 1000) {
    showCurrentParameterPage("Amp Release", String(int(ampReleasestr)) + " ms", AMP_ENV);
  } else {
    showCurrentParameterPage("Amp Release", String(ampReleasestr * 0.001) + " s", AMP_ENV);
  }
}

void updatefilterAttack() {
  if (filterAttackstr < 1000) {
    showCurrentParameterPage("Filter Attack", String(int(filterAttackstr)) + " ms", AMP_ENV);
  } else {
    showCurrentParameterPage("Filter Attack", String(filterAttackstr * 0.001) + " s", AMP_ENV);
  }
}

void updatefilterDecay() {
  if (filterDecaystr < 1000) {
    showCurrentParameterPage("Filter Decay", String(int(filterDecaystr)) + " ms", AMP_ENV);
  } else {
    showCurrentParameterPage("Filter Decay", String(filterDecaystr * 0.001) + " s", AMP_ENV);
  }
}

void updatefilterSustain() {
  showCurrentParameterPage("Filter Sustain", String(filterSustainstr), AMP_ENV);
}

void updatefilterRelease() {
  if (filterReleasestr < 1000) {
    showCurrentParameterPage("Filter Release", String(int(filterReleasestr)) + " ms", AMP_ENV);
  } else {
    showCurrentParameterPage("Filter Release", String(filterReleasestr * 0.001) + " s", AMP_ENV);
  }
}


void updatePatchname() {
  showPatchPage(String(patchNo), patchName);
}

void myControlChange(byte channel, byte control, int value) {

  //  Serial.println("MIDI: " + String(control) + " : " + String(value));
  //turnOffOneandTwo();

  switch (control) {

    case CCvolume:
      volume = value;
      volumestr = value / 8;
      updatevolume();
      break;

    case CCglide:
      glide = value;
      glidestr = value / 8;
      updateglide();
      break;

    case CCosc1_32:
      osc1_32 = 1;
      osc1_16 = 0;
      osc1_8 = 0;
      updateosc1_32();
      break;

    case CCosc1_16:
      osc1_32 = 0;
      osc1_16 = 1;
      osc1_8 = 0;
      updateosc1_16();
      break;

    case CCosc1_8:
      osc1_32 = 0;
      osc1_16 = 0;
      osc1_8 = 1;
      updateosc1_8();
      break;

    case CCosc1_saw:
      osc1_saw = 1;
      osc1_tri = 0;
      osc1_pulse = 0;
      updateosc1_saw();
      break;

    case CCosc1_tri:
      osc1_saw = 0;
      osc1_tri = 1;
      osc1_pulse = 0;
      updateosc1_tri();
      break;

    case CCosc1_pulse:
      osc1_saw = 0;
      osc1_tri = 0;
      osc1_pulse = 1;
      updateosc1_pulse();
      break;

    case CCsingle:
      single = 1;
      multi = 0;
      updatesingle();
      break;

    case CCmulti:
      multi = 1;
      single = 0;
      updatemulti();
      break;

    case CClfoTriangle:
      lfoTriangle = 1;
      lfoSquare = 0;
      updatelfoTriangle();
      break;

    case CClfoSquare:
      lfoSquare = 1;
      lfoTriangle = 0;
      updatelfoSquare();
      break;

    case CCsyncOff:
      syncOff = 1;
      syncOn = 0;
      updatesyncOff();
      break;

    case CCsyncOn:
      syncOn = 1;
      syncOff = 0;
      updatesyncOn();
      break;

    case CCoctave0:
      octave0 = 1;
      octave1 = 0;
      updateoctave0();
      break;

    case CCoctave1:
      octave0 = 0;
      octave1 = 1;
      updateoctave1();
      break;

    case CCkbOff:
      kbOff = 1;
      kbHalf = 0;
      kbFull = 0;
      updatekbOff();
      break;

    case CCkbHalf:
      kbOff = 0;
      kbHalf = 1;
      kbFull = 0;
      updatekbHalf();
      break;

    case CCkbFull:
      kbOff = 0;
      kbHalf = 0;
      kbFull = 1;
      updatekbFull();
      break;

    case CCosc2_32:
      osc2_32 = 1;
      osc2_16 = 0;
      osc2_8 = 0;
      updateosc2_32();
      break;

    case CCosc2_16:
      osc2_32 = 0;
      osc2_16 = 1;
      osc2_8 = 0;
      updateosc2_16();
      break;

    case CCosc2_8:
      osc2_32 = 0;
      osc2_16 = 0;
      osc2_8 = 1;
      updateosc2_8();
      break;

    case CCosc2_saw:
      osc2_saw = 1;
      osc2_tri = 0;
      osc2_pulse = 0;
      updateosc2_saw();
      break;

    case CCosc2_tri:
      osc2_saw = 0;
      osc2_tri = 1;
      osc2_pulse = 0;
      updateosc2_tri();
      break;

    case CCosc2_pulse:
      osc2_saw = 0;
      osc2_tri = 0;
      osc2_pulse = 1;
      updateosc2_pulse();
      break;

    case CClfoOscOff:
      lfoOscOff = 1;
      lfoOscOn = 0;
      updatelfoOscOff();
      break;

    case CClfoOscOn:
      lfoOscOff = 0;
      lfoOscOn = 1;
      updatelfoOscOn();
      break;

    case CClfoVCFOff:
      lfoVCFOff = 1;
      lfoVCFOn = 0;
      updatelfoVCFOff();
      break;

    case CClfoVCFOn:
      lfoVCFOff = 0;
      lfoVCFOn = 1;
      updatelfoVCFOn();
      break;

    case CClevel1:
      level1 = 1;
      level2 = 0;
      updatelevel1();
      break;

    case CClevel2:
      level2 = 1;
      level1 = 0;
      updatelevel2();
      break;

    case CCbutton1:
      button1 = 1;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton1();
      break;

    case CCbutton2:
      button1 = 0;
      button2 = 1;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton2();
      break;

    case CCbutton3:
      button1 = 0;
      button2 = 0;
      button3 = 1;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton3();
      break;

    case CCbutton4:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 1;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton4();
      break;

    case CCbutton5:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 1;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton5();
      break;

    case CCbutton6:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 1;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton6();
      break;

    case CCbutton7:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 1;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton7();
      break;

    case CCbutton8:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 1;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton8();
      break;

    case CCbutton9:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 1;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton9();
      break;

    case CCbutton10:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 1;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton10();
      break;

    case CCbutton11:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 1;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton11();
      break;

    case CCbutton12:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 1;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton12();
      break;

    case CCbutton13:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 1;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton13();
      break;

    case CCbutton14:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 1;
      button15 = 0;
      button16 = 0;
      updatebutton14();
      break;

    case CCbutton15:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 1;
      button16 = 0;
      updatebutton15();
      break;

    case CCbutton16:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 1;
      updatebutton16();
      break;

    case CCnoiseLevel:
      noiseLevel = value;
      noiseLevelstr = int(value / 8);
      updateNoiseLevel();
      break;

    case CCfilterCutoff:
      filterCutoff = value;
      filterCutoffstr = FILTERCUTOFF[value / 8];
      updateFilterCutoff();
      break;

    case CCfilterRes:
      filterRes = value;
      filterResstr = int(value / 8);
      updatefilterRes();
      break;

    case CCfilterlevel:
      filterLevel = value;
      filterLevelstr = int(value / 8);
      updatefilterLevel();
      break;

    case CCosc1PW:
      osc1PW = value;
      osc1PWstr = PULSEWIDTH[value / 8];
      updateosc1PW();
      break;

    case CCosc2PW:
      osc2PW = value;
      osc2PWstr = PULSEWIDTH[value / 8];
      updateosc2PW();
      break;

    case CCosc1PWM:
      osc1PWM = value;
      osc1PWMstr = int(value / 8);
      updateosc1PWM();
      break;

    case CCosc2PWM:
      osc2PWM = value;
      osc2PWMstr = int(value / 8);
      updateosc2PWM();
      break;

    case CCLfoRate:
      LfoRatestr = LFOTEMPO[value / 8];  // for display
      LfoRate = value;
      updateLfoRate();
      break;

    case CCpwLFO:
      pwLFO = value;
      pwLFOstr = LFOTEMPO[value / 8];  // for display
      updatepwLFO();
      break;

    case CCosc2level:
      osc2level = value;
      osc2levelstr = value / 8;  // for display
      updateosc2level();
      break;

    case CCosc1level:
      osc1level = value;
      osc1levelstr = value / 8;  // for display
      updateosc1level();
      break;

    case CCosc2interval:
      osc2intervalstr = INTERVAL[value / 8];
      osc2interval = value;
      updateosc2interval();
      break;

    case CCampAttack:
      ampAttack = value;
      ampAttackstr = ENVTIMES[value / 8];
      updateampAttack();
      break;

    case CCampDecay:
      ampDecay = value;
      ampDecaystr = ENVTIMES[value / 8];
      updateampDecay();
      break;

    case CCampSustain:
      ampSustain = value;
      ampSustainstr = LINEAR_FILTERMIXERSTR[value / 8];
      updateampSustain();
      break;

    case CCampRelease:
      ampRelease = value;
      ampReleasestr = ENVTIMES[value / 8];
      updateampRelease();
      break;

    case CCfilterAttack:
      filterAttack = value;
      filterAttackstr = ENVTIMES[value / 8];
      updatefilterAttack();
      break;

    case CCfilterDecay:
      filterDecay = value;
      filterDecaystr = ENVTIMES[value / 8];
      updatefilterDecay();
      break;

    case CCfilterSustain:
      filterSustain = value;
      filterSustainstr = LINEAR_FILTERMIXERSTR[value / 8];
      updatefilterSustain();
      break;

    case CCfilterRelease:
      filterRelease = value;
      filterReleasestr = ENVTIMES[value / 8];
      updatefilterRelease();
      break;

    case CCallnotesoff:
      allNotesOff();
      break;
  }
}

void myProgramChange(byte channel, byte program) {
  state = PATCH;
  patchNo = program + 1;
  recallPatch(patchNo);
  Serial.print("MIDI Pgm Change:");
  Serial.println(patchNo);
  state = PARAMETER;
}

void recallPatch(int patchNo) {
  allNotesOff();
  File patchFile = SD.open(String(patchNo).c_str());
  if (!patchFile) {
    Serial.println("File not found");
  } else {
    String data[NO_OF_PARAMS];  //Array of data read in
    recallPatchData(patchFile, data);
    setCurrentPatchData(data);
    patchFile.close();
    storeLastPatch(patchNo);
    showPatchNumberButton();
    updatelevel2();
  }
}

void setCurrentPatchData(String data[]) {
  patchName = data[0];
  noiseLevel = data[1].toInt();
  glide = data[2].toInt();
  osc1_32 = data[3].toInt();
  osc1_16 = data[4].toInt();
  osc1_8 = data[5].toInt();
  osc1_saw = data[6].toInt();
  osc1_tri = data[7].toInt();
  osc1_pulse = data[8].toInt();
  osc2_32 = data[9].toInt();
  osc2_16 = data[10].toInt();
  osc2_8 = data[11].toInt();
  osc2_saw = data[12].toInt();
  osc2_tri = data[13].toInt();
  osc2_pulse = data[14].toInt();
  single = data[15].toInt();
  multi = data[16].toInt();
  lfoTriangle = data[17].toInt();
  lfoSquare = data[18].toInt();
  lfoOscOff = data[19].toInt();
  lfoOscOn = data[20].toInt();
  lfoVCFOff = data[21].toInt();
  lfoVCFOn = data[22].toInt();
  syncOff = data[23].toInt();
  syncOn = data[24].toInt();
  kbOff = data[25].toInt();
  kbHalf = data[26].toInt();
  kbFull = data[27].toInt();
  LfoRate = data[28].toFloat();
  pwLFO = data[29].toFloat();
  osc1level = data[30].toInt();
  osc2level = data[31].toInt();
  osc1PW = data[32].toInt();
  osc2PW = data[33].toInt();
  osc1PWM = data[34].toInt();
  osc2PWM = data[35].toInt();
  ampAttack = data[36].toInt();
  ampDecay = data[37].toInt();
  ampSustain = data[38].toInt();
  ampRelease = data[39].toInt();
  osc2interval = data[40].toInt();
  filterAttack = data[41].toInt();
  filterDecay = data[42].toInt();
  filterSustain = data[43].toInt();
  filterRelease = data[44].toInt();
  filterRes = data[45].toInt();
  filterCutoff = data[46].toFloat();
  filterLevel = data[47].toInt();
  osc1foot = data[48].toInt();
  osc2foot = data[49].toInt();
  octave0 = data[50].toInt();
  octave1 = data[51].toInt();
  shvco = data[52].toInt();
  shvcf = data[53].toInt();
  vcfVelocity = data[54].toInt();
  vcaVelocity = data[55].toInt();
  vcfLoop = data[56].toInt();
  vcaLoop = data[57].toInt();
  vcfLinear = data[58].toInt();
  vcaLinear = data[59].toInt();
  keyMode = data[60].toInt();
  modWheelDepth = data[61].toInt();
  pitchBendRange = data[62].toInt();
  volume = data[63].toInt();

  //Switches
  updateosc1_32();
  updateosc1_16();
  updateosc1_8();
  updateosc1_saw();
  updateosc1_tri();
  updateosc1_pulse();
  updatesingle();
  updatemulti();
  updatelfoTriangle();
  updatelfoSquare();
  updatesyncOff();
  updatesyncOn();
  updateoctave0();
  updateoctave1();
  updatekbOff();
  updatekbHalf();
  updatekbFull();
  updateosc2_32();
  updateosc2_16();
  updateosc2_8();
  updateosc2_saw();
  updateosc2_tri();
  updateosc2_pulse();
  updatelfoOscOff();
  updatelfoOscOn();
  updatelfoVCFOff();
  updatelfoVCFOn();
  updateshvco();
  updateshvcf();
  updatevcfVelocity();
  updatevcaVelocity();
  updatevcfLoop();
  updatevcaLoop();
  updatevcfLinear();
  updatevcaLinear();


  //Patchname
  updatePatchname();

  Serial.print("Set Patch: ");
  Serial.println(patchName);
}

String getCurrentPatchData() {
  return patchName + "," + String(noiseLevel) + "," + String(glide) + "," + String(osc1_32) + "," + String(osc1_16) + "," + String(osc1_8) + "," + String(osc1_saw) + "," + String(osc1_tri) + "," + String(osc1_pulse) + "," + String(osc2_32) + "," + String(osc2_16) + "," + String(osc2_8) + "," + String(osc2_saw) + "," + String(osc2_tri) + "," + String(osc2_pulse) + "," + String(single) + "," + String(multi) + "," + String(lfoTriangle) + "," + String(lfoSquare) + "," + String(lfoOscOff) + "," + String(lfoOscOn) + "," + String(lfoVCFOff) + "," + String(lfoVCFOn) + "," + String(syncOff) + "," + String(syncOn) + "," + String(kbOff) + "," + String(kbHalf) + "," + String(kbFull) + "," + String(LfoRate) + "," + String(pwLFO) + "," + String(osc1level) + "," + String(osc2level) + "," + String(osc1PW) + "," + String(osc2PW) + "," + String(osc1PWM) + "," + String(osc2PWM) + "," + String(ampAttack) + "," + String(ampDecay) + "," + String(ampSustain) + "," + String(ampRelease) + "," + String(osc2interval) + "," + String(filterAttack) + "," + String(filterDecay) + "," + String(filterSustain) + "," + String(filterRelease) + "," + String(filterRes) + "," + String(filterCutoff) + "," + String(filterLevel) + "," + String(osc1foot) + "," + String(osc2foot) + "," + String(octave0) + "," + String(octave1) + "," + String(shvco) + "," + String(shvcf) + "," + String(vcfVelocity) + "," + String(vcaVelocity) + "," + String(vcfLoop) + "," + String(vcaLoop) + "," + String(vcfLinear) + "," + String(vcaLinear) + "," + String(keyMode) + "," + String(modWheelDepth) + "," + String(pitchBendRange) + "," + String(volume);
}

void checkMux() {

  mux1Read = adc->adc1->analogRead(MUX1_S);
  mux2Read = adc->adc1->analogRead(MUX2_S);

  if (mux1Read > (mux1ValuesPrev[muxInput] + QUANTISE_FACTOR) || mux1Read < (mux1ValuesPrev[muxInput] - QUANTISE_FACTOR)) {
    mux1ValuesPrev[muxInput] = mux1Read;

    switch (muxInput) {
      case MUX1_KBGLIDE:
        myControlChange(midiChannel, CCglide, mux1Read);
        break;
      case MUX1_LFORATE:
        myControlChange(midiChannel, CCLfoRate, mux1Read);
        break;
      case MUX1_OSC1LEVEL:
        myControlChange(midiChannel, CCosc1level, mux1Read);
        break;
      case MUX1_CUTOFF:
        myControlChange(midiChannel, CCfilterCutoff, mux1Read);
        break;
      case MUX1_EMPHASIS:
        myControlChange(midiChannel, CCfilterRes, mux1Read);
        break;
      case MUX1_CONTOURAMT:
        myControlChange(midiChannel, CCfilterlevel, mux1Read);
        break;
      case MUX1_OSC1PW:
        myControlChange(midiChannel, CCosc1PW, mux1Read);
        break;
      case MUX1_OSC1PWM:
        myControlChange(midiChannel, CCosc1PWM, mux1Read);
        break;
      case MUX1_NOISE:
        myControlChange(midiChannel, CCnoiseLevel, mux1Read);
        break;
      case MUX1_VOLUME:
        myControlChange(midiChannel, CCvolume, mux1Read);
        break;
    }
  }

  if (mux2Read > (mux2ValuesPrev[muxInput] + QUANTISE_FACTOR) || mux2Read < (mux2ValuesPrev[muxInput] - QUANTISE_FACTOR)) {
    mux2ValuesPrev[muxInput] = mux2Read;

    switch (muxInput) {
      case MUX2_OSC2LEVEL:
        myControlChange(midiChannel, CCosc2level, mux2Read);
        break;
      case MUX2_FILTERATTACK:
        myControlChange(midiChannel, CCfilterAttack, mux2Read);
        break;
      case MUX2_FILTERDECAY:
        myControlChange(midiChannel, CCfilterDecay, mux2Read);
        break;
      case MUX2_FILTERSUSTAIN:
        myControlChange(midiChannel, CCfilterSustain, mux2Read);
        break;
      case MUX2_FILTERRELEASE:
        myControlChange(midiChannel, CCfilterRelease, mux2Read);
        break;
      case MUX2_INTERVAL:
        myControlChange(midiChannel, CCosc2interval, mux2Read);
        break;
      case MUX2_OSC2PW:
        myControlChange(midiChannel, CCosc2PW, mux2Read);
        break;
      case MUX2_OSC2PWM:
        myControlChange(midiChannel, CCosc2PWM, mux2Read);
        break;
      case MUX2_AMPATTACK:
        myControlChange(midiChannel, CCampAttack, mux2Read);
        break;
      case MUX2_AMPDECAY:
        myControlChange(midiChannel, CCampDecay, mux2Read);
        break;
      case MUX2_AMPSUSTAIN:
        myControlChange(midiChannel, CCampSustain, mux2Read);
        break;
      case MUX2_AMPRELEASE:
        myControlChange(midiChannel, CCampRelease, mux2Read);
        break;
      case MUX2_PWLFORATE:
        myControlChange(midiChannel, CCpwLFO, mux2Read);
        break;
    }
  }

  muxInput++;
  if (muxInput >= MUXCHANNELS)
    muxInput = 0;

  digitalWriteFast(MUX_0, muxInput & B0001);
  digitalWriteFast(MUX_1, muxInput & B0010);
  digitalWriteFast(MUX_2, muxInput & B0100);
  digitalWriteFast(MUX_3, muxInput & B1000);
}

void writeDemux() {
  delayMicroseconds(DelayForSH3);

  //DEMUX 1
  switch (muxOutput) {
    case 0:  // 5volt
      setVoltage(DAC_NOTE1, 0, 1, int(filterAttack * 1.85));
      setVoltage(DAC_NOTE1, 1, 1, int(filterDecay * 1.85));
      break;
    case 1:  // 5Volt
      setVoltage(DAC_NOTE1, 0, 1, int(filterSustain * 1.85));
      setVoltage(DAC_NOTE1, 1, 1, int(filterRelease * 1.85));
      break;
    case 2:  // 5Volt
      setVoltage(DAC_NOTE1, 0, 1, int(ampAttack * 1.85));
      setVoltage(DAC_NOTE1, 1, 1, int(ampDecay * 1.85));
      break;
    case 3:  // 5Volt
      setVoltage(DAC_NOTE1, 0, 1, int(ampSustain * 1.85));
      setVoltage(DAC_NOTE1, 1, 1, int(ampRelease * 1.85));
      break;
    case 4:  // 5Volt
      setVoltage(DAC_NOTE1, 0, 1, int(LfoRate * 1.85));
      setVoltage(DAC_NOTE1, 1, 1, int(pwLFO * 1.85));
      break;
    case 5:  // 2Volt
      setVoltage(DAC_NOTE1, 0, 1, modulation << 4);
      setVoltage(DAC_NOTE1, 1, 1, int(LfoWave * 1.85));  //5v
      break;
    case 6:  // 2Volt
      setVoltage(DAC_NOTE1, 0, 1, int(osc1PWM / 1.07));
      setVoltage(DAC_NOTE1, 1, 1, int(osc2PWM / 1.07));
      break;
    case 7:  // 2Volt
      setVoltage(DAC_NOTE1, 0, 1, int((pitchbend / 8) + 1024));
      setVoltage(DAC_NOTE1, 1, 1, int(noiseLevel * 2));
      break;
    case 8:  // 10 Volt
      setVoltage(DAC_NOTE1, 0, 1, int(filterCutoff * 1.9));
      setVoltage(DAC_NOTE1, 1, 1, int(filterRes * 1.9));
      break;
    case 9:  // 6 Volt
      setVoltage(DAC_NOTE1, 0, 1, int((osc1PW * 1.22) + 50));
      setVoltage(DAC_NOTE1, 1, 1, int((osc2PW * 1.22) + 50));
      break;
    case 10:  // 2 Volt
      setVoltage(DAC_NOTE1, 0, 1, int(osc1level * 2));
      setVoltage(DAC_NOTE1, 1, 1, int(osc2level * 2));
      break;
    case 11:
      // 0-2V
      if (osc2interval < 9) {
        setVoltage(DAC_NOTE1, 0, 1, 0);
      } else {
        setVoltage(DAC_NOTE1, 0, 1, int(osc2interval * 2));
      }
      // 10 Volt
      setVoltage(DAC_NOTE1, 1, 1, int(glide * 1.9));
      break;
    case 12:
      // 0-5V
      setVoltage(DAC_NOTE1, 0, 1, int(filterLevel * 1.85));
      setVoltage(DAC_NOTE1, 1, 1, int(volume * 2));
      break;
    case 13:
      setVoltage(DAC_NOTE1, 0, 0, int(osc1foot));
      setVoltage(DAC_NOTE1, 1, 0, int(osc2foot));
      break;
    case 14:
      break;
    case 15:
      break;
  }
  delayMicroseconds(DelayForSH3);

  muxOutput++;
  if (muxOutput >= DEMUXCHANNELS)
    muxOutput = 0;

  digitalWriteFast(DEMUX_0, muxOutput & B0001);
  digitalWriteFast(DEMUX_1, muxOutput & B0010);
  digitalWriteFast(DEMUX_2, muxOutput & B0100);
  digitalWriteFast(DEMUX_3, muxOutput & B1000);
}

void showSettingsPage() {
  showSettingsPage(settings::current_setting(), settings::current_setting_value(), state);
}

void onButtonPress(uint16_t btnIndex, uint8_t btnType) {

  if (btnIndex == OSC1_32 && btnType == ROX_PRESSED) {
    osc1_32switch = !osc1_32switch;
    myControlChange(midiChannel, CCosc1_32, osc1_32switch);
  }

  if (btnIndex == OSC1_16 && btnType == ROX_PRESSED) {
    osc1_16switch = !osc1_16switch;
    myControlChange(midiChannel, CCosc1_16, osc1_16switch);
  }

  if (btnIndex == OSC1_8 && btnType == ROX_PRESSED) {
    osc1_8switch = !osc1_8switch;
    myControlChange(midiChannel, CCosc1_8, osc1_8switch);
  }

  if (btnIndex == OSC1_SAW && btnType == ROX_PRESSED) {
    osc1_sawswitch = !osc1_sawswitch;
    myControlChange(midiChannel, CCosc1_saw, osc1_sawswitch);
  }

  if (btnIndex == OSC1_TRI && btnType == ROX_PRESSED) {
    osc1_triswitch = !osc1_triswitch;
    myControlChange(midiChannel, CCosc1_tri, osc1_triswitch);
  }

  if (btnIndex == OSC1_PULSE && btnType == ROX_PRESSED) {
    osc1_pulseswitch = !osc1_pulseswitch;
    myControlChange(midiChannel, CCosc1_pulse, osc1_pulseswitch);
  }

  if (btnIndex == SINGLE_TRIG && btnType == ROX_PRESSED) {
    singleswitch = !singleswitch;
    myControlChange(midiChannel, CCsingle, singleswitch);
  }

  if (btnIndex == MULTIPLE_TRIG && btnType == ROX_PRESSED) {
    multiswitch = !multiswitch;
    myControlChange(midiChannel, CCmulti, multiswitch);
  }

  if (btnIndex == LFO_TRIANGLE && btnType == ROX_PRESSED) {
    lfoTriangleswitch = !lfoTriangleswitch;
    myControlChange(midiChannel, CClfoTriangle, lfoTriangleswitch);
  }

  if (btnIndex == LFO_SQUARE && btnType == ROX_PRESSED) {
    lfoSquareswitch = !lfoSquareswitch;
    myControlChange(midiChannel, CClfoSquare, lfoSquareswitch);
  }

  if (btnIndex == SYNC_OFF && btnType == ROX_PRESSED) {
    syncOffswitch = 1;
    syncOnswitch = 0;
    myControlChange(midiChannel, CCsyncOff, syncOffswitch);
  }

  if (btnIndex == SYNC_ON && btnType == ROX_PRESSED) {
    syncOnswitch = 1;
    syncOffswitch = 0;
    myControlChange(midiChannel, CCsyncOn, syncOnswitch);
  }

  if (btnIndex == OCTAVE_0 && btnType == ROX_PRESSED) {
    octave0switch = !octave0switch;
    myControlChange(midiChannel, CCoctave0, octave0switch);
  }

  if (btnIndex == OCTAVE_1 && btnType == ROX_PRESSED) {
    octave1switch = !octave1switch;
    myControlChange(midiChannel, CCoctave1, octave1switch);
  }

  if (btnIndex == KB_OFF && btnType == ROX_PRESSED) {
    kbOffswitch = !kbOffswitch;
    myControlChange(midiChannel, CCkbOff, kbOffswitch);
  }

  if (btnIndex == KB_HALF && btnType == ROX_PRESSED) {
    kbHalfswitch = !kbHalfswitch;
    myControlChange(midiChannel, CCkbHalf, kbHalfswitch);
  }

  if (btnIndex == KB_FULL && btnType == ROX_PRESSED) {
    kbFullswitch = !kbFullswitch;
    myControlChange(midiChannel, CCkbFull, kbFullswitch);
  }

  if (btnIndex == OSC2_32 && btnType == ROX_PRESSED) {
    osc2_32switch = !osc2_32switch;
    myControlChange(midiChannel, CCosc2_32, osc2_32switch);
  }

  if (btnIndex == OSC2_16 && btnType == ROX_PRESSED) {
    osc2_16switch = !osc2_16switch;
    myControlChange(midiChannel, CCosc2_16, osc2_16switch);
  }

  if (btnIndex == OSC2_8 && btnType == ROX_PRESSED) {
    osc2_8switch = !osc2_8switch;
    myControlChange(midiChannel, CCosc2_8, osc2_8switch);
  }

  if (btnIndex == OSC2_SAW && btnType == ROX_PRESSED) {
    osc2_sawswitch = !osc2_sawswitch;
    myControlChange(midiChannel, CCosc2_saw, osc2_sawswitch);
  }

  if (btnIndex == OSC2_TRI && btnType == ROX_PRESSED) {
    osc2_triswitch = !osc2_triswitch;
    myControlChange(midiChannel, CCosc2_tri, osc2_triswitch);
  }

  if (btnIndex == OSC2_PULSE && btnType == ROX_PRESSED) {
    osc2_pulseswitch = !osc2_pulseswitch;
    myControlChange(midiChannel, CCosc2_pulse, osc2_pulseswitch);
  }

  if (btnIndex == LFO_OSC_OFF && btnType == ROX_PRESSED) {
    lfoOscOffswitch = !lfoOscOffswitch;
    myControlChange(midiChannel, CClfoOscOff, lfoOscOffswitch);
  }

  if (btnIndex == LFO_OSC_ON && btnType == ROX_PRESSED) {
    lfoOscOnswitch = !lfoOscOnswitch;
    myControlChange(midiChannel, CClfoOscOn, lfoOscOnswitch);
  }

  if (btnIndex == LFO_VCF_OFF && btnType == ROX_PRESSED) {
    lfoVCFOffswitch = !lfoVCFOffswitch;
    myControlChange(midiChannel, CClfoVCFOff, lfoVCFOffswitch);
  }

  if (btnIndex == LFO_VCF_ON && btnType == ROX_PRESSED) {
    lfoVCFOnswitch = !lfoVCFOnswitch;
    myControlChange(midiChannel, CClfoVCFOn, lfoVCFOnswitch);
  }

  if (btnIndex == LEVEL1 && btnType == ROX_PRESSED) {
    level1switch = !level1switch;
    myControlChange(midiChannel, CClevel1, level1switch);
  }

  if (btnIndex == LEVEL2 && btnType == ROX_PRESSED) {
    level2switch = !level2switch;
    myControlChange(midiChannel, CClevel2, level2switch);
  }

  if (btnIndex == BUTTON1 && btnType == ROX_PRESSED) {
    button1switch = !button1switch;
    myControlChange(midiChannel, CCbutton1, button1switch);
  }

  if (btnIndex == BUTTON2 && btnType == ROX_PRESSED) {
    button2switch = !button2switch;
    myControlChange(midiChannel, CCbutton2, button2switch);
  }

  if (btnIndex == BUTTON3 && btnType == ROX_PRESSED) {
    button3switch = !button3switch;
    myControlChange(midiChannel, CCbutton3, button3switch);
  }

  if (btnIndex == BUTTON4 && btnType == ROX_PRESSED) {
    button4switch = !button4switch;
    myControlChange(midiChannel, CCbutton4, button4switch);
  }

  if (btnIndex == BUTTON5 && btnType == ROX_PRESSED) {
    button5switch = !button5switch;
    myControlChange(midiChannel, CCbutton5, button5switch);
  }

  if (btnIndex == BUTTON6 && btnType == ROX_PRESSED) {
    button6switch = !button6switch;
    myControlChange(midiChannel, CCbutton6, button6switch);
  }

  if (btnIndex == BUTTON7 && btnType == ROX_PRESSED) {
    button7switch = !button7switch;
    myControlChange(midiChannel, CCbutton7, button7switch);
  }

  if (btnIndex == BUTTON8 && btnType == ROX_PRESSED) {
    button8switch = !button8switch;
    myControlChange(midiChannel, CCbutton8, button8switch);
  }

  if (btnIndex == BUTTON9 && btnType == ROX_PRESSED) {
    button9switch = !button9switch;
    myControlChange(midiChannel, CCbutton9, button9switch);
  }

  if (btnIndex == BUTTON10 && btnType == ROX_PRESSED) {
    button10switch = !button10switch;
    myControlChange(midiChannel, CCbutton10, button10switch);
  }

  if (btnIndex == BUTTON11 && btnType == ROX_PRESSED) {
    button11switch = !button11switch;
    myControlChange(midiChannel, CCbutton11, button11switch);
  }

  if (btnIndex == BUTTON12 && btnType == ROX_PRESSED) {
    button12switch = !button12switch;
    myControlChange(midiChannel, CCbutton12, button12switch);
  }

  if (btnIndex == BUTTON13 && btnType == ROX_PRESSED) {
    button13switch = !button13switch;
    myControlChange(midiChannel, CCbutton13, button13switch);
  }

  if (btnIndex == BUTTON14 && btnType == ROX_PRESSED) {
    button14switch = !button14switch;
    myControlChange(midiChannel, CCbutton14, button14switch);
  }

  if (btnIndex == BUTTON15 && btnType == ROX_PRESSED) {
    button15switch = !button15switch;
    myControlChange(midiChannel, CCbutton15, button15switch);
  }

  if (btnIndex == BUTTON16 && btnType == ROX_PRESSED) {
    button16switch = !button16switch;
    myControlChange(midiChannel, CCbutton16, button16switch);
  }
}

void checkEEProm() {

  if (oldclocksource != clocksource) {

    switch (clocksource) {
      case 0:
        boardswitch.writePin(CLOCK_SOURCE, LOW);
        break;

      case 1:
        boardswitch.writePin(CLOCK_SOURCE, HIGH);
        break;
    }
    oldclocksource = clocksource;
  }
}

void checkSwitches() {

  saveButton.update();
  if (saveButton.read() == LOW && saveButton.duration() > HOLD_DURATION) {
    switch (state) {
      case PARAMETER:
      case PATCH:
        state = DELETE;
        saveButton.write(HIGH);  //Come out of this state
        del = true;              //Hack
        break;
    }
  } else if (saveButton.risingEdge()) {
    if (!del) {
      switch (state) {
        case PARAMETER:
          if (patches.size() < PATCHES_LIMIT) {
            resetPatchesOrdering();  //Reset order of patches from first patch
            patches.push({ patches.size() + 1, INITPATCHNAME });
            state = SAVE;
          }
          break;
        case SAVE:
          //Save as new patch with INITIALPATCH name or overwrite existing keeping name - bypassing patch renaming
          patchName = patches.last().patchName;
          state = PATCH;
          savePatch(String(patches.last().patchNo).c_str(), getCurrentPatchData());
          showPatchPage(patches.last().patchNo, patches.last().patchName);
          patchNo = patches.last().patchNo;
          loadPatches();  //Get rid of pushed patch if it wasn't saved
          setPatchesOrdering(patchNo);
          renamedPatch = "";
          state = PARAMETER;
          break;
        case PATCHNAMING:
          if (renamedPatch.length() > 0) patchName = renamedPatch;  //Prevent empty strings
          state = PATCH;
          savePatch(String(patches.last().patchNo).c_str(), getCurrentPatchData());
          showPatchPage(patches.last().patchNo, patchName);
          patchNo = patches.last().patchNo;
          loadPatches();  //Get rid of pushed patch if it wasn't saved
          setPatchesOrdering(patchNo);
          renamedPatch = "";
          state = PARAMETER;
          break;
      }
    } else {
      del = false;
    }
  }

  settingsButton.update();
  if (settingsButton.held()) {
    //If recall held, set current patch to match current hardware state
    //Reinitialise all hardware values to force them to be re-read if different
    state = REINITIALISE;
    reinitialiseToPanel();
  } else if (settingsButton.numClicks() == 1) {
    switch (state) {
      case PARAMETER:
        state = SETTINGS;
        showSettingsPage();
        break;
      case SETTINGS:
        showSettingsPage();
      case SETTINGSVALUE:
        settings::save_current_value();
        state = SETTINGS;
        showSettingsPage();
        break;
    }
  }

  backButton.update();
  if (backButton.read() == LOW && backButton.duration() > HOLD_DURATION) {
    //If Back button held, Panic - all notes off
    allNotesOff();
    backButton.write(HIGH);              //Come out of this state
    panic = true;                        //Hack
  } else if (backButton.risingEdge()) {  //cannot be fallingEdge because holding button won't work
    if (!panic) {
      switch (state) {
        case RECALL:
          setPatchesOrdering(patchNo);
          state = PARAMETER;
          break;
        case SAVE:
          renamedPatch = "";
          state = PARAMETER;
          loadPatches();  //Remove patch that was to be saved
          setPatchesOrdering(patchNo);
          break;
        case PATCHNAMING:
          charIndex = 0;
          renamedPatch = "";
          state = SAVE;
          break;
        case DELETE:
          setPatchesOrdering(patchNo);
          state = PARAMETER;
          break;
        case SETTINGS:
          state = PARAMETER;
          break;
        case SETTINGSVALUE:
          state = SETTINGS;
          showSettingsPage();
          break;
      }
    } else {
      panic = false;
    }
  }

  //Encoder switch
  recallButton.update();
  if (recallButton.read() == LOW && recallButton.duration() > HOLD_DURATION) {
    //If Recall button held, return to current patch setting
    //which clears any changes made
    state = PATCH;
    //Recall the current patch
    patchNo = patches.first().patchNo;
    recallPatch(patchNo);
    state = PARAMETER;
    recallButton.write(HIGH);  //Come out of this state
    recall = true;             //Hack
  } else if (recallButton.risingEdge()) {
    if (!recall) {
      switch (state) {
        case PARAMETER:
          state = RECALL;  //show patch list
          break;
        case RECALL:
          state = PATCH;
          //Recall the current patch
          patchNo = patches.first().patchNo;
          recallPatch(patchNo);
          state = PARAMETER;
          break;
        case SAVE:
          showRenamingPage(patches.last().patchName);
          patchName = patches.last().patchName;
          state = PATCHNAMING;
          break;
        case PATCHNAMING:
          if (renamedPatch.length() < 13) {
            renamedPatch.concat(String(currentCharacter));
            charIndex = 0;
            currentCharacter = CHARACTERS[charIndex];
            showRenamingPage(renamedPatch);
          }
          break;
        case DELETE:
          //Don't delete final patch
          if (patches.size() > 1) {
            state = DELETEMSG;
            patchNo = patches.first().patchNo;     //PatchNo to delete from SD card
            patches.shift();                       //Remove patch from circular buffer
            deletePatch(String(patchNo).c_str());  //Delete from SD card
            loadPatches();                         //Repopulate circular buffer to start from lowest Patch No
            renumberPatchesOnSD();
            loadPatches();                      //Repopulate circular buffer again after delete
            patchNo = patches.first().patchNo;  //Go back to 1
            recallPatch(patchNo);               //Load first patch
          }
          state = PARAMETER;
          break;
        case SETTINGS:
          state = SETTINGSVALUE;
          showSettingsPage();
          break;
        case SETTINGSVALUE:
          settings::save_current_value();
          state = SETTINGS;
          showSettingsPage();
          break;
      }
    } else {
      recall = false;
    }
  }
}

void reinitialiseToPanel() {
  //This sets the current patch to be the same as the current hardware panel state - all the pots
  //The four button controls stay the same state
  //This reinialises the previous hardware values to force a re-read
  muxInput = 0;
  for (int i = 0; i < MUXCHANNELS; i++) {
    mux1ValuesPrev[i] = RE_READ;
    mux2ValuesPrev[i] = RE_READ;
  }
  patchName = INITPATCHNAME;
  showPatchPage("Initial", "Panel Settings");
}

void checkEncoder() {
  //Encoder works with relative inc and dec values
  //Detent encoder goes up in 4 steps, hence +/-3

  long encRead = encoder.read();
  if ((encCW && encRead > encPrevious + 3) || (!encCW && encRead < encPrevious - 3)) {
    switch (state) {
      case PARAMETER:
        state = PATCH;
        patches.push(patches.shift());
        patchNo = patches.first().patchNo;
        recallPatch(patchNo);
        state = PARAMETER;
        break;
      case RECALL:
        patches.push(patches.shift());
        break;
      case SAVE:
        patches.push(patches.shift());
        break;
      case PATCHNAMING:
        if (charIndex == TOTALCHARS) charIndex = 0;  //Wrap around
        currentCharacter = CHARACTERS[charIndex++];
        showRenamingPage(renamedPatch + currentCharacter);
        break;
      case DELETE:
        patches.push(patches.shift());
        break;
      case SETTINGS:
        settings::increment_setting();
        showSettingsPage();
        break;
      case SETTINGSVALUE:
        settings::increment_setting_value();
        showSettingsPage();
        break;
    }
    encPrevious = encRead;
  } else if ((encCW && encRead < encPrevious - 3) || (!encCW && encRead > encPrevious + 3)) {
    switch (state) {
      case PARAMETER:
        state = PATCH;
        patches.unshift(patches.pop());
        patchNo = patches.first().patchNo;
        recallPatch(patchNo);
        state = PARAMETER;
        break;
      case RECALL:
        patches.unshift(patches.pop());
        break;
      case SAVE:
        patches.unshift(patches.pop());
        break;
      case PATCHNAMING:
        if (charIndex == -1)
          charIndex = TOTALCHARS - 1;
        currentCharacter = CHARACTERS[charIndex--];
        showRenamingPage(renamedPatch + currentCharacter);
        break;
      case DELETE:
        patches.unshift(patches.pop());
        break;
      case SETTINGS:
        settings::decrement_setting();
        showSettingsPage();
        break;
      case SETTINGSVALUE:
        settings::decrement_setting_value();
        showSettingsPage();
        break;
    }
    encPrevious = encRead;
  }
}

void loop() {
  myusb.Task();
  midi1.read(midiChannel);  //USB HOST MIDI Class Compliant
  MIDI.read(midiChannel);
  usbMIDI.read(midiChannel);
  checkMux();
  writeDemux();
  boardswitch.update();
  mux.update();
  checkSwitches();
  checkEncoder();
  stopClockPulse();
  checkEEProm();
}


int mod(int a, int b) {
  int r = a % b;
  return r < 0 ? r + b : r;
}
